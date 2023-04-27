/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2023 Google LLC
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <sys/malloc.h>

#include "gve.h"
#include "gve_adminq.h"

static MALLOC_DEFINE(M_GVE_QPL, "gve qpl", "gve qpl allocations");

uint32_t
gve_num_tx_qpls(struct gve_priv *priv)
{
	if (priv->queue_format != GVE_GQI_QPL_FORMAT)
		return (0);

	return (priv->tx_cfg.num_queues);
}

uint32_t
gve_num_rx_qpls(struct gve_priv *priv)
{
	if (priv->queue_format != GVE_GQI_QPL_FORMAT)
		return (0);

	return (priv->rx_cfg.num_queues);
}

struct gve_queue_page_list *
gve_assign_tx_qpl(struct gve_priv *priv)
{
	int id = find_first_zero_bit(priv->qpl_cfg.qpl_id_map,
		     priv->qpl_cfg.qpl_map_size);

	/* we are out of tx qpls */
	if (id >= gve_num_tx_qpls(priv))
		return (NULL);

	set_bit(id, priv->qpl_cfg.qpl_id_map);
	return (&priv->qpls[id]);
}

struct gve_queue_page_list *
gve_assign_rx_qpl(struct gve_priv *priv)
{
	int id = find_next_zero_bit(priv->qpl_cfg.qpl_id_map,
	    priv->qpl_cfg.qpl_map_size, gve_num_tx_qpls(priv));

	/* we are out of rx qpls */
	if (id == gve_num_tx_qpls(priv) + gve_num_rx_qpls(priv))
		return (NULL);

	set_bit(id, priv->qpl_cfg.qpl_id_map);
	return (&priv->qpls[id]);
}

void
gve_unassign_qpl(struct gve_priv *priv, int id)
{
	clear_bit(id, priv->qpl_cfg.qpl_id_map);
}

static void
gve_free_qpl(struct gve_priv *priv, uint32_t id)
{
	struct gve_queue_page_list *qpl = &priv->qpls[id];
	int i;

	if (qpl->dmas != NULL && qpl->pages != NULL) {
		for (i = 0; i < qpl->num_entries; i++) {
			if (qpl->pages[i] != NULL) {
				gve_dma_free_coherent(&qpl->dmas[i]);
				priv->num_registered_pages--;
			}
		}
	}

	if (qpl->pages != NULL)
		free(qpl->pages, M_GVE_QPL);

	if (qpl->dmas != NULL)
		free(qpl->dmas, M_GVE_QPL);
}

static int
gve_alloc_qpl(struct gve_priv *priv, uint32_t id, int pages)
{
	struct gve_queue_page_list *qpl = &priv->qpls[id];
	int err;
	int i;

	if (pages + priv->num_registered_pages > priv->max_registered_pages) {
		device_printf(priv->dev, "Reached max number of registered pages %lu > %lu\n",
		    pages + priv->num_registered_pages,
		    priv->max_registered_pages);
		return (EINVAL);
	}

	qpl->id = id;
	qpl->num_entries = 0;

	qpl->dmas = malloc(pages * sizeof(*qpl->dmas), M_GVE_QPL,
		        M_WAITOK | M_ZERO);
	if (qpl->dmas == NULL)
		return (ENOMEM);

	qpl->pages = malloc(pages * sizeof(*qpl->pages), M_GVE_QPL,
			 M_WAITOK | M_ZERO);
	if (qpl->pages == NULL) {
		err = ENOMEM;
		goto abort;
	}

	for (i = 0; i < pages; i++) {
		err = gve_dma_alloc_coherent(priv, PAGE_SIZE, PAGE_SIZE,
			  &qpl->dmas[i], BUS_DMA_WAITOK | BUS_DMA_ZERO);
		if (err != 0) {
			device_printf(priv->dev, "Failed to allocate QPL page\n");
			err = ENOMEM;
			goto abort;
		}
		qpl->pages[i] = virt_to_page(qpl->dmas[i].cpu_addr);
		qpl->num_entries++;
		priv->num_registered_pages++;
	}

	return (0);

abort:
	gve_free_qpl(priv, id);
	return (err);
}

void
gve_free_qpls(struct gve_priv *priv)
{
	int num_qpls = gve_num_tx_qpls(priv) + gve_num_rx_qpls(priv);
	int i;

	if (num_qpls == 0)
		return;

	if (priv->qpls != NULL) {
		for (i = 0; i < num_qpls; i++)
			gve_free_qpl(priv, i);
		free(priv->qpls, M_GVE_QPL);
	}

	if (priv->qpl_cfg.qpl_id_map != NULL)
		free(priv->qpl_cfg.qpl_id_map, M_GVE_QPL);
}

int gve_alloc_qpls(struct gve_priv *priv)
{
	int num_qpls = gve_num_tx_qpls(priv) + gve_num_rx_qpls(priv);
	int err;
	int i;

	if (num_qpls == 0)
		return (0);

	priv->qpls = malloc(num_qpls * sizeof(*priv->qpls), M_GVE_QPL,
		         M_WAITOK | M_ZERO);
	if (priv->qpls == NULL)
		return (ENOMEM);

	for (i = 0; i < gve_num_tx_qpls(priv); i++) {
		err = gve_alloc_qpl(priv, i, priv->tx_desc_cnt / GVE_QPL_DIVISOR);
		if (err != 0)
			goto abort;
	}

	for (; i < num_qpls; i++) {
		err = gve_alloc_qpl(priv, i, priv->rx_desc_cnt);
		if (err != 0)
			goto abort;
	}

	priv->qpl_cfg.qpl_map_size = BITS_TO_LONGS(num_qpls) *
				     sizeof(unsigned long) * BITS_PER_BYTE;
	priv->qpl_cfg.qpl_id_map = malloc(BITS_TO_LONGS(num_qpls) *
					  sizeof(unsigned long), M_GVE_QPL,
					  M_WAITOK | M_ZERO);
	if (priv->qpl_cfg.qpl_id_map == 0) {
		err = ENOMEM;
		goto abort;
	}

	return (0);

abort:
	gve_free_qpls(priv);
	return (err);
}

static int
gve_unregister_n_qpls(struct gve_priv *priv, int n)
{
	int err;
	int i;

	for (i = 0; i < n; i++) {
		err = gve_adminq_unregister_page_list(priv, priv->qpls[i].id);
		if (err != 0) {
			device_printf(priv->dev,
			    "Failed to unregister qpl %d, err: %d\n",
			    priv->qpls[i].id, err);
		}
	}

	if (err != 0)
		return (err);

	return (0);
}

int
gve_register_qpls(struct gve_priv *priv)
{
	int num_qpls = gve_num_tx_qpls(priv) + gve_num_rx_qpls(priv);
	int err;
	int i;

	if (gve_get_state_flag(priv, GVE_STATE_FLAG_QPLREG_OK))
		return (0);

	for (i = 0; i < num_qpls; i++) {
		err = gve_adminq_register_page_list(priv, &priv->qpls[i]);
		if (err != 0) {
			device_printf(priv->dev,
			    "Failed to register qpl %d, err: %d\n",
			    priv->qpls[i].id, err);
			goto abort;
		}
	}

	gve_set_state_flag(priv, GVE_STATE_FLAG_QPLREG_OK);
	return (0);

abort:
	gve_unregister_n_qpls(priv, i);
	return (err);
}

int
gve_unregister_qpls(struct gve_priv *priv)
{
	int num_qpls = gve_num_tx_qpls(priv) + gve_num_rx_qpls(priv);
	int err;

	if (!gve_get_state_flag(priv, GVE_STATE_FLAG_QPLREG_OK))
		return (0);

	err = gve_unregister_n_qpls(priv, num_qpls);
	if (err != 0)
		return (err);

	gve_clear_state_flag(priv, GVE_STATE_FLAG_QPLREG_OK);
	return (0);
}
