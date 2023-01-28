/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2023 Google LLC
 * All rights reserved.
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
#include "gve.h"
#include "gve_adminq.h"

static int
gve_rx_alloc_ring(struct gve_priv *priv, int i)
{
	struct gve_rx_ring *rx = &priv->rx[i];
	struct gve_ring_com *com = &rx->com;
	int err;

	com->priv = priv;
	com->id = i;

	rx->mask = priv->rx_pages_per_qpl - 1;

	com->qpl = gve_assign_rx_qpl(priv);
	if (com->qpl == NULL) {
		device_printf(priv->dev, "No QPL left for rx ring %d i", i);
		return (ENOMEM);
	}

	err = gve_dma_alloc_coherent(priv, sizeof(struct gve_queue_resources),
	          PAGE_SIZE, &com->q_resources_mem,
		  BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT);
	if (err != 0) {
		device_printf(priv->dev, "Cannot alloc queue resources for rx ring %d", i);
		goto abort_with_qpl;
	}
	com->q_resources = com->q_resources_mem.cpu_addr;

	err = gve_dma_alloc_coherent(priv,
		  sizeof(struct gve_rx_desc) * priv->rx_desc_cnt,
		  CACHE_LINE_SIZE, &rx->desc_ring_mem,
		  BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT);
	if (err != 0) {
		device_printf(priv->dev, "Cannot alloc desc ring for rx ring %d", i);
		goto abort_with_q_resources;
	}
	rx->desc_ring = rx->desc_ring_mem.cpu_addr;

	err = gve_dma_alloc_coherent(priv,
		  sizeof(union gve_rx_data_slot) * priv->rx_desc_cnt,
		  CACHE_LINE_SIZE, &rx->data_ring_mem,
		  BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT);
	if (err != 0) {
		device_printf(priv->dev, "Cannot alloc data ring for rx ring %d", i);
		goto abort_with_desc_ring;
	}
	rx->data_ring = rx->data_ring_mem.cpu_addr;

	rx->page_info = malloc(priv->rx_desc_cnt * sizeof(*rx->page_info), M_GVE,
			    M_WAITOK | M_ZERO);
	if (rx->page_info == NULL) {
		err = (ENOMEM);
		goto abort_with_data_ring;
	}

	gve_prefill_rx_slots(rx);
	gve_alloc_counters((counter_u64_t *)&rx->stats, NUM_RX_STATS);

	rx->cnt = 0;
	rx->seq_no = 1;
	return (0);

abort_with_data_ring:
	gve_dma_free_coherent(&rx->data_ring_mem);
	rx->data_ring = NULL;
abort_with_desc_ring:
	gve_dma_free_coherent(&rx->desc_ring_mem);
	rx->desc_ring = NULL;
abort_with_q_resources:
	gve_dma_free_coherent(&com->q_resources_mem);
	com->q_resources = NULL;
abort_with_qpl:
	gve_unassign_qpl(priv, com->qpl->id);
	return (err);
}

static void
gve_rx_free_ring(struct gve_priv *priv, int i)
{
	struct gve_rx_ring *rx = &priv->rx[i];
	struct gve_ring_com *com = &rx->com;

	gve_free_counters((counter_u64_t *)&rx->stats, NUM_RX_STATS);

	free(rx->page_info, M_GVE);
	rx->page_info = NULL;

	gve_dma_free_coherent(&rx->data_ring_mem);
	rx->data_ring = NULL;

	gve_dma_free_coherent(&rx->desc_ring_mem);
	rx->desc_ring = NULL;

	gve_dma_free_coherent(&com->q_resources_mem);
	com->q_resources = NULL;

	gve_unassign_qpl(priv, com->qpl->id);
}

int
gve_alloc_rx_rings(struct gve_priv *priv)
{
	int err = 0;
	int i;

	priv->rx = malloc(sizeof(struct gve_rx_ring) * priv->rx_cfg.num_queues,
		       M_GVE, M_NOWAIT | M_ZERO);
	if (priv->rx == NULL) {
		device_printf(priv->dev, "Could not alloc rx ring array\n");
		return (ENOMEM);
	}

	for (i = 0; i < priv->rx_cfg.num_queues; i++) {
		err = gve_rx_alloc_ring(priv, i);
		if (err != 0) {
			device_printf(priv->dev, "Failed to alloc rx ring=%d: err=%d\n",
			    i, err);
			goto free_rings;
		}
	}

	return (0);

free_rings:
	while (i--) {
		gve_rx_free_ring(priv, i);
	}
	free(priv->rx, M_GVE);
	return (err);
}

void
gve_free_rx_rings(struct gve_priv *priv)
{
	int i;

	for (i = 0; i < priv->rx_cfg.num_queues; i++)
		gve_rx_free_ring(priv, i);

	free(priv->rx, M_GVE);
}

static void
gve_start_rx_ring(struct gve_priv *priv, int i)
{
	struct gve_rx_ring *rx = &priv->rx[i];
	struct gve_ring_com *com = &rx->com;

	if ((priv->ifp->if_capenable & IFCAP_LRO) != 0) {
		if (tcp_lro_init(&rx->lro) != 0)
			device_printf(priv->dev, "Cannot init lro for rx ring %d", i);
		rx->lro.ifp = priv->ifp;
	}

	NET_TASK_INIT(&com->cleanup_task, 0, gve_rx_cleanup_tq, rx);
	com->cleanup_tq = taskqueue_create_fast("gve rx", M_WAITOK,
			      taskqueue_thread_enqueue, &com->cleanup_tq);
	taskqueue_start_threads(&com->cleanup_tq, 1, PI_NET,
	    "%s rxq %d", device_get_nameunit(priv->dev), i);

	bus_dmamap_sync(rx->desc_ring_mem.tag, rx->desc_ring_mem.map,
	    BUS_DMASYNC_PREREAD);

	gve_db_bar_write_4(priv, com->db_offset, rx->fill_cnt);
}

int
gve_create_rx_rings(struct gve_priv *priv)
{
	struct gve_ring_com *com;
	struct gve_rx_ring *rx;
	int err;
	int i;

	if (gve_get_state_flag(priv, GVE_STATE_FLAG_RX_RINGS_OK))
		return (0);

	err = gve_adminq_create_rx_queues(priv, priv->rx_cfg.num_queues);
	if (err != 0)
		return (err);

	bus_dmamap_sync(priv->irqs_db_mem.tag, priv->irqs_db_mem.map,
	    BUS_DMASYNC_POSTREAD);

	for (i = 0; i < priv->rx_cfg.num_queues; i++) {
		rx = &priv->rx[i];
		com = &rx->com;

		com->irq_db_offset = 4 * be32toh(priv->irq_db_indices[com->ntfy_id].index);

		bus_dmamap_sync(com->q_resources_mem.tag, com->q_resources_mem.map,
		    BUS_DMASYNC_POSTREAD);
		com->db_offset = 4 * be32toh(com->q_resources->db_index);
		com->counter_idx = be32toh(com->q_resources->counter_index);

		gve_start_rx_ring(priv, i);
	}

	gve_set_state_flag(priv, GVE_STATE_FLAG_RX_RINGS_OK);
	return (0);
}

static void
gve_stop_rx_ring(struct gve_priv *priv, int i)
{
	struct gve_rx_ring *rx = &priv->rx[i];
	struct gve_ring_com *com = &rx->com;

	tcp_lro_free(&rx->lro);

	while (taskqueue_cancel(com->cleanup_tq, &com->cleanup_task, NULL))
		taskqueue_drain(com->cleanup_tq, &com->cleanup_task);
	taskqueue_free(com->cleanup_tq);
}

int
gve_destroy_rx_rings(struct gve_priv *priv)
{
	int err;
	int i;

	if (!gve_get_state_flag(priv, GVE_STATE_FLAG_RX_RINGS_OK))
		return (0);

	for (i = 0; i < priv->rx_cfg.num_queues; i++)
		gve_stop_rx_ring(priv, i);

	err = gve_adminq_destroy_rx_queues(priv, priv->rx_cfg.num_queues);
	if (err != 0)
		return (err);

	gve_clear_state_flag(priv, GVE_STATE_FLAG_RX_RINGS_OK);
	return (0);
}

void
gve_prefill_rx_slots(struct gve_rx_ring *rx)
{
	struct gve_ring_com *com = &rx->com;
	struct gve_dma_handle *dma;
	int i;

	for (i = 0; i < com->priv->rx_desc_cnt; i++) {
		rx->data_ring[i].qpl_offset = htobe64(PAGE_SIZE * i);
		rx->fill_cnt++;

		rx->page_info[i].page_offset = 0;
		rx->page_info[i].page_address = com->qpl->dmas[i].cpu_addr;
		rx->page_info[i].can_flip = true;

		dma = &com->qpl->dmas[i];
		bus_dmamap_sync(dma->tag, dma->map, BUS_DMASYNC_PREREAD);
	}

	bus_dmamap_sync(rx->data_ring_mem.tag, rx->data_ring_mem.map,
	    BUS_DMASYNC_PREWRITE);
}

static void
gve_rx_ctx_clear(struct gve_rx_ctx *ctx)
{
	ctx->mbuf_head = NULL;
	ctx->mbuf_tail = NULL;
	ctx->total_size = 0;
	ctx->frag_cnt = 0;
	ctx->drop_pkt = false;
}

int
gve_rx_intr(void *arg)
{
	struct gve_rx_ring *rx = arg;
	struct gve_priv *priv = rx->com.priv;
	struct gve_ring_com *com = &rx->com;

	if (unlikely((if_getdrvflags(priv->ifp) & IFF_DRV_RUNNING) == 0))
		return (FILTER_STRAY);

	gve_db_bar_write_4(priv, com->irq_db_offset, GVE_IRQ_MASK);
	taskqueue_enqueue(rx->com.cleanup_tq, &rx->com.cleanup_task);
	return (FILTER_HANDLED);
}

static inline void
gve_set_rss_type(__be16 flag, struct mbuf *mbuf)
{
	if ((flag & GVE_RXF_IPV4) != 0) {
		if ((flag & GVE_RXF_TCP) != 0)
			M_HASHTYPE_SET(mbuf, M_HASHTYPE_RSS_TCP_IPV4);
		else if ((flag & GVE_RXF_UDP) != 0)
			M_HASHTYPE_SET(mbuf, M_HASHTYPE_RSS_UDP_IPV4);
		else
			M_HASHTYPE_SET(mbuf, M_HASHTYPE_RSS_IPV4);
		return;
	}

	if ((flag & GVE_RXF_IPV6) != 0) {
		if ((flag & GVE_RXF_TCP) != 0)
			M_HASHTYPE_SET(mbuf, M_HASHTYPE_RSS_TCP_IPV6);
		else if ((flag & GVE_RXF_UDP) != 0)
			M_HASHTYPE_SET(mbuf, M_HASHTYPE_RSS_UDP_IPV6);
		else
			M_HASHTYPE_SET(mbuf, M_HASHTYPE_RSS_IPV6);
		return;
	}
}

static void
gve_mextadd_free(struct mbuf *mbuf)
{
	struct gve_rx_slot_page_info *page_info;

	page_info = (struct gve_rx_slot_page_info *)mbuf->m_ext.ext_arg1;
	if (likely(page_info != NULL))
		page_info->can_flip = true;
}

static void
gve_rx_flip_buff(struct gve_rx_slot_page_info *page_info, __be64 *slot_addr)
{
	const __be64 offset = htobe64(GVE_DEFAULT_RX_BUFFER_OFFSET);
	page_info->page_offset ^= GVE_DEFAULT_RX_BUFFER_OFFSET;
	*(slot_addr) ^= offset;
}

#define GVE_PKT_CONT_BIT_IS_SET(x) (GVE_RXF_PKT_CONT & x)

static struct mbuf *
gve_rx_create_mbuf(struct gve_priv *priv, struct gve_rx_ring *rx,
    struct gve_rx_slot_page_info *page_info, uint16_t len,
    union gve_rx_data_slot *data_slot, bool is_only_frag)
{
	struct gve_rx_ctx *ctx = &rx->ctx;
	struct mbuf *mbuf;

	uint32_t offset = page_info->page_offset + page_info->pad;
	void *va = (char*)page_info->page_address + offset;

	if (len <= priv->rx_copybreak && is_only_frag) {
		mbuf = m_get2(len, M_NOWAIT, MT_DATA, M_PKTHDR);
		if (unlikely(mbuf == NULL))
			return (NULL);

		m_copyback(mbuf, 0, len, va);
		counter_enter();
		counter_u64_add_protected(rx->stats.rx_copybreak_cnt, 1);
		counter_exit();
		ctx->mbuf_head = mbuf;
		ctx->mbuf_tail = mbuf;
	} else {
		struct mbuf *mbuf_tail = ctx->mbuf_tail;

		if (mbuf_tail == NULL) {
			if (page_info->can_flip)
				MGETHDR(mbuf, M_NOWAIT, MT_DATA);
			else
				mbuf = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);

			ctx->mbuf_head = mbuf;
			ctx->mbuf_tail = mbuf;
		} else {
			if (page_info->can_flip)
				MGET(mbuf, M_NOWAIT, MT_DATA);
			else
				mbuf = m_getcl(M_NOWAIT, MT_DATA, 0);

			mbuf_tail->m_next = mbuf;
			ctx->mbuf_tail = mbuf;
		}

		if (unlikely(mbuf == NULL))
			return (NULL);

		if (page_info->can_flip) {
			MEXTADD(mbuf, va, len, gve_mextadd_free,
			    page_info, NULL, M_RDONLY, EXT_NET_DRV);

			counter_enter();
			counter_u64_add_protected(rx->stats.rx_frag_flip_cnt, 1);
			counter_exit();

			page_info->can_flip = false;
			gve_rx_flip_buff(page_info, &data_slot->qpl_offset);
		} else {
			m_copyback(mbuf, 0, len, va);
			counter_enter();
			counter_u64_add_protected(rx->stats.rx_frag_copy_cnt, 1);
			counter_exit();
		}
	}

	mbuf->m_len = len;
	ctx->total_size += len;

	return (mbuf);
}

static void
gve_rx(struct gve_rx_ring *rx, struct gve_rx_desc *desc, uint32_t idx)
{
	struct gve_rx_slot_page_info *page_info;
	struct gve_dma_handle *page_dma_handle;
	struct gve_priv *priv = rx->com.priv;
	union gve_rx_data_slot *data_slot;
	struct gve_rx_ctx *ctx = &rx->ctx;
	struct mbuf *mbuf = NULL;
	if_t ifp = priv->ifp;
	bool do_if_input;
	uint16_t len;

	bool is_first_frag = ctx->frag_cnt == 0;
	bool is_last_frag = !GVE_PKT_CONT_BIT_IS_SET(desc->flags_seq);
	bool is_only_frag = is_first_frag && is_last_frag;

	if (unlikely(ctx->drop_pkt))
		goto finish_frag;

	if ((desc->flags_seq & GVE_RXF_ERR) != 0) {
		ctx->drop_pkt = true;
		counter_enter();
		counter_u64_add_protected(rx->stats.rx_dropped_pkt_desc_err, 1);
		counter_u64_add_protected(rx->stats.rx_dropped_pkt, 1);
		counter_exit();
		m_freem(ctx->mbuf_head);
		goto finish_frag;
	}

	page_info = &rx->page_info[idx];
	data_slot = &rx->data_ring[idx];
	page_dma_handle = &(rx->com.qpl->dmas[idx]);

	page_info->pad = is_first_frag ? GVE_RX_PAD : 0;
	len = be16toh(desc->len) - page_info->pad;

	bus_dmamap_sync(page_dma_handle->tag, page_dma_handle->map,
	    BUS_DMASYNC_POSTREAD);

	mbuf = gve_rx_create_mbuf(priv, rx, page_info, len, data_slot,
	           is_only_frag);
	if (mbuf == NULL) {
		ctx->drop_pkt = true;
		counter_enter();
		counter_u64_add_protected(rx->stats.rx_dropped_pkt_mbuf_alloc_fail, 1);
		counter_u64_add_protected(rx->stats.rx_dropped_pkt, 1);
		counter_exit();
		m_freem(ctx->mbuf_head);
		goto finish_frag;
	}

	if (is_first_frag) {
		mbuf->m_pkthdr.rcvif = priv->ifp;

		if (gve_needs_rss(desc->flags_seq)) {
			gve_set_rss_type(desc->flags_seq, mbuf);
			mbuf->m_pkthdr.flowid = be32toh(desc->rss_hash);
		}

		if ((desc->csum != 0) && ((desc->flags_seq & GVE_RXF_FRAG) == 0)) {
			mbuf->m_pkthdr.csum_flags = CSUM_IP_CHECKED |
				                    CSUM_IP_VALID |
						    CSUM_DATA_VALID |
						    CSUM_PSEUDO_HDR;
			mbuf->m_pkthdr.csum_data = 0xffff;
		}
	}

	if (is_last_frag) {
		mbuf = ctx->mbuf_head;
		mbuf->m_pkthdr.len = ctx->total_size;
		do_if_input = true;

		if (((ifp->if_capenable & IFCAP_LRO) != 0) && /* LRO is enabled */
		    (desc->flags_seq & GVE_RXF_TCP) &&   /* pkt is a TCP pkt */
		    ((mbuf->m_pkthdr.csum_flags & CSUM_DATA_VALID) != 0)) { /* NIC verified csum */
			if ((rx->lro.lro_cnt != 0) &&         /* LRO resources exist */
			(tcp_lro_rx(&rx->lro, mbuf, 0) == 0)) {
				do_if_input = false;
			}
		}

		if (do_if_input)
			ifp->if_input(ifp, mbuf);

		counter_enter();
		counter_u64_add_protected(rx->stats.rbytes, ctx->total_size);
		counter_u64_add_protected(rx->stats.rpackets, 1);
		counter_exit();
	}

finish_frag:
	ctx->frag_cnt++;
	if (is_last_frag)
		gve_rx_ctx_clear(ctx);

}

static bool
gve_rx_work_pending(struct gve_rx_ring *rx)
{
	struct gve_rx_desc *desc;
	__be16 flags_seq;
	uint32_t next_idx;

	next_idx = rx->cnt & rx->mask;
	desc = rx->desc_ring + next_idx;

	flags_seq = desc->flags_seq;

	return (GVE_SEQNO(flags_seq) == rx->seq_no);
}

static void
gve_rx_cleanup(struct gve_priv *priv, struct gve_rx_ring *rx, int budget)
{
	uint32_t idx = rx->cnt & rx->mask;
	struct gve_rx_desc *desc;
	struct gve_rx_ctx *ctx = &rx->ctx;
	uint32_t work_done = 0;

	NET_EPOCH_ASSERT();

	bus_dmamap_sync(rx->desc_ring_mem.tag, rx->desc_ring_mem.map,
	    BUS_DMASYNC_POSTREAD);
	desc = &rx->desc_ring[idx];

	while ((GVE_SEQNO(desc->flags_seq) == rx->seq_no) &&
	       (work_done < budget || ctx->frag_cnt)) {

		gve_rx(rx, desc, idx);

		rx->cnt++;
		idx = rx->cnt & rx->mask;
		desc = &rx->desc_ring[idx];
		rx->seq_no = gve_next_seqno(rx->seq_no);
		work_done++;
	}

	/* The device will only send whole packets. */
	if (unlikely(ctx->frag_cnt)) {
		m_freem(ctx->mbuf_head);
		gve_rx_ctx_clear(&rx->ctx);
		device_printf(priv->dev,
		    "Unexpected seq number %d with incomplete packet, expected %d, scheduling reset",
		    GVE_SEQNO(desc->flags_seq), rx->seq_no);
		gve_schedule_reset(priv);
	}

	if (work_done != 0)
		tcp_lro_flush_all(&rx->lro);

	bus_dmamap_sync(rx->data_ring_mem.tag, rx->data_ring_mem.map,
	    BUS_DMASYNC_PREWRITE);

	/* Buffers are refilled as the descs are processed */
	rx->fill_cnt += work_done;
	gve_db_bar_write_4(priv, rx->com.db_offset, rx->fill_cnt);
}

void
gve_rx_cleanup_tq(void *arg, int pending)
{
	struct gve_rx_ring *rx = arg;
	struct gve_priv *priv = rx->com.priv;

	gve_rx_cleanup(priv, rx, /*budget=*/128);
	gve_db_bar_write_4(priv, rx->com.irq_db_offset,
	    GVE_IRQ_ACK | GVE_IRQ_EVENT);

	/* Fragments received before this barrier MAY NOT cause the NIC to send an
	 * interrupt but they will still be handled by the enqueue below.
	 * Fragments received after the barrier WILL trigger an interrupt.
	 * */
	mb();

	if (gve_rx_work_pending(rx)) {
		gve_db_bar_write_4(priv, rx->com.irq_db_offset, GVE_IRQ_MASK);
		taskqueue_enqueue(rx->com.cleanup_tq, &rx->com.cleanup_task);
	}
}
