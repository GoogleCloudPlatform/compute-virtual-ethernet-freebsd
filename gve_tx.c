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
#include "gve.h"
#include "gve_adminq.h"

static int
gve_tx_fifo_init(struct gve_priv *priv, struct gve_tx_fifo *fifo,
    struct gve_queue_page_list *qpl)
{
	fifo->base = vmap(qpl->pages, qpl->num_entries, VM_MAP, PAGE_KERNEL);
	if (unlikely(fifo->base == NULL)) {
		device_printf(priv->dev, "Failed to vmap fifo, qpl_id = %d\n",
		    qpl->id);
		return (ENOMEM);
	}

	fifo->size = qpl->num_entries * PAGE_SIZE;
	atomic_set(&fifo->available, fifo->size);
	fifo->head = 0;
	return (0);
}

static void
gve_tx_fifo_release(struct gve_priv *priv, struct gve_tx_fifo *fifo)
{
	if (atomic_read(&fifo->available) != fifo->size)
		device_printf(priv->dev, "Releasing non-empty fifo");
	vunmap(fifo->base);
}

static int
gve_tx_alloc_ring(struct gve_priv *priv, int i)
{
	struct gve_tx_ring *tx = &priv->tx[i];
	struct gve_ring_com *com = &tx->com;
	char mtx_name[16];
	int err;

	com->priv = priv;
	com->id = i;

	com->qpl = gve_assign_tx_qpl(priv);
	if (com->qpl == NULL) {
		device_printf(priv->dev, "No QPL left for tx ring %d i", i);
		return (ENOMEM);
	}

	err = gve_dma_alloc_coherent(priv, sizeof(struct gve_queue_resources),
	          PAGE_SIZE, &com->q_resources_mem,
		  BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT);
	if (err != 0) {
		device_printf(priv->dev, "Cannot alloc queue resources for tx ring %d", i);
		goto abort_with_qpl;
	}
	com->q_resources = com->q_resources_mem.cpu_addr;

	err = gve_dma_alloc_coherent(priv,
		  sizeof(union gve_tx_desc) * priv->tx_desc_cnt,
		  CACHE_LINE_SIZE, &tx->desc_ring_mem,
		  BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT);
	if (err != 0) {
		device_printf(priv->dev, "Cannot alloc desc ring for tx ring %d", i);
		goto abort_with_q_resources;
	}
	tx->desc_ring = tx->desc_ring_mem.cpu_addr;

	err = gve_tx_fifo_init(priv, &tx->fifo, com->qpl);
	if (err != 0)
		goto abort_with_desc_ring;

	tx->info = malloc(sizeof(struct gve_tx_buffer_state) * priv->tx_desc_cnt,
		       M_GVE, M_WAITOK | M_ZERO);
	if (tx->info == NULL) {
		device_printf(priv->dev, "Cannot alloc buf state array for tx ring %d", i);
		goto abort_with_fifo;
	}

	sprintf(mtx_name, "gvetx%d", i);
	mtx_init(&tx->ring_mtx, mtx_name, NULL, MTX_DEF);

	tx->br = buf_ring_alloc(GVE_TX_BUFRING_SIZE, M_DEVBUF, M_WAITOK, &tx->ring_mtx);
	if (tx->br == NULL) {
		device_printf(priv->dev, "Cannot alloc buf ring for tx ring %d", i);
		goto abort_with_iovec;
	}

	gve_alloc_counters((counter_u64_t *)&tx->stats, NUM_TX_STATS);

	return (0);

abort_with_iovec:
	free(tx->info, M_GVE);
	tx->info = NULL;
abort_with_fifo:
	gve_tx_fifo_release(priv, &tx->fifo);
abort_with_desc_ring:
	gve_dma_free_coherent(&com->q_resources_mem);
	tx->desc_ring = NULL;
abort_with_q_resources:
	gve_dma_free_coherent(&com->q_resources_mem);
	com->q_resources = NULL;
abort_with_qpl:
	gve_unassign_qpl(priv, com->qpl->id);
	return (err);
}

static void
gve_tx_free_ring(struct gve_priv *priv, int i)
{
	struct gve_tx_ring *tx = &priv->tx[i];
	struct gve_ring_com *com = &tx->com;

	gve_free_counters((counter_u64_t *)&tx->stats, NUM_TX_STATS);

	buf_ring_free(tx->br, M_DEVBUF);
	tx->br = NULL;

	mtx_destroy(&tx->ring_mtx);

	free(tx->info, M_GVE);
	tx->info = NULL;

	gve_tx_fifo_release(priv, &tx->fifo);

	gve_dma_free_coherent(&tx->desc_ring_mem);
	tx->desc_ring = NULL;

	gve_dma_free_coherent(&com->q_resources_mem);
	com->q_resources = NULL;

	gve_unassign_qpl(priv, com->qpl->id);
}

int
gve_alloc_tx_rings(struct gve_priv *priv)
{
	int err = 0;
	int i;

	priv->tx = malloc(sizeof(struct gve_tx_ring) * priv->tx_cfg.num_queues,
		       M_GVE, M_NOWAIT | M_ZERO);
	if (priv->tx == NULL) {
		device_printf(priv->dev, "Could not alloc tx ring array\n");
		return (ENOMEM);
	}

	for (i = 0; i < priv->tx_cfg.num_queues; i++) {
		err = gve_tx_alloc_ring(priv, i);
		if (err != 0) {
			device_printf(priv->dev, "Failed to alloc tx ring=%d: err=%d\n",
			    i, err);
			goto free_rings;
		}
	}

	return (0);

free_rings:
	while (i--) {
		gve_tx_free_ring(priv, i);
	}
	free(priv->tx, M_GVE);
	return (err);
}

void
gve_free_tx_rings(struct gve_priv *priv)
{
	int i;

	for (i = 0; i < priv->tx_cfg.num_queues; i++)
		gve_tx_free_ring(priv, i);

	free(priv->tx, M_GVE);
}

static void
gve_start_tx_ring(struct gve_priv *priv, int i)
{
	struct gve_tx_ring *tx = &priv->tx[i];
	struct gve_ring_com *com = &tx->com;

	tx->req = tx->done = 0;
	tx->mask = priv->tx_desc_cnt - 1;

	NET_TASK_INIT(&com->cleanup_task, 0, gve_tx_cleanup_tq, tx);
	com->cleanup_tq = taskqueue_create_fast("gve tx", M_WAITOK,
			      taskqueue_thread_enqueue, &com->cleanup_tq);
	taskqueue_start_threads(&com->cleanup_tq, 1, PI_NET, "%s txq %d",
	    device_get_nameunit(priv->dev), i);

	TASK_INIT(&tx->xmit_task, 0, gve_xmit_tq, tx);
	tx->xmit_tq = taskqueue_create_fast("gve tx xmit",
		          M_WAITOK, taskqueue_thread_enqueue, &tx->xmit_tq);
	taskqueue_start_threads(&tx->xmit_tq, 1, PI_NET, "%s txq %d xmit",
	    device_get_nameunit(priv->dev), i);

	bus_dmamap_sync(tx->desc_ring_mem.tag, tx->desc_ring_mem.map,
	    BUS_DMASYNC_PREWRITE);
}

int
gve_create_tx_rings(struct gve_priv *priv)
{
	struct gve_ring_com *com;
	struct gve_tx_ring *tx;
	int err;
	int i;

	if (gve_get_state_flag(priv, GVE_STATE_FLAG_TX_RINGS_OK))
	    return (0);

	err = gve_adminq_create_tx_queues(priv, priv->tx_cfg.num_queues);
	if (err != 0)
		return (err);

	bus_dmamap_sync(priv->irqs_db_mem.tag, priv->irqs_db_mem.map,
	    BUS_DMASYNC_POSTREAD);

	for (i = 0; i < priv->tx_cfg.num_queues; i++) {
		tx = &priv->tx[i];
		com = &tx->com;

		com->irq_db_offset = 4 * be32toh(priv->irq_db_indices[com->ntfy_id].index);

		bus_dmamap_sync(com->q_resources_mem.tag, com->q_resources_mem.map,
		    BUS_DMASYNC_POSTREAD);
		com->db_offset = 4 * be32toh(com->q_resources->db_index);
		com->counter_idx = be32toh(com->q_resources->counter_index);

		gve_start_tx_ring(priv, i);
	}

	gve_set_state_flag(priv, GVE_STATE_FLAG_TX_RINGS_OK);
	return (0);
}

static void
gve_stop_tx_ring(struct gve_priv *priv, int i)
{
	struct gve_tx_ring *tx = &priv->tx[i];
	struct gve_ring_com *com = &tx->com;

	while (taskqueue_cancel(com->cleanup_tq, &com->cleanup_task, NULL))
		taskqueue_drain(com->cleanup_tq, &com->cleanup_task);
	taskqueue_free(com->cleanup_tq);

	while (taskqueue_cancel(tx->xmit_tq, &tx->xmit_task, NULL))
		taskqueue_drain(tx->xmit_tq, &tx->xmit_task);
	taskqueue_free(tx->xmit_tq);
}

int
gve_destroy_tx_rings(struct gve_priv *priv)
{
	int err;
	int i;

	if (!gve_get_state_flag(priv, GVE_STATE_FLAG_TX_RINGS_OK))
	    return (0);

	for (i = 0; i < priv->tx_cfg.num_queues; i++)
		gve_stop_tx_ring(priv, i);

	err = gve_adminq_destroy_tx_queues(priv, priv->tx_cfg.num_queues);
	if (err != 0)
		return (err);

	gve_clear_state_flag(priv, GVE_STATE_FLAG_TX_RINGS_OK);
	return (0);
}

int
gve_tx_intr(void *arg)
{
	struct gve_tx_ring *tx = arg;
	struct gve_priv *priv = tx->com.priv;
	struct gve_ring_com *com = &tx->com;

	if (unlikely((if_getdrvflags(priv->ifp) & IFF_DRV_RUNNING) == 0))
		return (FILTER_STRAY);

	gve_db_bar_write_4(priv, com->irq_db_offset, GVE_IRQ_MASK);
	taskqueue_enqueue(com->cleanup_tq, &com->cleanup_task);
	return (FILTER_HANDLED);
}

static uint32_t
gve_tx_load_event_counter(struct gve_priv *priv, struct gve_tx_ring *tx)
{
	bus_dmamap_sync(priv->counter_array_mem.tag, priv->counter_array_mem.map,
	    BUS_DMASYNC_POSTREAD);
	__be32 counter = READ_ONCE(priv->counters[tx->com.counter_idx]);
	return (be32toh(counter));
}

static void
gve_tx_free_fifo(struct gve_tx_fifo *fifo, size_t bytes)
{
	atomic_add(bytes, &fifo->available);
}

static void
gve_tx_cleanup(struct gve_priv *priv, struct gve_tx_ring *tx, int todo)
{
	struct gve_tx_buffer_state *info;
	size_t space_freed = 0;
	struct mbuf *mbuf;
	int i, j;
	uint32_t idx;

	for (j = 0; j < todo; j++) {
		idx = tx->done & tx->mask;
		info = &tx->info[idx];
		mbuf = info->mbuf;
		tx->done++;

		if (mbuf != NULL) {
			info->mbuf = NULL;
			counter_enter();
			counter_u64_add_protected(tx->stats.tbytes, mbuf->m_pkthdr.len);
			counter_u64_add_protected(tx->stats.tpackets, 1);
			counter_exit();
			m_freem(mbuf);

			for (i = 0; i < ARRAY_SIZE(info->iov); i++) {
				space_freed += info->iov[i].iov_len + info->iov[i].iov_padding;
				info->iov[i].iov_len = 0;
				info->iov[i].iov_padding = 0;
			}
		}
	}

	gve_tx_free_fifo(&tx->fifo, space_freed);
}

void
gve_tx_cleanup_tq(void *arg, int pending)
{
	struct gve_tx_ring *tx = arg;
	struct gve_priv *priv = tx->com.priv;
	uint32_t nic_done = gve_tx_load_event_counter(priv, tx);
	uint32_t todo = nic_done - tx->done;

	gve_tx_cleanup(priv, tx, todo);
	gve_db_bar_write_4(priv, tx->com.irq_db_offset,
			   GVE_IRQ_ACK | GVE_IRQ_EVENT);

	/* Completions born before this barrier MAY NOT cause the NIC to send an
	 * interrupt but they will still be handled by the enqueue below.
	 * Completions born after the barrier WILL trigger an interrupt.
	 * */
	mb();

	nic_done = gve_tx_load_event_counter(priv, tx);
	todo = nic_done - tx->done;
	if (todo != 0) {
		gve_db_bar_write_4(priv, tx->com.irq_db_offset, GVE_IRQ_MASK);
		taskqueue_enqueue(tx->com.cleanup_tq, &tx->com.cleanup_task);
	}
}

static void
gve_dma_sync_for_device(struct gve_queue_page_list *qpl,
			uint64_t iov_offset, uint64_t iov_len)
{
	uint64_t last_page = (iov_offset + iov_len - 1) / PAGE_SIZE;
	uint64_t first_page = iov_offset / PAGE_SIZE;
	struct gve_dma_handle *dma;
	uint64_t page;

	for (page = first_page; page <= last_page; page++) {
		dma = &(qpl->dmas[page]);
		bus_dmamap_sync(dma->tag, dma->map, BUS_DMASYNC_PREWRITE);
	}
}

static void
gve_tx_fill_mtd_desc(union gve_tx_desc *mtd_desc, struct mbuf *mbuf)
{
	BUILD_BUG_ON(sizeof(mtd_desc->mtd) != sizeof(mtd_desc->pkt));

	mtd_desc->mtd.type_flags = GVE_TXD_MTD | GVE_MTD_SUBTYPE_PATH;
	mtd_desc->mtd.path_state = GVE_MTD_PATH_STATE_DEFAULT |
				   GVE_MTD_PATH_HASH_L4;
	mtd_desc->mtd.path_hash = htobe32(mbuf->m_pkthdr.flowid);
	mtd_desc->mtd.reserved0 = 0;
	mtd_desc->mtd.reserved1 = 0;
}

static void
gve_tx_fill_pkt_desc(union gve_tx_desc *pkt_desc, bool is_tso,
		     uint16_t l4_hdr_offset, uint32_t desc_cnt,
		     uint16_t first_seg_len, uint64_t addr, bool has_csum_flag,
		     int csum_offset, uint16_t pkt_len)
{
	if (is_tso) {
		pkt_desc->pkt.type_flags = GVE_TXD_TSO | GVE_TXF_L4CSUM;
		pkt_desc->pkt.l4_csum_offset = csum_offset >> 1;
		pkt_desc->pkt.l4_hdr_offset = l4_hdr_offset >> 1;
	} else if (has_csum_flag) {
		pkt_desc->pkt.type_flags = GVE_TXD_STD | GVE_TXF_L4CSUM;
		pkt_desc->pkt.l4_csum_offset = csum_offset >> 1;
		pkt_desc->pkt.l4_hdr_offset = l4_hdr_offset >> 1;
	} else {
		pkt_desc->pkt.type_flags = GVE_TXD_STD;
		pkt_desc->pkt.l4_csum_offset = 0;
		pkt_desc->pkt.l4_hdr_offset = 0;
	}
	pkt_desc->pkt.desc_cnt = desc_cnt;
	pkt_desc->pkt.len = htobe16(pkt_len);
	pkt_desc->pkt.seg_len = htobe16(first_seg_len);
	pkt_desc->pkt.seg_addr = htobe64(addr);
}

static void
gve_tx_fill_seg_desc(union gve_tx_desc *seg_desc,
		     bool is_tso, uint16_t len, uint64_t addr,
		     bool is_ipv6, uint8_t l3_off, uint16_t tso_mss)
{
	seg_desc->seg.type_flags = GVE_TXD_SEG;
	if (is_tso) {
		if (is_ipv6)
			seg_desc->seg.type_flags |= GVE_TXSF_IPV6;
		seg_desc->seg.l3_offset = l3_off >> 1;
		seg_desc->seg.mss = htobe16(tso_mss);
	}
	seg_desc->seg.seg_len = htobe16(len);
	seg_desc->seg.seg_addr = htobe64(addr);
}

static inline uint32_t
gve_tx_avail(struct gve_tx_ring *tx)
{
	return (tx->mask + 1 - (tx->req - tx->done));
}

static bool
gve_tx_fifo_can_alloc(struct gve_tx_fifo *fifo, size_t bytes)
{
	return (atomic_read(&fifo->available) <= bytes) ? false : true;
}

static inline bool
gve_can_tx(struct gve_tx_ring *tx, int bytes_required)
{
	bool can_alloc = gve_tx_fifo_can_alloc(&tx->fifo, bytes_required);
	return (gve_tx_avail(tx) >= (GVE_TX_MAX_IOVEC + 1) && can_alloc);
}

static int
gve_tx_fifo_pad_alloc_one_frag(struct gve_tx_fifo *fifo, size_t bytes)
{
	return (fifo->head + bytes < fifo->size) ? 0 : fifo->size - fifo->head;
}

static inline int
gve_fifo_bytes_required(struct gve_tx_ring *tx, uint16_t first_seg_len, uint16_t pkt_len)
{
	int pad_bytes, align_hdr_pad;
	int bytes;

	pad_bytes = gve_tx_fifo_pad_alloc_one_frag(&tx->fifo, first_seg_len);
	/* We need to take into account the header alignment padding. */
	align_hdr_pad = ALIGN(first_seg_len, CACHE_LINE_SIZE) - first_seg_len;
	bytes = align_hdr_pad + pad_bytes + pkt_len;

	return (bytes);
}

static int
gve_tx_alloc_fifo(struct gve_tx_fifo *fifo, size_t bytes,
		  struct gve_tx_iovec iov[2])
{
	size_t overflow, padding;
	uint32_t aligned_head;
	int nfrags = 0;

	if (bytes == 0)
		return (0);

	/* This check happens before we know how much padding is needed to
	 * align to a cacheline boundary for the payload, but that is fine,
	 * because the FIFO head always start aligned, and the FIFO's boundaries
	 * are aligned, so if there is space for the data, there is space for
	 * the padding to the next alignment.
	 */
	WARN(!gve_tx_fifo_can_alloc(fifo, bytes),
	    "Reached %s when there's not enough space in the fifo", __func__);

	nfrags++;

	iov[0].iov_offset = fifo->head;
	iov[0].iov_len = bytes;
	fifo->head += bytes;

	if (fifo->head > fifo->size) {
		/* If the allocation did not fit in the tail fragment of the
		 * FIFO, also use the head fragment.
		 */
		nfrags++;
		overflow = fifo->head - fifo->size;
		iov[0].iov_len -= overflow;
		iov[1].iov_offset = 0;	/* Start of fifo*/
		iov[1].iov_len = overflow;

		fifo->head = overflow;
	}

	/* Re-align to a cacheline boundary */
	aligned_head = ALIGN(fifo->head, CACHE_LINE_SIZE);
	padding = aligned_head - fifo->head;
	iov[nfrags - 1].iov_padding = padding;
	atomic_sub(bytes + padding, &fifo->available);
	fifo->head = aligned_head;

	if (fifo->head == fifo->size)
		fifo->head = 0;

	return (nfrags);
}

/* Only error this returns is ENOBUFS when the tx fifo is short of space */
static int
gve_xmit(struct gve_tx_ring *tx, struct mbuf *mbuf)
{
	int csum_flags, csum_offset, mtd_desc_nr, offset, copy_offset;
	uint16_t tso_mss, l4_off, l4_data_off, pkt_len, first_seg_len;
	int pad_bytes, hdr_nfrags, payload_nfrags;
	union gve_tx_desc *pkt_desc, *seg_desc;
	bool is_tso, has_csum_flag, is_ipv6;
	struct gve_tx_buffer_state *info;
	uint32_t idx = tx->req & tx->mask;
	struct ether_vlan_header *eh;
	struct mbuf *mbuf_next;
	int payload_iov = 2;
	int bytes_required;
	struct tcphdr *th;
	struct ip *ip;
	uint32_t next_idx;
	uint8_t l3_off;
	int i;

	info = &tx->info[idx];
	csum_flags = mbuf->m_pkthdr.csum_flags;
	pkt_len = mbuf->m_pkthdr.len;
	is_tso = csum_flags & CSUM_TSO;
	has_csum_flag = csum_flags & (CSUM_TCP | CSUM_UDP);
	mtd_desc_nr = M_HASHTYPE_GET(mbuf) != M_HASHTYPE_NONE ? 1 : 0;
	tso_mss = is_tso ? mbuf->m_pkthdr.tso_segsz : 0;

	eh = mtod(mbuf, struct ether_vlan_header *);
	is_ipv6 = ntohs(eh->evl_proto) == ETHERTYPE_IPV6;
	if (eh->evl_encap_proto == htons(ETHERTYPE_VLAN))
		l3_off = ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN;
	else
		l3_off = ETHER_HDR_LEN;

	mbuf_next = m_getptr(mbuf, l3_off, &offset);
	ip = (struct ip *)(mtodo(mbuf_next, offset));
	l4_off = l3_off + (ip->ip_hl << 2);

	l4_data_off = 0;
	mbuf_next = m_getptr(mbuf, l4_off, &offset);
	if (ip->ip_p == IPPROTO_TCP) {
		th = (struct tcphdr *)(mtodo(mbuf_next, offset));
		l4_data_off = l4_off + (th->th_off << 2);
	} else if (ip->ip_p == IPPROTO_UDP) {
		l4_data_off = l4_off + sizeof(struct udphdr);
	}

	if (has_csum_flag) {
		if ((csum_flags & CSUM_TCP) != 0)
			csum_offset = offsetof(struct tcphdr, th_sum);
		else
			csum_offset = offsetof(struct udphdr, uh_sum);
	}

	/* If this packet is neither a TCP nor a UDP packet, the first segment,
	 * the one represented by the packet descriptor, will carry the
	 * spec-stipulated minimum of 182B. */
	if (l4_data_off != 0)
		first_seg_len = l4_data_off;
	else
		first_seg_len = min_t(uint16_t, pkt_len, 182);

	bytes_required = gve_fifo_bytes_required(tx, first_seg_len, pkt_len);
	if (unlikely(!gve_can_tx(tx, bytes_required))) {
		counter_enter();
		counter_u64_add_protected(tx->stats.tx_dropped_pkt_nospace_device, 1);
		counter_u64_add_protected(tx->stats.tx_dropped_pkt, 1);
		counter_exit();
		return (ENOBUFS);
	}

	/* So that the cleanup taskqueue can free the mbuf eventually. */
	info->mbuf = mbuf;

	/* We don't want to split the header, so if necessary, pad to the end
	 * of the fifo and then put the header at the beginning of the fifo.
	 */
	pad_bytes = gve_tx_fifo_pad_alloc_one_frag(&tx->fifo, first_seg_len);
	hdr_nfrags = gve_tx_alloc_fifo(&tx->fifo, first_seg_len + pad_bytes,
		         &info->iov[0]);
	WARN(!hdr_nfrags, "hdr_nfrags should never be 0!");
	payload_nfrags = gve_tx_alloc_fifo(&tx->fifo, pkt_len - first_seg_len,
			     &info->iov[payload_iov]);

	pkt_desc = &tx->desc_ring[idx];
	gve_tx_fill_pkt_desc(pkt_desc, is_tso, l4_off,
	    1 + mtd_desc_nr + payload_nfrags, first_seg_len,
	    info->iov[hdr_nfrags - 1].iov_offset, has_csum_flag, csum_offset,
	    pkt_len);

	m_copydata(mbuf, 0, first_seg_len,
	    (char*)tx->fifo.base + info->iov[hdr_nfrags - 1].iov_offset);
	gve_dma_sync_for_device(tx->com.qpl,
	    info->iov[hdr_nfrags - 1].iov_offset,
	    info->iov[hdr_nfrags - 1].iov_len);
	copy_offset = first_seg_len;

	if (mtd_desc_nr == 1) {
		next_idx = (tx->req + 1) & tx->mask;
		gve_tx_fill_mtd_desc(&tx->desc_ring[next_idx], mbuf);
	}

	for (i = payload_iov; i < payload_nfrags + payload_iov; i++) {
		next_idx = (tx->req + 1 + mtd_desc_nr + i - payload_iov) & tx->mask;
		seg_desc = &tx->desc_ring[next_idx];

		gve_tx_fill_seg_desc(seg_desc, is_tso, info->iov[i].iov_len,
		    info->iov[i].iov_offset, is_ipv6, l3_off, tso_mss);

		m_copydata(mbuf, copy_offset, info->iov[i].iov_len,
		    (char*)tx->fifo.base + info->iov[i].iov_offset);
		gve_dma_sync_for_device(tx->com.qpl,
		    info->iov[i].iov_offset, info->iov[i].iov_len);
		copy_offset += info->iov[i].iov_len;
	}

	tx->req += (1 + mtd_desc_nr + payload_nfrags);
	if (is_tso) {
		counter_enter();
		counter_u64_add_protected(tx->stats.tso_packet_cnt, 1);
		counter_exit();
	}
	return (0);
}

static void
gve_xmit_br(struct gve_tx_ring *tx)
{
	struct gve_priv *priv = tx->com.priv;
	struct ifnet *ifp = priv->ifp;
	struct mbuf *mbuf;

	while (!drbr_empty(ifp, tx->br) &&
	       (if_getdrvflags(ifp) & IFF_DRV_RUNNING) != 0) {

		mbuf = drbr_peek(ifp, tx->br);
		if (unlikely(gve_xmit(tx, mbuf) != 0)) {
			drbr_putback(ifp, tx->br, mbuf);
			taskqueue_enqueue(tx->xmit_tq, &tx->xmit_task);
			break;
		}

		bus_dmamap_sync(tx->desc_ring_mem.tag, tx->desc_ring_mem.map,
		    BUS_DMASYNC_PREWRITE);
		gve_db_bar_write_4(priv, tx->com.db_offset, tx->req);

		drbr_advance(ifp, tx->br);
	}
}

void
gve_xmit_tq(void *arg, int pending)
{
	struct gve_tx_ring *tx = (struct gve_tx_ring *)arg;

	GVE_RING_LOCK(tx);
	gve_xmit_br(tx);
	GVE_RING_UNLOCK(tx);
}

int
gve_xmit_ifp(if_t ifp, struct mbuf *mbuf)
{
	struct gve_priv *priv = ifp->if_softc;
	struct gve_tx_ring *tx;
	bool is_br_empty;
	int err;
	uint32_t i;

	if (unlikely((if_getdrvflags(priv->ifp) & IFF_DRV_RUNNING) == 0))
		return (ENODEV);

	if (M_HASHTYPE_GET(mbuf) != M_HASHTYPE_NONE)
		i = mbuf->m_pkthdr.flowid % priv->tx_cfg.num_queues;
	else
		i = curcpu % priv->tx_cfg.num_queues;
	tx = &priv->tx[i];

	is_br_empty = drbr_empty(ifp, tx->br);
	err = drbr_enqueue(ifp, tx->br, mbuf);
	if (unlikely(err != 0)) {
		taskqueue_enqueue(tx->xmit_tq, &tx->xmit_task);
		counter_enter();
		counter_u64_add_protected(tx->stats.tx_dropped_pkt_nospace_bufring, 1);
		counter_u64_add_protected(tx->stats.tx_dropped_pkt, 1);
		counter_exit();
		return (err);
	}

	/* If the mbuf we just enqueued is the only one on the ring, then
	 * transmit it right away in the interests of low latency. */
	if (is_br_empty && (GVE_RING_TRYLOCK(tx) != 0)) {
		gve_xmit_br(tx);
		GVE_RING_UNLOCK(tx);
	} else {
		taskqueue_enqueue(tx->xmit_tq, &tx->xmit_task);
	}

	return (0);
}

void
gve_qflush(if_t ifp)
{
	struct gve_priv *priv = ifp->if_softc;
	struct gve_tx_ring *tx;
	int i;

	for(i = 0; i < priv->tx_cfg.num_queues; ++i) {
		tx = &priv->tx[i];
		if (drbr_empty(ifp, tx->br) == 0) {
			GVE_RING_LOCK(tx);
			drbr_flush(ifp, tx->br);
			GVE_RING_UNLOCK(tx);
		}
	}

	if_qflush(ifp);
}
