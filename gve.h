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
#ifndef _GVE_FBSD_H
#define _GVE_FBSD_H

#include "gve_desc.h"
#include "gve_plat.h"
#include "gve_register.h"

#ifndef PCI_VENDOR_ID_GOOGLE
#define PCI_VENDOR_ID_GOOGLE	0x1ae0
#endif

#define PCI_DEV_ID_GVNIC	0x0042
#define GVE_REGISTER_BAR	0
#define GVE_DOORBELL_BAR	2

/* Driver can alloc up to 2 segments for the header and 2 for the payload. */
#define GVE_TX_MAX_IOVEC	4
#define GVE_TX_BUFRING_SIZE	4096

#define GVE_MIN_RING_SIZE 64
#define GVE_MAX_RING_SIZE 12288
#define ADMINQ_SIZE 4096

#define NUM_RX_STATS 8
#define NUM_TX_STATS 6

#define GVE_DEFAULT_RX_BUFFER_SIZE (ADMINQ_SIZE / 2)
#define GVE_DEFAULT_RX_BUFFER_OFFSET (ADMINQ_SIZE / 2)

/* Number of descriptors per queue page list.
 * Page count AKA QPL size can be derived by dividing the number of elements in
 * a page by the number of descriptors available.
 */
#define GVE_QPL_DIVISOR	16

static MALLOC_DEFINE(M_GVE, "gve", "gve allocations");

struct gve_dma_handle
{
	bus_addr_t	bus_addr;
	void		*cpu_addr;
	bus_dma_tag_t	tag;
	bus_dmamap_t	map;
	int		size;
};

/* Each slot in the desc ring has a 1:1 mapping to a slot in the data ring */
struct gve_rx_desc_queue {
	struct gve_rx_desc *desc_ring; /* the descriptor ring */
	dma_addr_t bus; /* the bus for the desc_ring */
	uint8_t seqno; /* the next expected seqno for this desc*/
};

union gve_tx_desc {
	struct gve_tx_pkt_desc pkt; /* first desc for a packet */
	struct gve_tx_mtd_desc mtd; /* optional metadata descriptor */
	struct gve_tx_seg_desc seg; /* subsequent descs for a packet */
};

/* Tracks the memory in the fifo occupied by a segment of a packet */
struct gve_tx_iovec {
	uint32_t iov_offset; /* offset into this segment */
	uint32_t iov_len; /* length */
	uint32_t iov_padding; /* padding associated with this segment */
};

/* Tracks allowed and current queue settings */
struct gve_queue_config {
	uint16_t max_queues;
	uint16_t num_queues; /* current */
};

/* Tracks the available and used qpl IDs */
struct gve_qpl_config {
	uint32_t qpl_map_size; /* map memory size */
	unsigned long *qpl_id_map; /* bitmap of used qpl ids */
};

struct gve_irq_db {
	__be32 index;
} ____cacheline_aligned;


/* GVE_QUEUE_FORMAT_UNSPECIFIED must be zero since 0 is the default value
 * when the entire configure_device_resources command is zeroed out and the
 * queue_format is not specified.
 */
enum gve_queue_format {
	GVE_QUEUE_FORMAT_UNSPECIFIED	= 0x0,
	GVE_GQI_RDA_FORMAT		= 0x1,
	GVE_GQI_QPL_FORMAT		= 0x2,
	GVE_DQO_RDA_FORMAT		= 0x3,
};

enum gve_state_flags_bit {
	GVE_STATE_FLAG_ADMINQ_OK = 1,
	GVE_STATE_FLAG_RESOURCES_OK = 2,
	GVE_STATE_FLAG_QPLREG_OK = 3,
	GVE_STATE_FLAG_RX_RINGS_OK = 4,
	GVE_STATE_FLAG_TX_RINGS_OK = 5,
	GVE_STATE_FLAG_QUEUES_UP = 6,
	GVE_STATE_FLAG_LINK_UP = 7,
	GVE_STATE_FLAG_DO_RESET = 8,
	GVE_STATE_FLAG_IN_RESET = 9,
};

#define GVE_DEVICE_STATUS_RESET (0x1 << 1)
#define GVE_DEVICE_STATUS_LINK_STATUS (0x1 << 2)
#define GVE_DEVICE_STATUS_REPORT_STATS (0x1 << 3)

#define GVE_RING_LOCK(ring)	mtx_lock(&(ring)->ring_mtx)
#define GVE_RING_TRYLOCK(ring)	mtx_trylock(&(ring)->ring_mtx)
#define GVE_RING_UNLOCK(ring)	mtx_unlock(&(ring)->ring_mtx)
#define GVE_RING_ASSERT(ring)	mtx_assert(&(ring)->ring_mtx, MA_OWNED)

#define GVE_GLOBAL_LOCK_INIT()	sx_init(&gve_global_lock, "gve global lock")
#define GVE_GLOBAL_LOCK_DESTROY() sx_destroy(&gve_global_lock)
#define GVE_GLOBAL_LOCK_LOCK() sx_xlock(&gve_global_lock)
#define GVE_GLOBAL_LOCK_UNLOCK() sx_unlock(&gve_global_lock)
#define GVE_GLOBAL_LOCK_ASSERT() sx_assert(&gve_global_lock, SA_XLOCKED)

extern struct sx gve_global_lock;

struct gve_queue_page_list {
	uint32_t id;
	uint32_t num_entries;
	struct page **pages;
	struct gve_dma_handle *dmas;
};

struct gve_irq {
	struct resource *res;
	void *cookie;
};

struct gve_rx_slot_page_info {
	void *page_address;
	uint32_t page_offset;
	uint16_t pad;
	uint8_t can_flip;
};

/* A single received packet split across multiple buffers may be
 * reconstructed using the information in this structure.
 */
struct gve_rx_ctx {
	/* head and tail of mbuf chain for the current packet or NULL if none */
	struct mbuf *mbuf_head;
	struct mbuf *mbuf_tail;
	uint32_t total_size;
	uint8_t frag_cnt;
	bool drop_pkt;
};

struct gve_ring_com {
	struct gve_priv *priv;
	uint32_t id;

	/* BAR2 offset for this ring's doorbell and the
	 * counter-array offset for this ring's counter.
	 * Acquired from the device individually for each
	 * queue in the queue_create adminq command. */
	struct gve_queue_resources *q_resources;
	struct gve_dma_handle q_resources_mem;

	/* Byte offset into BAR2 where this ring's 4-byte irq doorbell lies. */
	uint32_t irq_db_offset;
	/* Byte offset into BAR2 where this ring's 4-byte doorbell lies. */
	uint32_t db_offset;
	/* Index, not byte-offset, into the counter array where this ring's
	 * 4-byte counter lies. */
	uint32_t counter_idx;

	/* The index of the MSIX vector that was assigned to
	 * this ring in `gve_alloc_irqs`.
	 *
	 * It is passed to the device in the queue_create adminq
	 * command.
	 *
	 * Additionally, this also serves as the index into
	 * `priv->irq_db_indices` where this ring's irq doorbell's
	 * BAR2 offset, `irq_db_idx`, can be found.*/
	int ntfy_id;

	/* The fixed bounce buffer for this ring.
	 * Once allocated, has to be offered to the device
	 * over the register-page-list adminq command. */
	struct gve_queue_page_list *qpl;

	struct task cleanup_task;
	struct taskqueue *cleanup_tq;
} __aligned(CACHE_LINE_SIZE);

struct gve_rxq_stats {
	counter_u64_t rbytes;
	counter_u64_t rpackets;
	counter_u64_t rx_dropped_pkt;
	counter_u64_t rx_copybreak_cnt;
	counter_u64_t rx_frag_flip_cnt;
	counter_u64_t rx_frag_copy_cnt;
	counter_u64_t rx_dropped_pkt_desc_err;
	counter_u64_t rx_dropped_pkt_mbuf_alloc_fail;
};

struct gve_rx_ring {
	struct gve_ring_com com;

	struct gve_rx_ctx ctx;

	struct gve_dma_handle desc_ring_mem;
	struct gve_rx_desc *desc_ring;
	uint8_t seq_no;

	struct gve_dma_handle data_ring_mem;
	union gve_rx_data_slot *data_ring;

	struct gve_rx_slot_page_info *page_info;

	struct lro_ctrl lro;

	struct gve_rxq_stats stats;
	uint32_t cnt; /* free-running total number of completed packets */
	uint32_t fill_cnt; /* free-running total number of descs and buffs posted */
	uint32_t mask; /* masks the cnt and fill_cnt to the size of the ring */

} __aligned(CACHE_LINE_SIZE);

/* A TX buffer - each queue has one */
struct gve_tx_fifo {
	void *base; /* address of base of FIFO */
	uint32_t size; /* total size */
	atomic_t available; /* how much space is still available */
	uint32_t head; /* offset to write at */
};

struct gve_tx_buffer_state {
	struct mbuf *mbuf;
	struct gve_tx_iovec iov[GVE_TX_MAX_IOVEC];
};

struct gve_txq_stats {
	counter_u64_t tbytes;
	counter_u64_t tpackets;
	counter_u64_t tso_packet_cnt;
	counter_u64_t tx_dropped_pkt;
	counter_u64_t tx_dropped_pkt_nospace_device;
	counter_u64_t tx_dropped_pkt_nospace_bufring;
};

struct gve_tx_ring {
	struct gve_ring_com com;

	struct gve_dma_handle desc_ring_mem;
	union gve_tx_desc *desc_ring;
	struct gve_tx_buffer_state *info;
	uint8_t seq_no;

	struct buf_ring *br;
	struct mtx ring_mtx;

	struct task xmit_task;
	struct taskqueue *xmit_tq;

	struct gve_tx_fifo fifo;

	struct gve_txq_stats stats;
	uint32_t req;
	uint32_t done;
	uint32_t mask;
} __aligned(CACHE_LINE_SIZE);

struct gve_priv
{
	if_t ifp;
	device_t dev;
	struct ifmedia media;

	uint8_t mac[ETHER_ADDR_LEN];

	struct gve_dma_handle aq_mem;

	struct resource *reg_bar; /* BAR0 */
	struct resource *db_bar; /* BAR2 */
	struct resource *msix_table;

	uint32_t mgmt_msix_idx;
	uint32_t rx_copybreak;

	uint16_t num_event_counters;
	uint16_t default_num_queues;
	uint16_t tx_desc_cnt;
	uint16_t rx_desc_cnt;
	uint16_t rx_pages_per_qpl;
	uint16_t max_rx_desc_cnt;
	uint16_t max_tx_desc_cnt;
	uint64_t max_registered_pages;
	uint64_t num_registered_pages;
	uint32_t supported_features;
	uint16_t max_mtu;

	struct gve_dma_handle counter_array_mem;
	__be32 *counters;
	struct gve_dma_handle irqs_db_mem;
	struct gve_irq_db *irq_db_indices;

	enum gve_queue_format queue_format;
	struct gve_queue_page_list *qpls;
	struct gve_queue_config tx_cfg;
	struct gve_queue_config rx_cfg;
	struct gve_qpl_config qpl_cfg;
	uint32_t num_queues;

	struct gve_irq *irq_tbl;
	struct gve_tx_ring *tx;
	struct gve_rx_ring *rx;

	union gve_adminq_command *adminq;
	dma_addr_t adminq_bus_addr;
	uint32_t adminq_mask;
	uint32_t adminq_prod_cnt;
	uint32_t adminq_cmd_fail;
	uint32_t adminq_timeouts;
	uint32_t adminq_describe_device_cnt;
	uint32_t adminq_cfg_device_resources_cnt;
	uint32_t adminq_register_page_list_cnt;
	uint32_t adminq_unregister_page_list_cnt;
	uint32_t adminq_create_tx_queue_cnt;
	uint32_t adminq_create_rx_queue_cnt;
	uint32_t adminq_destroy_tx_queue_cnt;
	uint32_t adminq_destroy_rx_queue_cnt;
	uint32_t adminq_dcfg_device_resources_cnt;
	uint32_t adminq_set_driver_parameter_cnt;
	uint32_t adminq_report_stats_cnt;
	uint32_t adminq_report_link_speed_cnt;
	uint32_t adminq_get_ptype_map_cnt;
	uint32_t adminq_verify_driver_compatibility_cnt;

	/* Global stats */
	uint32_t interface_up_cnt; /* count of times interface turned up since last reset */
	uint32_t interface_down_cnt; /* count of times interface turned down since last reset */
	uint32_t reset_cnt; /* count of reset */

	struct task service_task;
	struct taskqueue *service_tq;

	unsigned long state_flags;
};

static inline bool
gve_get_state_flag(struct gve_priv *priv, int pos)
{
	return (test_bit(pos, &priv->state_flags));
}

static inline void
gve_set_state_flag(struct gve_priv *priv, int pos)
{
	set_bit(pos, &priv->state_flags);
}

static inline void
gve_clear_state_flag(struct gve_priv *priv, int pos)
{
	clear_bit(pos, &priv->state_flags);
}

void gve_schedule_reset(struct gve_priv *priv);

/* Register access functions */
uint32_t gve_reg_bar_read_4(struct gve_priv *priv, bus_size_t offset);
uint64_t gve_reg_bar_read_8(struct gve_priv *priv, bus_size_t offset);
void gve_reg_bar_write_4(struct gve_priv *priv, bus_size_t offset, uint32_t val);
void gve_reg_bar_write_8(struct gve_priv *priv, bus_size_t offset, uint64_t val);
void gve_db_bar_write_4(struct gve_priv *priv, bus_size_t offset, uint32_t val);

/* QPL functions */
uint32_t gve_num_tx_qpls(struct gve_priv *priv);
uint32_t gve_num_rx_qpls(struct gve_priv *priv);
struct gve_queue_page_list * gve_assign_tx_qpl(struct gve_priv *priv);
struct gve_queue_page_list * gve_assign_rx_qpl(struct gve_priv *priv);
void gve_unassign_qpl(struct gve_priv *priv, int id);
int gve_alloc_qpls(struct gve_priv *priv);
void gve_free_qpls(struct gve_priv *priv);
int gve_register_qpls(struct gve_priv *priv);
int gve_unregister_qpls(struct gve_priv *priv);

/* TX functions */
int gve_alloc_tx_rings(struct gve_priv *priv);
void gve_free_tx_rings(struct gve_priv *priv);
int gve_create_tx_rings(struct gve_priv *priv);
int gve_destroy_tx_rings(struct gve_priv *priv);
int gve_tx_intr(void *arg);
int gve_xmit_ifp(if_t ifp, struct mbuf *mbuf);
void gve_qflush(if_t ifp);
void gve_xmit_tq(void *arg, int pending);
void gve_tx_cleanup_tq(void *arg, int pending);

/* RX functions */
int gve_alloc_rx_rings(struct gve_priv *priv);
void gve_free_rx_rings(struct gve_priv *priv);
int gve_create_rx_rings(struct gve_priv *priv);
int gve_destroy_rx_rings(struct gve_priv *priv);
void gve_prefill_rx_slots(struct gve_rx_ring *rx);
int gve_rx_intr(void *arg);
void gve_rx_cleanup_tq(void *arg, int pending);

/* DMA functions */
int gve_dma_alloc_coherent(struct gve_priv *priv, int size, int align,
    struct gve_dma_handle *dma, int mapflags);
void gve_dma_free_coherent(struct gve_dma_handle *dma);

/* IRQ functions */
void gve_free_irqs(struct gve_priv *priv);
int gve_alloc_irqs(struct gve_priv *priv);
void gve_unmask_all_queue_irqs(struct gve_priv *priv);
void gve_mask_all_queue_irqs(struct gve_priv *priv);

/* Systcl functions */
void gve_setup_sysctl(struct gve_priv *priv);
void gve_accum_stats(struct gve_priv *priv, uint64_t *rpackets,
    uint64_t *rbytes, uint64_t *rx_dropped_pkt, uint64_t *tpackets,
    uint64_t *tbytes, uint64_t *tx_dropped_pkt);

/* Stats functions */
void gve_alloc_counters(counter_u64_t *stat, int num_stats);
void gve_free_counters(counter_u64_t *stat, int num_stats);

#endif /* _GVE_FBSD_H_ */
