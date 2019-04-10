/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 The FreeBSD Foundation, Inc.
 *
 * This driver was written by
 * Gerald ND Aryeetey <gndaryee@uwaterloo.ca> under sponsorship
 * from the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */
#ifndef _IF_MGB_H_
#define _IF_MGB_H_

#define unlikely(x)                     __builtin_expect(!!(x), 0)


#define PCI_VENDOR_ID_MICROCHIP		0x1055
#define PCI_DEVICE_ID_LAN7430		0x7430
#define PCI_DEVICE_ID_LAN7431		0x7431

#define MGB_TIMEOUT			(500)

/** Control/Status Registers **/
#define MGB_BAR				0 /* PCI Base Address */

/** Reset **/
#define MGB_HW_CFG			0x10 /** H/W Configuration Register **/
#define MGB_LITE_RESET 			0x2

/** MAC **/
#define MGB_MAC_CR			0x0100 /** MAC Crontrol Register **/
#define MGB_MAC_ADD_ENBL		0x1000 /* Automatic Duplex Detection */
#define MGB_MAC_ASD_ENBL		0x0800 /* Automatic Speed Detection */

#define MGB_MAC_ADDR_BASE_L		0x11C /** MAC address lower 4 bytes (read) register **/
#define MGB_MAC_ADDR_BASE_H		0x118 /** MAC address upper 2 bytes (read) register **/

/** PHY Reset (via power management control) **/
#define MGB_PMT_CTL			0x14 /** Power Management Control Register **/
#define MGB_PHY_RESET			0x10
#define MGB_PHY_READY			0x80

/** FIFO Controller **/
#define MGB_FCT_TX_CTL			0xC4
#define MGB_FCT_RX_CTL			0xAC
#define MGB_FCT_ENBL(_channel)		(1 << (28 + (_channel)))
#define MGB_FCT_DSBL(_channel)		(1 << (24 + (_channel)))
#define MGB_FCT_RESET(_channel)		(1 << (20 + (_channel)))



/** DMA Controller **/
#define MGB_DMAC_CMD			0xC0C
#define MGB_DMAC_RESET			0x80000000
#define MGB_DMAC_TX_RESET(_channel)	(1 << (24 + (_channel)))
#define MGB_DMAC_TX_START(_channel)	(1 << (20 + (_channel)))
#define MGB_DMAC_TX_STOP(_channel)	(1 << (16 + (_channel)))
#define MGB_DMAC_RX_RESET(_channel)	(1 << (8 + (_channel)))
#define MGB_DMAC_RX_START(_channel)	(1 << (4 + (_channel)))
#define MGB_DMAC_RX_STOP(_channel)	(1 << (0 + (_channel)))
#define MGB_DMAC_STATE(_start, _stop)	\
	(((_start) ? 2 : 0) | ((_stop) ? 1 : 0))
#define MGB_DMAC_STATE_INITIAL		MGB_DMAC_STATE(0, 0)
#define MGB_DMAC_STATE_STARTED		MGB_DMAC_STATE(1, 0)
#define MGB_DMAC_STATE_STOP_PENDING	MGB_DMAC_STATE(1, 1)
#define MGB_DMAC_STATE_STOPPED		MGB_DMAC_STATE(0, 1)

#define MGB_DMAC_INTR_STS		0xC10
#define MGB_DMAC_INTR_ENBL_SET		0xC14
#define MGB_DMAC_INTR_ENBL_CLR		0xC18
#define MGB_DMAC_TX_INTR_ENBL		(0x1)
#define MGB_DMAC_RX_INTR_ENBL		(0x1 << 8)

/** DMA Rings **/
/**
 * Page size is 256 bytes so this matches
 *
 * Ring size, however, these could be tunable
 * to be a multiple of 4 (max is 65532)
 *
 **/
#define MGB_DMA_RING_SIZE		1024 /* in programming guide, this number is 100 */
#define MGB_DMA_MAXSEGS			1024
#define MGB_DMA_REG(reg, _channel)	(reg | (_channel << 6))
#define MGB_DMA_RING_LIST_SIZE		\
	(sizeof(struct mgb_ring_desc) * MGB_DMA_RING_SIZE)
#define MGB_DMA_RING_INFO_SIZE		\
	(sizeof(uint32_t) + MGB_DMA_RING_LIST_SIZE)

#define MGB_DMA_TX_CONFIG0(_channel)	MGB_DMA_REG(0x0D40, _channel)
#define MGB_DMA_TX_CONFIG1(_channel)	MGB_DMA_REG(0x0D44, _channel)
#define MGB_DMA_TX_BASE_H(_channel)	MGB_DMA_REG(0x0D48, _channel)
#define MGB_DMA_TX_BASE_L(_channel)	MGB_DMA_REG(0x0D4C, _channel)
#define MGB_DMA_TX_HEAD_WB_H(_channel)	MGB_DMA_REG(0x0D50, _channel) /* head Writeback */
#define MGB_DMA_TX_HEAD_WB_L(_channel)	MGB_DMA_REG(0x0D54, _channel)
#define MGB_DMA_TX_HEAD(_channel)	MGB_DMA_REG(0x0D58, _channel)
#define MGB_DMA_TX_TAIL(_channel)	MGB_DMA_REG(0x0D5C, _channel)

#define MGB_DMA_RX_CONFIG0(_channel)	MGB_DMA_REG(0x0D40, _channel)
#define MGB_DMA_RX_CONFIG1(_channel)	MGB_DMA_REG(0x0D44, _channel)
#define MGB_DMA_RX_BASE_H(_channel)	MGB_DMA_REG(0x0D48, _channel)
#define MGB_DMA_RX_BASE_L(_channel)	MGB_DMA_REG(0x0D4C, _channel)
#define MGB_DMA_RX_HEAD_WB_H(_channel)	MGB_DMA_REG(0x0D50, _channel) /* head Writeback */
#define MGB_DMA_RX_HEAD_WB_L(_channel)	MGB_DMA_REG(0x0D54, _channel)
#define MGB_DMA_RX_HEAD(_channel)	MGB_DMA_REG(0x0D58, _channel)
#define MGB_DMA_RX_TAIL(_channel)	MGB_DMA_REG(0x0D5C, _channel)

#define MGB_DMA_RING_LEN_MASK		0xFFFF
#define MGB_DMA_IOC_ENBL		0x10000000
#define MGB_DMA_HEAD_WB_ENBL		0x20
#define MGB_DMA_RING_PAD_MASK		0x03000000
#define MGB_DMA_RING_PAD_0		0x00000000
#define MGB_DMA_RING_PAD_2		0x02000000



#define MGB_DESC_CTL_OWN		(1 << 16)
#define MGB_DESC_CTL_BUFLEN_MASK	(0x3FFF)

#define MGB_NEXT_RING_IDX(_idx)		(((_idx) + 1) % MGB_DMA_RING_SIZE)
#define MGB_RING_SPACE(_sc)		\
	(((_sc->tx_ring_data.last_head - _sc->tx_ring_data.last_tail - 1) \
	 + MGB_DMA_RING_SIZE ) % MGB_DMA_RING_SIZE )

/** PHY **/
#define MGB_MII_ACCESS			0x120
#define MGB_MII_DATA			0x124
#define MGB_MII_PHY_ADDR_MASK		0x1F
#define MGB_MII_PHY_ADDR_SHIFT		11
#define MGB_MII_REG_ADDR_MASK		0x1F
#define MGB_MII_REG_ADDR_SHIFT		6
#define MGB_MII_READ			0x0
#define MGB_MII_WRITE			0x2
#define MGB_MII_BUSY			0x1

/** Interrupt registers **/
#define MGB_INTR_STS			0x780
#define MGB_INTR_SET			0x784
#define MGB_INTR_ENBL_SET		0x788
#define MGB_INTR_ENBL_CLR		0x78C
#define MGB_INTR_TRIGGER		0x0200
#define MGB_INTR_STS_ANY		(0x1)
#define MGB_INTR_STS_RX			(0x1 << 24)
#define MGB_INTR_STS_TX			(0x1 << 16)
#define MGB_INTR_STS_TEST		(0x2 << 8)

#define MGB_STS_OK			( 0 )
#define MGB_STS_TIMEOUT 		(-1 )

#define CSR_READ_BYTE(sc, reg)		\
	bus_read_1(sc->regs, reg)

#define CSR_WRITE_BYTE(sc, reg, val)	\
	bus_write_1(sc->regs, reg, val)

#define CSR_UPDATE_BYTE(sc, reg, val)	\
	CSR_WRITE_BYTE(sc, reg, CSR_READ_BYTE(sc, reg) | (val))

#define CSR_READ_REG(sc, reg)		\
	bus_read_4(sc->regs, reg)

#define CSR_WRITE_REG(sc, reg, val)	\
	bus_write_4(sc->regs, reg, val)

#define CSR_UPDATE_REG(sc, reg, val)	\
	CSR_WRITE_REG(sc, reg, CSR_READ_REG(sc, reg) | (val))

#define CSR_READ_2_BYTES(sc, reg)	\
	bus_read_2(sc->regs, reg)

#define CSR_READ_REG_BYTES(sc, reg, dest, cnt)	\
	bus_read_region_1(sc->regs, reg, dest, cnt)

#define CSR_TRANSLATE_ADDR_LOW32(addr)		((uint64_t) (addr) & 0xFFFFFFFF)
#define CSR_TRANSLATE_ADDR_HIGH32(addr)		((uint64_t) (addr) >> 32)

struct mgb_vendor_info {
	uint16_t 	vid;
	uint16_t 	did;
	char 		*name;
} mgb_vendor_info;

struct mgb_irq {
	struct resource			*res;
	void				*handler;
};

enum mgb_dmac_cmd { DMAC_RESET, DMAC_START, DMAC_STOP };
enum mgb_fct_cmd { FCT_RESET, FCT_ENABLE, FCT_DISABLE };

struct mgb_ring_desc_addr {
	uint32_t				low;
	uint32_t				high;
};

struct mgb_ring_desc {
	uint32_t				ctl;
	struct mgb_ring_desc_addr		addr;
	uint32_t				sts;
};

/* TODO: Combine RX and TX rings into a single tag/map */
/* TODO: WoL */

#if 0
struct mgb_ring_info {
	uint32_t				head_wb;
	struct mgb_ring_desc			*ring;
}
#endif
#define MGB_HEAD_WB_PTR(_ring_info_ptr)		\
	((uint32_t *)(_ring_info_ptr))

#define MGB_RING_PTR(_ring_info_ptr)		\
	((struct mgb_ring_desc *)(MGB_HEAD_WB_PTR(_ring_info_ptr) + 1))

struct mgb_buffer_desc {
	struct mbuf			*m;
	bus_dmamap_t			 dmamap;
	struct mgb_ring_desc		*ring_desc;
	struct mgb_buffer_desc		*prev;
};

struct mgb_ring_data {
	bus_dma_tag_t			 tag;
	bus_dmamap_t			 dmamap;
	bus_addr_t			 head_wb_bus_addr;
	bus_addr_t			 ring_bus_addr;

	void				*ring_info;
	uint32_t			*head_wb;
	struct mgb_ring_desc		*ring;

	uint32_t			 last_head;
	uint32_t			 last_tail;
};

struct mgb_buffer_data {
	bus_dma_tag_t			 tag;
	bus_dmamap_t			 sparemap;
	struct mgb_buffer_desc		 desc[MGB_DMA_RING_SIZE];
};

struct mgb_softc {
	if_t				 ifp;
	device_t			 dev;

	struct resource			*regs;
	struct mgb_irq			 irq;

	bool 				 isr_test_flag;

	device_t			 miibus;

	int				 if_flags;
	int				 ethaddr;
	int				 flags;

	struct mtx			 mtx;
	struct callout			 watchdog;
	int				 timer;


	bus_dma_tag_t			 dma_parent_tag;
	struct mgb_ring_data		 rx_ring_data;
	struct mgb_ring_data		 tx_ring_data;
	struct mgb_buffer_data		 rx_buffer_data;
	struct mgb_buffer_data		 tx_buffer_data;

};


/* MTX macros */
#define MGB_LOCK(_sc)			mtx_lock(_sc.mtx)
#define MGB_UNLOCK(_sc)			mtx_unlock(_sc.mtx)
#define MGB_LOCK_ASSERT(_sc)		mtx_assert(_sc.mtx, MA_OWNED)


/* FLAGS */
#define MGB_FLAG_INTX			0x00000000
#define MGB_FLAG_MSI			0x00000001
#define MGB_FLAG_MSIX			0x00000002

#define MGB_INTR_FLAG_MASK		0x3

#endif /* _IF_MGB_H_ */
