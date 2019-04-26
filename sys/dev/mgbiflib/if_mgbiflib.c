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
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * Microchip LAN7430/LAN7431 Low Power PCIe to Gigabit Ethernet Controller driver.
 *
 * Product information:
 * LAN7430 https://www.microchip.com/wwwproducts/en/LAN7430
 *   - Integrated IEEE 802.3 compliant PHY
 * LAN7431 https://www.microchip.com/wwwproducts/en/LAN7431
 *   - RGMII Interface
 *
 * This driver uses the default 'ukphy' PHY driver.
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/endian.h>

/* Needed for debugging */
#include <sys/kdb.h>

/* Needed for KLD */
#include <sys/module.h>
#include <sys/param.h>
#include <sys/kernel.h>

/* Resource Alloc (work with PCI bus) */
#include <sys/bus.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <machine/resource.h>

/* Needed for Network I/F */
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_types.h>
#include <net/if_media.h>

/* Needed for iflib */
#include <net/iflib.h>
#include "ifdi_if.h"

/* Needed for PCI support */
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

/* Needed for MII support */
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include "miibus_if.h"

/* Registers and structures for MGB driver */
#include <dev/mgbiflib/if_mgbiflib.h>

static pci_vendor_info_t mgb_vendor_info_array[] = {
	PVID(MGB_MICROCHIP_VENDOR_ID, MGB_LAN7430_DEVICE_ID,
	    "Microchip LAN7430 PCIe Gigabit Ethernet Controller"),
	PVID(MGB_MICROCHIP_VENDOR_ID, MGB_LAN7431_DEVICE_ID,
	    "Microchip LAN7431 PCIe Gigabit Ethernet Controller"),
	PVID_END
};

/* Device methods */
static device_register_t 		mgb_register;

/* IFLIB methods */
static ifdi_attach_pre_t		mgb_attach_pre;
static ifdi_attach_post_t		mgb_attach_post;
static ifdi_detach_t			mgb_detach;

static ifdi_tx_queues_alloc_t		mgb_tx_queues_alloc;
static ifdi_rx_queues_alloc_t		mgb_rx_queues_alloc;
static ifdi_queues_free_t		mgb_queues_free;

static ifdi_init_t			mgb_init;
static ifdi_stop_t			mgb_stop;

static ifdi_msix_intr_assign_t		mgb_msix_intr_assign;
static ifdi_tx_queue_intr_enable_t	mgb_tx_queue_intr_enable;
static ifdi_rx_queue_intr_enable_t	mgb_rx_queue_intr_enable;
static ifdi_intr_enable_t		mgb_intr_enable_all;
static ifdi_intr_disable_t		mgb_intr_disable_all;

static ifdi_timer_t			mgb_timer;

/* IFLIB_TXRX methods */
static int				mgb_isc_txd_encap(void *, if_pkt_info_t);
static void				mgb_isc_txd_flush(void *, uint16_t, qidx_t);
static int 				mgb_isc_txd_credits_update(void *, uint16_t, bool);
static int				mgb_isc_rxd_available(void *, uint16_t, qidx_t, qidx_t);
static int				mgb_isc_rxd_pkt_get(void *, if_rxd_info_t);
static void 				mgb_isc_rxd_refill(void * , if_rxd_update_t);
static void 				mgb_isc_rxd_flush(void *, uint16_t, uint8_t, qidx_t);

/* Interrupts */
static driver_filter_t			mgb_legacy_intr;
static driver_filter_t			mgb_admin_intr;
static driver_filter_t			mgb_rxq_intr;
static bool 				mgb_intr_test(struct mgb_softc *);

/* MII methods */
static miibus_readreg_t			mgb_miibus_readreg;
static miibus_writereg_t		mgb_miibus_writereg;
static miibus_linkchg_t			mgb_miibus_linkchg;
static miibus_statchg_t			mgb_miibus_statchg;

static int				mgb_media_change(if_t);
static void 				mgb_media_status(if_t, struct ifmediareq *);

/* Helper/Test functions */
static int				mgb_test_bar(struct mgb_softc *);
static int				mgb_alloc_regs(struct mgb_softc *);
static int				mgb_release_regs(struct mgb_softc *);

static void				mgb_get_ethaddr(struct mgb_softc *,
					    struct ether_addr *);

static int				mgb_wait_for_bits(struct mgb_softc *,
					    int, int, int);
/* H/W init, reset and teardown helpers */
static int				mgb_hw_init(struct mgb_softc *);
static int				mgb_hw_reset(struct mgb_softc *);
static int				mgb_mac_init(struct mgb_softc *);
static int				mgb_dmac_reset(struct mgb_softc *);
static int				mgb_phy_reset(struct mgb_softc *);

static int				mgb_dma_init(struct mgb_softc *);
static int 				mgb_dma_tx_ring_init(struct mgb_softc *, int);
static int 				mgb_dma_rx_ring_init(struct mgb_softc *, int);

static int				mgb_dmac_control(struct mgb_softc *, int, int,
					    enum mgb_dmac_cmd);
static int				mgb_fct_control(struct mgb_softc *, int, int,
					    enum mgb_fct_cmd);

/*********************************************************************
 *  FreeBSD Device Interface Entry Points
 *********************************************************************/

static device_method_t mgb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_register,	mgb_register),
	DEVMETHOD(device_probe,		iflib_device_probe),
	DEVMETHOD(device_attach,	iflib_device_attach),
	DEVMETHOD(device_detach,	iflib_device_detach),
	DEVMETHOD(device_shutdown,	iflib_device_shutdown),
	DEVMETHOD(device_suspend,	iflib_device_suspend),
	DEVMETHOD(device_resume,	iflib_device_resume),

	/* MII Interface */
	DEVMETHOD(miibus_readreg,	mgb_miibus_readreg),
	DEVMETHOD(miibus_writereg,	mgb_miibus_writereg),
	DEVMETHOD(miibus_linkchg,	mgb_miibus_linkchg),
	DEVMETHOD(miibus_statchg,	mgb_miibus_statchg),

	DEVMETHOD_END
};

static driver_t mgb_driver = {
	"mgb", mgb_methods, sizeof(struct mgb_softc)
};

devclass_t mgb_devclass;
DRIVER_MODULE(mgb, pci, mgb_driver, mgb_devclass, NULL, NULL);
IFLIB_PNP_INFO(pci, mgb, mgb_vendor_info_array);
MODULE_VERSION(mgb, 1);

#if 0 /* MIIBUS_DEBUG */
/* If MIIBUS debug stuff is in attach then order matters. Use below instead. */
DRIVER_MODULE_ORDERED(miibus, mgb, miibus_driver, miibus_devclass, NULL, NULL,
    SI_ORDER_ANY);
#endif /* MIIBUS_DEBUG */
DRIVER_MODULE(miibus, mgb, miibus_driver, miibus_devclass, NULL, NULL);

MODULE_DEPEND(mgb, pci, 1, 1, 1);
MODULE_DEPEND(mgb, ether, 1, 1, 1);
MODULE_DEPEND(mgb, miibus, 1, 1, 1);
MODULE_DEPEND(mgb, iflib, 1, 1, 1);

static device_method_t mgb_iflib_methods[] = {
	DEVMETHOD(ifdi_attach_pre, mgb_attach_pre),
	DEVMETHOD(ifdi_attach_post, mgb_attach_post),
	DEVMETHOD(ifdi_detach, mgb_detach),

	DEVMETHOD(ifdi_init, mgb_init),
	DEVMETHOD(ifdi_stop, mgb_stop),

	DEVMETHOD(ifdi_tx_queues_alloc, mgb_tx_queues_alloc),
	DEVMETHOD(ifdi_rx_queues_alloc, mgb_rx_queues_alloc),
	DEVMETHOD(ifdi_queues_free, mgb_queues_free),

	DEVMETHOD(ifdi_msix_intr_assign, mgb_msix_intr_assign),
	DEVMETHOD(ifdi_tx_queue_intr_enable, mgb_tx_queue_intr_enable),
	DEVMETHOD(ifdi_rx_queue_intr_enable, mgb_rx_queue_intr_enable),
	DEVMETHOD(ifdi_intr_enable, mgb_intr_enable_all),
	DEVMETHOD(ifdi_intr_disable, mgb_intr_disable_all),

	DEVMETHOD(ifdi_timer, mgb_timer),

#if 0 /* UNUSED_IFLIB_METHODS */
	DEVMETHOD(ifdi_multi_set, vmxnet3_multi_set),
	DEVMETHOD(ifdi_mtu_set, vmxnet3_mtu_set),
	DEVMETHOD(ifdi_promisc_set, vmxnet3_promisc_set),
	DEVMETHOD(ifdi_get_counter, vmxnet3_get_counter),
	DEVMETHOD(ifdi_update_admin_status, vmxnet3_update_admin_status),

	DEVMETHOD(ifdi_link_intr_enable, vmxnet3_link_intr_enable),

	DEVMETHOD(ifdi_vlan_register, vmxnet3_vlan_register),
	DEVMETHOD(ifdi_vlan_unregister, vmxnet3_vlan_unregister),

	DEVMETHOD(ifdi_media_status, mgb_media_status),
	DEVMETHOD(ifdi_media_change, mgb_media_change),

	DEVMETHOD(ifdi_shutdown, vmxnet3_shutdown),
	DEVMETHOD(ifdi_suspend, vmxnet3_suspend),
	DEVMETHOD(ifdi_resume, vmxnet3_resume),
#endif /* UNUSED_IFLIB_METHODS */
	DEVMETHOD_END
};

static driver_t mgb_iflib_driver = {
	"mgb", mgb_iflib_methods, sizeof(struct mgb_softc)
};

struct if_txrx mgb_txrx  = {
	.ift_txd_encap = mgb_isc_txd_encap,
	.ift_txd_flush = mgb_isc_txd_flush,
	.ift_txd_credits_update = mgb_isc_txd_credits_update,
	.ift_rxd_available = mgb_isc_rxd_available, /* unimplemented */
	.ift_rxd_pkt_get = mgb_isc_rxd_pkt_get, /* unimplemented */
	.ift_rxd_refill = mgb_isc_rxd_refill,
	.ift_rxd_flush = mgb_isc_rxd_flush,

	.ift_legacy_intr = mgb_legacy_intr
};

struct if_shared_ctx mgb_sctx_init = {
	.isc_magic = IFLIB_MAGIC,

	.isc_q_align = PAGE_SIZE,
	/*
	 * setting admin_intrcnt to 0 (with nrxq/ntxq = 1) will confuse iflib.
	 * It'll think we're using legacy interrupts !!!
	 */
	.isc_admin_intrcnt = 1,
	/* IFLIB_GEN_MAC is only for pseudo-devices */
	/* .isc_flags = IFLIB_HAS_RXCQ | IFLIB_HAS_TXCQ, */

	.isc_vendor_info = mgb_vendor_info_array,
	.isc_driver_version = "1",
	.isc_driver = &mgb_iflib_driver,

	.isc_ntxqs = 2, /* One for wb, One for ring */

	.isc_tx_maxsize = MGB_DMA_RING_SIZE  * MCLBYTES,
	/* .isc_tx_nsegments = MGB_DMA_MAXSEGS, */
	.isc_tx_maxsegsize = MCLBYTES,

	.isc_ntxd_min = {1, 1}, /* Will want to make this bigger */
	.isc_ntxd_max = {MGB_DMA_RING_SIZE, 1},
	.isc_ntxd_default = {MGB_DMA_RING_SIZE, 1},

	.isc_nrxqs = 2, /* One for wb, one for ring */

	.isc_rx_maxsize = MCLBYTES,
	.isc_rx_nsegments = 1,
	.isc_rx_maxsegsize = MCLBYTES,

	.isc_nrxd_min = {1, 1}, /* Will want to make this bigger */
	.isc_nrxd_max = {MGB_DMA_RING_SIZE, 1},
	.isc_nrxd_default = {MGB_DMA_RING_SIZE, 1},

	.isc_nfl = 1, /* XXX: one free list for each queue */
#if 0 /* UNUSED_CTX */

	.isc_tso_maxsize = VMXNET3_TSO_MAXSIZE + sizeof(struct ether_vlan_header),
	.isc_tso_maxsegsize = VMXNET3_TX_MAXSEGSIZE,
#endif /* UNUSED_CTX */
};

/*********************************************************************/

/* struct callout timer; */

static void *
mgb_register(device_t dev)
{
	return (&mgb_sctx_init);
}

static int
mgb_attach_pre(if_ctx_t ctx)
{
	struct mgb_softc *sc;
	if_softc_ctx_t scctx;
	int error, phyaddr, rid;
	struct ether_addr hwaddr;

	struct mii_data *miid;

	sc = iflib_get_softc(ctx);
	sc->ctx = ctx;
	sc->dev = iflib_get_dev(ctx);
	scctx = iflib_get_softc_ctx(ctx);

	/* IFLIB required setup */
	scctx->isc_txrx = &mgb_txrx;
	scctx->isc_tx_nsegments =  MGB_DMA_RING_SIZE; /* MGB_DMA_MAXSEGS; */
	/* Ring desc queues */
	scctx->isc_txqsizes[0] = sizeof(struct mgb_ring_desc) * scctx->isc_ntxd[0];
	scctx->isc_rxqsizes[0] = sizeof(struct mgb_ring_desc) * scctx->isc_nrxd[0];

	/* Head WB queues */
	scctx->isc_txqsizes[1] = sizeof(uint32_t) * scctx->isc_ntxd[1];
	scctx->isc_rxqsizes[1] = sizeof(uint32_t) * scctx->isc_nrxd[1];

	/* XXX: Must have 1 txqset, but can have up to 4 rxqsets */
	scctx->isc_nrxqsets = 1;
	scctx->isc_ntxqsets = 1;

	/* scctx->isc_tx_csum_flags = (CSUM_TCP | CSUM_UDP) | (CSUM_TCP_IPV6 | CSUM_UDP_IPV6) | CSUM_TSO */
	scctx->isc_tx_csum_flags = 0;
	scctx->isc_capabilities = scctx->isc_capenable = 0;
#if 0
	/*
	 * iflib doesn't like capabilities = 0, so check needs to be disabled
	 * CSUM, TSO and VLAN support are TBD
	 */
	    IFCAP_TXCSUM | IFCAP_TXCSUM_IPV6 |
	    IFCAP_TSO4 | IFCAP_TSO6 |
	    IFCAP_RXCSUM | IFCAP_RXCSUM_IPV6 |
	    IFCAP_VLAN_MTU | IFCAP_VLAN_HWTAGGING |
	    IFCAP_VLAN_HWCSUM | IFCAP_VLAN_HWTSO |
	    IFCAP_JUMBO_MTU;
	scctx->isc_capabilities |= IFCAP_LRO | IFCAP_VLAN_HWFILTER;
#endif

	/* get the BAR */
	error = mgb_alloc_regs(sc);
	if(unlikely(error != 0)) {
		device_printf(sc->dev, "Unable to allocate bus resource: registers.\n");
		goto fail;
	}

	error = mgb_test_bar(sc);
	if(unlikely(error != 0))
		goto fail;

	error = mgb_hw_init(sc);
	if (unlikely(error != 0)) {
		device_printf(sc->dev, "MGB device init failed. (err: %d)\n", error);
		goto fail;
	}

	switch(pci_get_device(sc->dev))
	{
	case MGB_LAN7430_DEVICE_ID:
		phyaddr = 1;
		break;
	case MGB_LAN7431_DEVICE_ID:
	default:
		phyaddr = MII_PHY_ANY;
		break;
	}
	/* would be nice if it called iflib media methods ... diving into ifp struct is messy ... */
	error = mii_attach(sc->dev, &sc->miibus, iflib_get_ifp(ctx),
	    mgb_media_change, mgb_media_status,
	    BMSR_DEFCAPMASK, phyaddr, MII_OFFSET_ANY, MIIF_DOPAUSE);
	if(unlikely(error != 0)) {
		device_printf(sc->dev, "Failed to attach MII interface\n");
		goto fail;
	}

	miid = device_get_softc(sc->miibus);
	scctx->isc_media = &miid->mii_media;

	scctx->isc_msix_bar = pci_msix_table_bar(sc->dev);
	/** Setup PBA BAR **/
	rid = pci_msix_pba_bar(sc->dev);
	if (rid != scctx->isc_msix_bar) {
		sc->pba = bus_alloc_resource_any(sc->dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
		if (unlikely(sc->pba == NULL)) {
			error = ENXIO;
			device_printf(sc->dev, "Failed to setup PBA BAR\n");
			goto fail;
		}
	}

	/*
	 * XXX: This call should be updated to be more like iflib_gen_mac ...
	 * (or at least same OUI :P)
	 */
	mgb_get_ethaddr(sc, &hwaddr);
	if (unlikely(ETHER_IS_BROADCAST(hwaddr.octet) ||
	    ETHER_IS_MULTICAST(hwaddr.octet) ||
	    ETHER_IS_ZERO(hwaddr.octet)))
		ether_fakeaddr(&hwaddr);
	iflib_set_mac(ctx, hwaddr.octet);

	/* Clear all vector maps */
	CSR_WRITE_REG(sc, MGB_INTR_VEC_RX_MAP, 0);
	CSR_WRITE_REG(sc, MGB_INTR_VEC_TX_MAP, 0);
	CSR_WRITE_REG(sc, MGB_INTR_VEC_OTHER_MAP, 0);

	return (0);

fail:
	mgb_detach(ctx);
	return (error);
}

static int
mgb_attach_post(if_ctx_t ctx)
{
	struct mgb_softc *sc;

	sc = iflib_get_softc(ctx);

	device_printf(sc->dev, "Interrupt test: %s\n",
	    (mgb_intr_test(sc) ? "PASS" : "FAIL"));


	/* callout_init(&timer, 1); */
	return (0);
}

static int
mgb_detach(if_ctx_t ctx)
{
	struct mgb_softc *sc;
	int error;

	/* callout_drain(&timer); */

	sc = iflib_get_softc(ctx);
	/* Stop MAC */
	/* TODO: Should do a general hw_teardown */
	CSR_WRITE_REG(sc, MGB_MAC_RX, CSR_READ_REG(sc, MGB_MAC_RX) & ~MGB_MAC_ENBL);
	CSR_WRITE_REG(sc, MGB_MAC_TX, CSR_READ_REG(sc, MGB_MAC_TX) & ~MGB_MAC_ENBL);
	mgb_wait_for_bits(sc, MGB_MAC_RX, MGB_MAC_DSBL, 0);
	mgb_wait_for_bits(sc, MGB_MAC_TX, MGB_MAC_DSBL, 0);
	/* Release IRQs */
	iflib_irq_free(ctx, &sc->rx_irq);
	iflib_irq_free(ctx, &sc->admin_irq);

	if (sc->miibus != NULL)
		device_delete_child(sc->dev, sc->miibus);
	if (sc->pba != NULL)
		error = bus_release_resource(sc->dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->pba), sc->pba);
	sc->pba = NULL;

	error = mgb_release_regs(sc);

	return (error);
}

static int
mgb_media_change(if_t ifp)
{
	struct mii_data *miid;
	struct mii_softc *miisc;
	struct mgb_softc *sc;
	if_ctx_t ctx;
	int needs_reset;

	ctx = if_getsoftc(ifp);
	sc = iflib_get_softc(ctx);
	miid = device_get_softc(sc->miibus);
	LIST_FOREACH(miisc, &miid->mii_phys, mii_list)
		PHY_RESET(miisc);

	needs_reset = mii_mediachg(miid);
	if (needs_reset != 0)
		ifp->if_init(ctx);
	return (needs_reset);
}

static void
mgb_media_status(if_t ifp, struct ifmediareq *ifmr)
{
	struct mgb_softc *sc;
	struct mii_data *miid;

	sc = iflib_get_softc(if_getsoftc(ifp));
	miid = device_get_softc(sc->miibus);
	if ((if_getflags(ifp) & IFF_UP) == 0)
		return;

	mii_pollstat(miid);
	ifmr->ifm_active = miid->mii_media_active;
	ifmr->ifm_status = miid->mii_media_status;
}

static int
mgb_tx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs, uint64_t *paddrs,
    int ntxqs, int ntxqsets)
{
	struct mgb_softc *sc;
	struct mgb_ring_data *rdata;
	int q;

	sc = iflib_get_softc(ctx);
	rdata = &sc->tx_ring_data; /* there should only be one ring set */
	for (q = 0; q < ntxqsets; q++) {
		KASSERT(ntxqs == 2, ("ntxqs = %d", ntxqs));
		/* Ring */
		rdata->ring =
		    (struct mgb_ring_desc *) vaddrs[q * ntxqs + 0];
		rdata->ring_bus_addr = paddrs[q * ntxqs + 0];

		/* Head WB */
		rdata->head_wb =
		    (uint32_t *) vaddrs[q * ntxqs + 1];
		rdata->head_wb_bus_addr = paddrs[q * ntxqs + 1];
	}
	return 0;
}

static int
mgb_rx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs, uint64_t *paddrs,
    int nrxqs, int nrxqsets)
{
	struct mgb_softc *sc;
	struct mgb_ring_data *rdata;
	int q;

	sc = iflib_get_softc(ctx);
	rdata = &sc->rx_ring_data; /* there should only be one ring set */
	for (q = 0; q < nrxqsets; q++) {
		KASSERT(nrxqs == 2, ("nrxqs = %d", nrxqs));
		/* Ring */
		rdata->ring =
		    (struct mgb_ring_desc *) vaddrs[q * nrxqs + 0];
		rdata->ring_bus_addr = paddrs[q * nrxqs + 0];

		/* Head WB */
		rdata->head_wb =
		    (uint32_t *) vaddrs[q * nrxqs + 1];
		rdata->head_wb_bus_addr = paddrs[q * nrxqs + 1];
	}
	return 0;
}

static void
mgb_queues_free(if_ctx_t ctx)
{
	struct mgb_softc *sc;

	sc = iflib_get_softc(ctx);

	memset(&sc->rx_ring_data,
	    0, sizeof(struct mgb_ring_data));
	memset(&sc->tx_ring_data,
	    0, sizeof(struct mgb_ring_data));
}

static void
mgb_init(if_ctx_t ctx)
{
	struct mgb_softc *sc;
	struct mii_data *miid;
	int error;

	sc = iflib_get_softc(ctx);
	miid = device_get_softc(sc->miibus);
	device_printf(sc->dev, "running init ...\n");

	mgb_dma_init(sc);

	/* XXX: Turn off perfect filtering, turn on (broad|multi|uni)cast rx */
	CSR_WRITE_REG(sc, 0x508, CSR_READ_REG(sc, 0x508) & (~0x2)); /* Disable Perfect Filtering */
	/* CSR_UPDATE_REG(sc, 0x508, 0x2); */
	CSR_UPDATE_REG(sc, 0x508, 0x400);
	CSR_UPDATE_REG(sc, 0x508, 0x200);
	CSR_UPDATE_REG(sc, 0x508, 0x100);
	/* callout_reset(&timer, hz / 2, (timeout_t *)mgb_rxq_intr, sc); */

	error = mii_mediachg(miid);
	KASSERT(!error, ("mii_mediachg returned: %d", error));
}

static void
mgb_dump_all_stats(struct mgb_softc *sc)
{
	int i;
	int first_stat = 0x1200;
	int last_stat = 0x12FC;

	for(i = first_stat; i <= last_stat; i += 4)
		if (CSR_READ_REG(sc, i) != 0)
			device_printf(sc->dev, "0x%04x: 0x%08x\n", i, CSR_READ_REG(sc, i));
	char *stat_names[] = {
		"MAC_ERR_STS ",
		"FCT_INT_STS ",
		"DMAC_CFG ",
		"DMAC_CMD ",
		"DMAC_INT_STS ",
		"DMAC_INT_EN ",
		"DMAC_RX_ERR_STS0 ",
		"DMAC_RX_ERR_STS1 ",
		"DMAC_RX_ERR_STS2 ",
		"DMAC_RX_ERR_STS3 ",
		"INT_STS ",
		"INT_EN ",
		"INT_VEC_EN ",
		"INT_VEC_MAP0 ",
		"INT_VEC_MAP1 ",
		"INT_VEC_MAP2 ",
		"TX_HEAD0",
		"TX_TAIL0",
		"DMAC_TX_ERR_STS0 ",
		NULL
	};
	int stats[] = {
		0x114,
		0xA0,
		0xC00,
		0xC0C,
		0xC10,
		0xC14,
		0xC60,
		0xCA0,
		0xCE0,
		0xD20,
		0x780,
		0x788,
		0x794,
		0x7A0,
		0x7A4,
		0x780,
		0xD58,
		0xD5C,
		0xD60,
		0x0
	};
	i = 0;
	printf("==============================\n");
	while(stats[i++])
		device_printf(sc->dev, "%s at offset 0x%04x = 0x%08x\n", stat_names[i - 1], stats[i - 1], CSR_READ_REG(sc, stats[i - 1]));
	printf("==== TX RING DESCS ====\n");
	for (i = 0; i < MGB_DMA_RING_SIZE; i++)
		device_printf(sc->dev, "ring[%d].data0=0x%08x\n"
		    "ring[%d].data1=0x%08x\n"
		    "ring[%d].data2=0x%08x\n"
		    "ring[%d].data3=0x%08x\n",
		    i, sc->tx_ring_data.ring[i].ctl,
		    i, sc->tx_ring_data.ring[i].addr.low,
		    i, sc->tx_ring_data.ring[i].addr.high,
		    i, sc->tx_ring_data.ring[i].sts);
}

static void
mgb_stop(if_ctx_t ctx)
{
	struct mgb_softc *sc;

	sc = iflib_get_softc(ctx);

	/* callout_stop(&timer); */

	/* XXX: Could potentially timeout */
	/* XXX: should loop of txqsets and rxqsets */
	mgb_dmac_control(sc, MGB_DMAC_RX_START, 0, DMAC_STOP);
	mgb_fct_control(sc, MGB_FCT_RX_CTL, 0, FCT_DISABLE);
	mgb_dmac_control(sc, MGB_DMAC_TX_START, 0, DMAC_STOP);
	mgb_fct_control(sc, MGB_FCT_TX_CTL, 0, FCT_DISABLE);
}

static int
mgb_legacy_intr(void *xsc)
{
	struct mgb_softc *sc;

	sc = xsc;
	device_printf(sc->dev, "LEGACY INTR\n");
	iflib_admin_intr_deferred(sc->ctx);
	return (FILTER_HANDLED);
}

static int
mgb_rxq_intr(void *xsc)
{
	struct mgb_softc *sc;

	sc = xsc;
	device_printf(sc->dev, "CALLED RXQ INTR!!!\n");
	/* What do? */
	return (FILTER_SCHEDULE_THREAD);
}

static int
mgb_admin_intr(void *xsc)
{
	struct mgb_softc *sc;
	if_softc_ctx_t scctx;
	uint32_t intr_sts, intr_en;
	int qidx;

	sc = xsc;
	scctx = iflib_get_softc_ctx(sc->ctx);

	intr_sts = CSR_READ_REG(sc, MGB_INTR_STS);
	intr_en = CSR_READ_REG(sc, MGB_INTR_ENBL_SET);
	device_printf(sc->dev, "=== GENERIC ADMIN INTR ===\n");

	intr_sts &= intr_en;
	if ((intr_sts & MGB_INTR_STS_ANY) == 0)
	{
		device_printf(sc->dev, "NOT OURS ...\n");
		return (FILTER_SCHEDULE_THREAD);
	}
	if ((intr_sts &  MGB_INTR_STS_TEST) != 0)
	{
		sc->isr_test_flag = true;
		/* device_printf(sc->dev, "ADMIN_INTR for TEST"); */
		CSR_WRITE_REG(sc, MGB_INTR_STS, MGB_INTR_STS_TEST);
		return (FILTER_HANDLED);
	}
	if ((intr_sts & MGB_INTR_STS_RX_ANY) != 0)
	{
		device_printf(sc->dev, "=== GENERIC RX INTR ===\n");
		for (qidx = 0; qidx < scctx->isc_nrxqsets; qidx++) {
			if ((intr_sts & MGB_INTR_STS_RX(qidx))){
				device_printf(sc->dev, "=== SPECIFIC RX[%d] INTR ===\n", qidx);
				/* clear interrupts for this queue */
				CSR_WRITE_REG(sc, MGB_INTR_ENBL_CLR, MGB_INTR_STS_RX(qidx));
				CSR_WRITE_REG(sc, MGB_INTR_STS, MGB_INTR_STS_RX(qidx));
				iflib_rx_intr_deferred(sc->ctx, qidx);
			}
		}
		return (FILTER_HANDLED);
	}
	if ((intr_sts & MGB_INTR_STS_TX_ANY) != 0)
	{
		for (qidx = 0; qidx < scctx->isc_ntxqsets; qidx++) {
			if ((intr_sts & MGB_INTR_STS_RX(qidx))) {
				/* clear the interrupt sts and run handler */
				CSR_WRITE_REG(sc, MGB_INTR_ENBL_CLR, MGB_INTR_STS_TX(qidx));
				CSR_WRITE_REG(sc, MGB_INTR_STS, MGB_INTR_STS_TX(qidx));
				iflib_tx_intr_deferred(sc->ctx, qidx);
			}
		}
		return (FILTER_HANDLED);
	}
	device_printf(sc->dev, "NOT HANDLED  ...\n");

	/* TODO: shouldn't continue if suspended */
	return (FILTER_SCHEDULE_THREAD);
}

static int
mgb_msix_intr_assign(if_ctx_t ctx, int msix)
{
	struct mgb_softc *sc;
	if_softc_ctx_t scctx;
	int error, i, vectorid;
	char irq_name[16];

	sc = iflib_get_softc(ctx);
	scctx = iflib_get_softc_ctx(ctx);

	if(!(scctx->isc_nrxqsets == scctx->isc_ntxqsets == 1))
	{
		device_printf(sc->dev, "Assumption that rxqsets and txqsets == 1 is false.\n");
		return ENXIO;
	}
	/* First vector should be admin interrupts */
	vectorid = 1;
	error = iflib_irq_alloc_generic(ctx, &sc->admin_irq, vectorid,
	    IFLIB_INTR_ADMIN, mgb_admin_intr, sc, 0,
	    "admin");
	if (error) {
		device_printf(sc->dev,
		    "Failed to register admin interrupt handler\n");
		return (error);
	}

	/* All other vectors will be RX/TX interrupts */
	for (i = 0; i < scctx->isc_nrxqsets; i++) {
		vectorid += 1;
		snprintf(irq_name, sizeof(irq_name), "rxq%d", i);
		error = iflib_irq_alloc_generic(ctx, &sc->rx_irq, vectorid,
		    IFLIB_INTR_RX, mgb_rxq_intr, sc, i, irq_name);
		if (error) {
			device_printf(sc->dev,
			    "Failed to register rxq %d interrupt handler\n", i);
			return (error);
		}

		/* map vector */
		CSR_UPDATE_REG(sc, MGB_INTR_VEC_RX_MAP, MGB_INTR_VEC_MAP(vectorid, i));
	}

	/* Not actually mapping TX interrupts ... */
	for (i = 0; i < scctx->isc_ntxqsets; i++) {
		snprintf(irq_name, sizeof(irq_name), "txq%d", i);
		iflib_softirq_alloc_generic(ctx, NULL, IFLIB_INTR_TX, NULL, i,
		    irq_name);
		/* don't map vector ... */
		/* CSR_WRITE_REG(sc, MGB_INTR_VEC_TX_MAP, MGB_INTR_VEC_MAP(vectorid, i)); */
	}

	return (0);
}

static void
mgb_intr_enable_all(if_ctx_t ctx)
{
	struct mgb_softc *sc;
	if_softc_ctx_t scctx;
	int i, dmac_enable = 0, intr_sts = 0, vec_en = 0;

	sc = iflib_get_softc(ctx);
	scctx = iflib_get_softc_ctx(ctx);
	intr_sts |= MGB_INTR_STS_ANY;
	vec_en |= MGB_INTR_STS_ANY;

	for (i = 0; i < scctx->isc_nrxqsets; i++) {
		intr_sts |= MGB_INTR_STS_RX(i);
		dmac_enable |= MGB_DMAC_RX_INTR_ENBL(i);
		vec_en |= MGB_INTR_RX_VEC_STS(i);
	}
#if 0
	/* XXX: TX interrupts aren't needed ... */
	for (i = 0; i < scctx->isc_ntxqsets; i++) {
		intr_sts |= MGB_INTR_STS_TX(i);
		/* no vectors for tx */
		dmac_enable |= MGB_DMAC_TX_INTR_ENBL(i);
	}
#endif
	/* can use INTR_SET to force an interrupt ... */
	CSR_WRITE_REG(sc, MGB_INTR_ENBL_SET, intr_sts);
	CSR_WRITE_REG(sc, MGB_INTR_VEC_ENBL_SET, vec_en);
	CSR_WRITE_REG(sc, MGB_DMAC_INTR_STS, dmac_enable);
	CSR_WRITE_REG(sc, MGB_DMAC_INTR_ENBL_SET, dmac_enable);
}

static void
mgb_intr_disable_all(if_ctx_t ctx)
{
	struct mgb_softc *sc;

	sc = iflib_get_softc(ctx);
	CSR_WRITE_REG(sc, MGB_INTR_ENBL_CLR, UINT32_MAX);
	CSR_WRITE_REG(sc, MGB_INTR_VEC_ENBL_CLR, UINT32_MAX);
	CSR_WRITE_REG(sc, MGB_INTR_STS, UINT32_MAX);

	CSR_WRITE_REG(sc, MGB_DMAC_INTR_ENBL_CLR, UINT32_MAX);
	CSR_WRITE_REG(sc, MGB_DMAC_INTR_STS, UINT32_MAX);
}

static int
mgb_rx_queue_intr_enable(if_ctx_t ctx, uint16_t qid)
{
	/* called after successful rx isr */
	struct mgb_softc *sc;

	sc = iflib_get_softc(ctx);
	CSR_WRITE_REG(sc, MGB_INTR_VEC_ENBL_SET, MGB_INTR_RX_VEC_STS(qid));
	CSR_WRITE_REG(sc, MGB_INTR_ENBL_SET, MGB_INTR_STS_RX(qid));

	CSR_WRITE_REG(sc, MGB_DMAC_INTR_STS, MGB_DMAC_RX_INTR_ENBL(qid));
	CSR_WRITE_REG(sc, MGB_DMAC_INTR_ENBL_SET, MGB_DMAC_RX_INTR_ENBL(qid));
	return (0);
}

static int
mgb_tx_queue_intr_enable(if_ctx_t ctx, uint16_t qid)
{
	/* XXX: not called (since tx interrupts not used) */
	struct mgb_softc *sc;

	sc = iflib_get_softc(ctx);

	CSR_WRITE_REG(sc, MGB_INTR_ENBL_SET, MGB_INTR_STS_TX(qid));

	CSR_WRITE_REG(sc, MGB_DMAC_INTR_STS, MGB_DMAC_TX_INTR_ENBL(qid));
	CSR_WRITE_REG(sc, MGB_DMAC_INTR_ENBL_SET, MGB_DMAC_TX_INTR_ENBL(qid));
	return (0);
}

static bool
mgb_intr_test(struct mgb_softc *sc)
{
	int i;

	sc->isr_test_flag = false;
	CSR_WRITE_REG(sc, MGB_INTR_STS, MGB_INTR_STS_TEST);
	CSR_WRITE_REG(sc, MGB_INTR_VEC_ENBL_SET, MGB_INTR_STS_ANY);
	CSR_WRITE_REG(sc, MGB_INTR_ENBL_SET, MGB_INTR_STS_ANY | MGB_INTR_STS_TEST);
	CSR_WRITE_REG(sc, MGB_INTR_SET, MGB_INTR_STS_TEST);
	if(sc->isr_test_flag)
		return true;
	for (i = 0; i < MGB_TIMEOUT; i++) {
		DELAY(10);
		if(sc->isr_test_flag)
			break;
	}
	CSR_WRITE_REG(sc, MGB_INTR_ENBL_CLR, MGB_INTR_STS_TEST);
	CSR_WRITE_REG(sc, MGB_INTR_STS, MGB_INTR_STS_TEST);
	return sc->isr_test_flag;
}

static int
mgb_isc_txd_encap(void *xsc , if_pkt_info_t ipi)
{
	struct mgb_softc *sc;
	if_softc_ctx_t scctx;
	struct mgb_ring_data *rdata;
	struct mgb_ring_desc *txd;
	int i;

	qidx_t pidx, nsegs;
	bus_dma_segment_t *segs;

	KASSERT(ipi->ipi_qsidx == 0, ("tried to refill head_wb queue ....\n"));
	sc = xsc;
	scctx = iflib_get_softc_ctx(sc->ctx);
	rdata = &sc->tx_ring_data;

	pidx = ipi->ipi_pidx;
	segs = ipi->ipi_segs;
	nsegs = ipi->ipi_nsegs;
	/* For each seg, create a descriptor */
	device_printf(sc->dev,"%s(ipi.nsegs=%d, ipi.pidx=%d)\n", __func__, nsegs, pidx);

	for (i = 0; i < nsegs; ++i) {
		KASSERT(nsegs == 1, ("Multisegment packet !!!!!\n"));
		txd = &rdata->ring[pidx];
		txd->ctl = htole32((segs[i].ds_len & MGB_DESC_CTL_BUFLEN_MASK ) |
		    /*
		     * XXX: This will be wrong in the multipacket case
		     * I suspect FS should be for the first packet and
		     * LS should be for the last packet
		     */
		    MGB_TX_DESC_CTL_FS | MGB_TX_DESC_CTL_LS |
		    MGB_DESC_CTL_FCS);
		txd->addr.low = htole32(CSR_TRANSLATE_ADDR_LOW32(segs[i].ds_addr));
		txd->addr.high = htole32(CSR_TRANSLATE_ADDR_HIGH32(segs[i].ds_addr));
		txd->sts = htole32((segs[i].ds_len << 16) & MGB_DESC_FRAME_LEN_MASK);
		pidx = MGB_NEXT_RING_IDX(pidx);
	}
	ipi->ipi_new_pidx = pidx;
	return (0);
}

static void
mgb_isc_txd_flush(void *xsc, uint16_t txqid, qidx_t pidx)
{
	struct mgb_softc *sc;
	struct mgb_ring_data *rdata;

	KASSERT(txqid == 0, ("tried to flush head_wb queue ...."));
	sc = xsc;
	rdata = &sc->tx_ring_data;

	device_printf(sc->dev,"%s(txqid=%d, pidx=%d)\n", __func__, txqid, pidx);

	if (rdata->last_tail != pidx) {
		rdata->last_tail = pidx;
		CSR_WRITE_REG(sc, MGB_DMA_TX_TAIL(txqid), rdata->last_tail);
	}
}

static int
mgb_isc_txd_credits_update(void *xsc, uint16_t txqid, bool clear)
{
	struct mgb_softc *sc;
	struct mgb_ring_desc *txd;
	struct mgb_ring_data *rdata;
	int processed = 0;

	/*
	 * XXX:
	 *
	 * > If clear is true, we need to report the number of TX command ring
	 * > descriptors that have been processed by the device.  If clear is
	 * > false, we just need to report whether or not at least one TX
	 * > command ring descriptor has been processed by the device.
	 * - vmx driver
	 *
	 * This should be equivalent to tx_isr with the added option of just
	 * checking if there is anything to be processed.
	 */
	KASSERT(txqid == 0, ("tried to credits_update head_wb queue ...."));
	sc = xsc;
	rdata = &sc->tx_ring_data;

	device_printf(sc->dev,"%s(txqid=%d, clear=%d) w/ head_wb=%d, last_head=%d \n", __func__, txqid, clear, *(rdata->head_wb), rdata->last_head);
	if (CSR_READ_REG(sc, 0xD60)) {
		device_printf(sc->dev, "==== DUMP_TX_DMA_RAM (err=%08x) ====\n", CSR_READ_REG(sc, 0xD60));
		int i;
		CSR_WRITE_REG(sc, 0x24, 0xF); // DP_SEL & TX_RAM_0
		for(i = 0; i < 128; i++) {
			CSR_WRITE_REG(sc, 0x2C, i); // DP_ADDR

			CSR_WRITE_REG(sc, 0x28, 0); // DP_CMD

			while ((CSR_READ_REG(sc, 0x24) & 0x80000000) == 0) // DP_SEL & READY
				DELAY(1000);

			device_printf(sc->dev, "DMAC_TX_RAM_0[%u]=%08x\n", i, CSR_READ_REG(sc, 0x30)); // DP_DATA
		}
	}

	while(*(rdata->head_wb) != rdata->last_head) {
		if (!clear)
			return 1;

		txd = &rdata->ring[rdata->last_head];
		memset(txd, 0, sizeof(struct mgb_ring_desc));
		rdata->last_head = MGB_NEXT_RING_IDX(rdata->last_head);
		processed++;
	}

	return (processed);
}

static int
mgb_isc_rxd_available(void *xsc, uint16_t rxqid, qidx_t idx, qidx_t budget)
{
	struct mgb_softc *sc;
	if_softc_ctx_t scctx;
	struct mgb_ring_data *rdata;
	int avail = 0;

	sc = xsc;
	KASSERT(false, ("%s\n", __func__));
	device_printf(sc->dev, "Call to untested txrx func => '%s'\n", __func__);
	KASSERT(rxqid == 0, ("tried to check availability in head_wb queue ....\n"));

	rdata = &sc->rx_ring_data;
	scctx = iflib_get_softc_ctx(sc->ctx);
	for (; idx != *(rdata->head_wb);
	    idx = (idx + 1) % scctx->isc_nrxd[rxqid]) {
		avail++;
		/* Could verify desc is device owned here */
		if (avail == budget)
			break;
	}
	return (avail);
}

static int
mgb_isc_rxd_pkt_get(void *xsc, if_rxd_info_t ri)
{
	struct mgb_softc *sc;
	struct mgb_ring_data *rdata;
	struct mgb_ring_desc *rxd;

	sc = xsc;
	KASSERT(false, ("%s\n", __func__));
	device_printf(sc->dev, "Call to untested txrx func => '%s'\n", __func__);

	rdata = &sc->rx_ring_data;
	while (*(rdata->head_wb) != rdata->last_head) {
		rxd = &rdata->ring[rdata->last_head];
		if ((rxd->ctl & MGB_DESC_CTL_OWN) != 0)
			return EINVAL;
		if ((rxd->ctl & MGB_RX_DESC_CTL_FS) != 0)
			return EINVAL;
		if ((rxd->ctl & MGB_RX_DESC_CTL_LS) != 0)
			return EINVAL;
		ri->iri_frags[0].irf_flid = 0;
		ri->iri_frags[0].irf_idx = rdata->last_head;
		ri->iri_frags[0].irf_len = MGB_DESC_GET_FRAME_LEN(rxd);

		rdata->last_head = MGB_NEXT_RING_IDX(rdata->last_head);
		break;
	}
	ri->iri_nfrags = 1;

	return (0);
}

static void
mgb_isc_rxd_refill(void *xsc, if_rxd_update_t iru)
{
	if_softc_ctx_t scctx;

	struct mgb_softc *sc;
	struct mgb_ring_data *rdata;
	struct mgb_ring_desc *rxd;

	uint64_t *paddrs;
	qidx_t *idxs;
	qidx_t idx;
	int count, len;

	count = iru->iru_count;
	len = iru->iru_buf_size;
	idxs = iru->iru_idxs;
	paddrs = iru->iru_paddrs;
	KASSERT(iru->iru_qsidx == 0, ("tried to refill head_wb queue ....\n"));

	sc = xsc;
	scctx = iflib_get_softc_ctx(sc->ctx);
	rdata = &sc->rx_ring_data;

	device_printf(sc->dev, "Filling %d descriptors from %d to %d...\n", count, idxs[0], idxs[count - 1]);

	while (count > 0) {
		idx = idxs[--count];
		rxd = &rdata->ring[idx];
		/* rx_prepare_ring_element */
		rxd->addr.low = htole32(CSR_TRANSLATE_ADDR_LOW32(paddrs[count]));
		rxd->addr.high = htole32(CSR_TRANSLATE_ADDR_HIGH32(paddrs[count]));
		/* TODO: with register maps this could be done without masks etc. */
		rxd->ctl = htole32(MGB_DESC_CTL_OWN | (len & MGB_DESC_CTL_BUFLEN_MASK));
		rxd->sts = 0;
	}
	return;
}

static void
mgb_isc_rxd_flush(void *xsc, uint16_t rxqid, uint8_t flid, qidx_t pidx)
{
	struct mgb_softc *sc;

	sc = xsc;

	KASSERT(rxqid == 0, ("tried to flush head_wb queue ...."));
	device_printf(sc->dev, "Informing device that new tail is %d.\n", pidx);

	sc->rx_ring_data.last_tail = pidx;
	CSR_WRITE_REG(sc, MGB_DMA_RX_TAIL(rxqid), sc->rx_ring_data.last_tail);
	return;
}

static int
mgb_test_bar(struct mgb_softc *sc)
{
	/* Equivalent to chip_check_id */
	/* XXX Endian */
	uint32_t id_rev, dev_id, rev;
       	id_rev = CSR_READ_REG(sc, 0);
	dev_id = id_rev >> 16;
	rev = id_rev & 0xFFFF;
	if (dev_id == MGB_LAN7430_DEVICE_ID || dev_id == MGB_LAN7431_DEVICE_ID) {
		device_printf(sc->dev, "ID check passed (ID: 0x%x, REV: 0x%x)\n", dev_id, rev);
		return 0;
	}
	else {
		device_printf(sc->dev, "ID check failed (ID: 0x%x, REV: 0x%x)\n", dev_id, rev);
		return ENXIO;
	}
}

static int
mgb_alloc_regs(struct mgb_softc *sc)
{
	int rid;
	rid = PCIR_BAR(MGB_BAR);
	pci_enable_busmaster(sc->dev);
	sc->regs = bus_alloc_resource_any(sc->dev, SYS_RES_MEMORY,
	    &rid, RF_ACTIVE);
	if (unlikely(sc->regs == NULL))
		 return ENXIO;

	return (0);
}

static int
mgb_release_regs(struct mgb_softc *sc)
{
	int error = 0;

	if (sc->regs != NULL)
		error = bus_release_resource(sc->dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->regs), sc->regs);
	sc->regs = NULL;
	pci_disable_busmaster(sc->dev);
	return error;
}

static int
mgb_dma_init(struct mgb_softc *sc)
{
	if_softc_ctx_t scctx;
	int ch, error = 0;

	scctx = iflib_get_softc_ctx(sc->ctx);

	for (ch = 0; ch < scctx->isc_nrxqsets; ch++)
		if ((error = mgb_dma_rx_ring_init(sc, ch)) != 0)
			goto fail;

	for (ch = 0; ch < scctx->isc_nrxqsets; ch++)
		if ((error = mgb_dma_tx_ring_init(sc, ch)) != 0)
			goto fail;

	device_printf(sc->dev, "==== DUMP_TX_DMA_RAM ====\n");
	int i;
	CSR_WRITE_REG(sc, 0x24, 0xF); // DP_SEL & TX_RAM_0
	for(i = 0; i < 128; i++) {
		CSR_WRITE_REG(sc, 0x2C, i); // DP_ADDR

		CSR_WRITE_REG(sc, 0x28, 0); // DP_CMD

		while ((CSR_READ_REG(sc, 0x24) & 0x80000000) == 0) // DP_SEL & READY
			DELAY(1000);

		device_printf(sc->dev, "DMAC_TX_RAM_0[%u]=%08x\n", i, CSR_READ_REG(sc, 0x30)); // DP_DATA
	}

fail:
	return error;
}

static int
mgb_dma_rx_ring_init(struct mgb_softc *sc, int channel)
{
	struct mgb_ring_data *rdata;
	int ring_config, error = 0;

	rdata = &sc->rx_ring_data;
	mgb_dmac_control(sc, MGB_DMAC_RX_START, 0, DMAC_RESET);
	KASSERT(MGB_DMAC_STATE_IS_INITIAL(sc, MGB_DMAC_RX_START, channel), ("Trying to init channels when not in init state\n"));

	/* write ring address */
	if (rdata->ring_bus_addr == 0) {
		device_printf(sc->dev, "Invalid ring bus addr.\n");
		goto fail;
	}
	device_printf(sc->dev, "VIRTUAL RX_RING=%p RX_HEAD_WB=%p\n", rdata->ring, rdata->head_wb);
	device_printf(sc->dev, "PHYSICAL RX_RING=%016lx RX_HEAD_WB=%016lx\n", rdata->ring_bus_addr, rdata->head_wb_bus_addr);
	CSR_WRITE_REG(sc, MGB_DMA_RX_BASE_H(channel),
	    CSR_TRANSLATE_ADDR_HIGH32(rdata->ring_bus_addr));
	CSR_WRITE_REG(sc, MGB_DMA_RX_BASE_L(channel),
	    CSR_TRANSLATE_ADDR_LOW32(rdata->ring_bus_addr));

	/* write head pointer writeback address */
	if (rdata->head_wb_bus_addr == 0) {
		device_printf(sc->dev, "Invalid head wb bus addr.\n");
		goto fail;
	}
	CSR_WRITE_REG(sc, MGB_DMA_RX_HEAD_WB_H(channel),
	    CSR_TRANSLATE_ADDR_HIGH32(rdata->head_wb_bus_addr));
	CSR_WRITE_REG(sc, MGB_DMA_RX_HEAD_WB_L(channel),
	    CSR_TRANSLATE_ADDR_LOW32(rdata->head_wb_bus_addr));

	/* Enable head pointer writeback */
	CSR_WRITE_REG(sc, MGB_DMA_RX_CONFIG0(channel), MGB_DMA_HEAD_WB_ENBL);

	ring_config = CSR_READ_REG(sc, MGB_DMA_RX_CONFIG1(channel));
	/*  ring size */
	ring_config &= ~MGB_DMA_RING_LEN_MASK;
	ring_config |= (MGB_DMA_RING_SIZE & MGB_DMA_RING_LEN_MASK);
	/* packet padding  (PAD_2 is better for IP header alignment ...) */
	ring_config &= ~MGB_DMA_RING_PAD_MASK;
	ring_config |= (MGB_DMA_RING_PAD_0 & MGB_DMA_RING_PAD_MASK);

	CSR_WRITE_REG(sc, MGB_DMA_RX_CONFIG1(channel), ring_config);

	rdata->last_head = CSR_READ_REG(sc, MGB_DMA_RX_HEAD(channel));

	mgb_fct_control(sc, MGB_FCT_RX_CTL, channel, FCT_RESET);
	if (error != 0) {
		device_printf(sc->dev, "Failed to reset RX FCT.\n");
		goto fail;
	}
	mgb_fct_control(sc, MGB_FCT_RX_CTL, channel, FCT_ENABLE);
	if (error != 0) {
		device_printf(sc->dev, "Failed to enable RX FCT.\n");
		goto fail;
	}
	mgb_dmac_control(sc, MGB_DMAC_RX_START, channel, DMAC_START);
	if (error != 0)
		device_printf(sc->dev, "Failed to start RX DMAC.\n");
fail:
	return (error);
}

static int
mgb_dma_tx_ring_init(struct mgb_softc *sc, int channel)
{
	struct mgb_ring_data *rdata;
	int ring_config, error = 0;

	rdata = &sc->tx_ring_data;
	error = mgb_fct_control(sc, MGB_FCT_TX_CTL, channel, FCT_RESET);
	if (error != 0) {
		device_printf(sc->dev, "Failed to reset TX FCT.\n");
		goto fail;
	}
	error = mgb_fct_control(sc, MGB_FCT_TX_CTL, channel, FCT_ENABLE);
	if (error != 0) {
		device_printf(sc->dev, "Failed to enable TX FCT.\n");
		goto fail;
	}
	error = mgb_dmac_control(sc, MGB_DMAC_TX_START, channel, DMAC_RESET);
	if (error != 0) {
		device_printf(sc->dev, "Failed to reset TX DMAC.\n");
		goto fail;
	}
	KASSERT(MGB_DMAC_STATE_IS_INITIAL(sc, MGB_DMAC_TX_START, channel), ("Trying to init channels in not init state\n"));

	device_printf(sc->dev, "VIRTUAL TX_RING=%p TX_HEAD_WB=%p\n", rdata->ring, rdata->head_wb);
	device_printf(sc->dev, "PHYSICAL TX_RING=%016lx TX_HEAD_WB=%016lx\n", rdata->ring_bus_addr, rdata->head_wb_bus_addr);
	/* write ring address */
	if (rdata->ring_bus_addr == 0) {
		device_printf(sc->dev, "Invalid ring bus addr.\n");
		goto fail;
	}
	CSR_WRITE_REG(sc, MGB_DMA_TX_BASE_H(channel),
	    CSR_TRANSLATE_ADDR_HIGH32(rdata->ring_bus_addr));
	CSR_WRITE_REG(sc, MGB_DMA_TX_BASE_L(channel),
	    CSR_TRANSLATE_ADDR_LOW32(rdata->ring_bus_addr));

	/* write ring size */
	ring_config = CSR_READ_REG(sc, MGB_DMA_TX_CONFIG1(channel));
	ring_config &= ~MGB_DMA_RING_LEN_MASK;
	ring_config |= (MGB_DMA_RING_SIZE & MGB_DMA_RING_LEN_MASK);
	CSR_WRITE_REG(sc, MGB_DMA_TX_CONFIG1(channel), ring_config);

	/* Enable interrupt on completion and head pointer writeback */
	ring_config = (MGB_DMA_HEAD_WB_LS_ENBL | MGB_DMA_HEAD_WB_ENBL);
	CSR_WRITE_REG(sc, MGB_DMA_TX_CONFIG0(channel), ring_config);

	/* write head pointer writeback address */
	if (rdata->head_wb_bus_addr == 0) {
		device_printf(sc->dev, "Invalid head wb bus addr.\n");
		goto fail;
	}
	CSR_WRITE_REG(sc, MGB_DMA_TX_HEAD_WB_H(channel),
	    CSR_TRANSLATE_ADDR_HIGH32(rdata->head_wb_bus_addr));
	CSR_WRITE_REG(sc, MGB_DMA_TX_HEAD_WB_L(channel),
	    CSR_TRANSLATE_ADDR_LOW32(rdata->head_wb_bus_addr));

	rdata->last_head = CSR_READ_REG(sc, MGB_DMA_TX_HEAD(channel));
	KASSERT(rdata->last_head == 0, ("MGB_DMA_TX_HEAD != 0"));
	rdata->last_tail = 0;
	CSR_WRITE_REG(sc, MGB_DMA_TX_TAIL(channel), rdata->last_tail);

	error = mgb_dmac_control(sc, MGB_DMAC_TX_START, channel, DMAC_START);
	if (error != 0)
		device_printf(sc->dev, "Failed to start TX DMAC.\n");
fail:
	return error;
}

static int
mgb_dmac_control(struct mgb_softc *sc, int start, int channel, enum mgb_dmac_cmd cmd)
{
	int error = 0;
	switch (cmd) {
	case DMAC_RESET:
		CSR_WRITE_REG(sc, MGB_DMAC_CMD,
		    MGB_DMAC_CMD_RESET(start, channel));
		error = mgb_wait_for_bits(sc, MGB_DMAC_CMD,
				0, MGB_DMAC_CMD_RESET(start, channel));
		break;

	case DMAC_START:
		/* NOTE: this simplifies the logic, since it will never
		 * try to start in STOP_PENDING, but it also increases work.
		 */
		error = mgb_dmac_control(sc, start, channel, DMAC_STOP);
		if (error != 0)
			return error;
		CSR_WRITE_REG(sc, MGB_DMAC_CMD,
		    MGB_DMAC_CMD_START(start, channel));
		break;

	case DMAC_STOP:
		CSR_WRITE_REG(sc, MGB_DMAC_CMD,
		    MGB_DMAC_CMD_STOP(start, channel));
		error = mgb_wait_for_bits(sc, MGB_DMAC_CMD,
		    MGB_DMAC_CMD_STOP(start, channel),
		    MGB_DMAC_CMD_START(start, channel));
		break;
	}
	return error;
}

static int
mgb_fct_control(struct mgb_softc *sc, int reg, int channel, enum mgb_fct_cmd cmd)
{
	switch (cmd) {
	case FCT_RESET:
		CSR_WRITE_REG(sc, reg, MGB_FCT_RESET(channel));
		return mgb_wait_for_bits(sc, reg, 0, MGB_FCT_RESET(channel));
	case FCT_ENABLE:
		CSR_WRITE_REG(sc, reg, MGB_FCT_ENBL(channel));
		return (0);
	case FCT_DISABLE:
		CSR_WRITE_REG(sc, reg, MGB_FCT_DSBL(channel));
		return mgb_wait_for_bits(sc, reg, 0, MGB_FCT_ENBL(channel));
	}
}

static int
mgb_hw_init(struct mgb_softc *sc)
{
	int error = 0;

	error = mgb_hw_reset(sc);
	if (unlikely(error != 0))
		goto fail;

	mgb_mac_init(sc);

	error = mgb_phy_reset(sc);
	if (unlikely(error != 0))
		goto fail;

	error = mgb_dmac_reset(sc);
	if (unlikely(error != 0))
		goto fail;

fail:
	return error;
}

static int
mgb_hw_reset(struct mgb_softc *sc)
{
	CSR_UPDATE_REG(sc, MGB_HW_CFG, MGB_LITE_RESET);
	return (mgb_wait_for_bits(sc, MGB_HW_CFG, 0, MGB_LITE_RESET));
}

static int
mgb_mac_init(struct mgb_softc *sc)
{
	/**
	 * enable automatic duplex detection and
	 * automatic speed detection
	 */
	CSR_UPDATE_REG(sc, MGB_MAC_CR,
	    MGB_MAC_ADD_ENBL | MGB_MAC_ASD_ENBL);
	CSR_UPDATE_REG(sc, MGB_MAC_TX, MGB_MAC_ENBL);
	CSR_UPDATE_REG(sc, MGB_MAC_RX, MGB_MAC_ENBL);
	return MGB_STS_OK;
}

static int
mgb_phy_reset(struct mgb_softc *sc)
{
	CSR_UPDATE_BYTE(sc, MGB_PMT_CTL, MGB_PHY_RESET);
	if (mgb_wait_for_bits(sc, MGB_PMT_CTL, 0, MGB_PHY_RESET)
	    == MGB_STS_TIMEOUT)
		return MGB_STS_TIMEOUT;
	return (mgb_wait_for_bits(sc, MGB_PMT_CTL, MGB_PHY_READY, 0));
}

static int
mgb_dmac_reset(struct mgb_softc *sc)
{
	CSR_WRITE_REG(sc, MGB_DMAC_CMD, MGB_DMAC_RESET);
	return (mgb_wait_for_bits(sc, MGB_DMAC_CMD, 0, MGB_DMAC_RESET));
}


/*
 * Poll device register for bits to be written or cleared.
 */
static int
mgb_wait_for_bits(struct mgb_softc *sc, int reg, int set_bits, int clear_bits)
{
	int i, val;
	i = 0;

	do {
		DELAY(100); /* > 5us delay */
		val = CSR_READ_REG(sc, reg);
		if ((val & set_bits) == set_bits &&
		    (val & clear_bits) == 0)
			return MGB_STS_OK;
	} while(i++ < MGB_TIMEOUT);

	return MGB_STS_TIMEOUT;
}

static void
mgb_get_ethaddr(struct mgb_softc *sc, struct ether_addr *dest)
{
	CSR_READ_REG_BYTES(sc, MGB_MAC_ADDR_BASE_L, &dest->octet[0], 4);
	CSR_READ_REG_BYTES(sc, MGB_MAC_ADDR_BASE_H, &dest->octet[4], 2);
}

static int
mgb_miibus_readreg(device_t dev, int phy, int reg)
{
	struct mgb_softc *sc;
	int mii_access;

	sc = iflib_get_softc(device_get_softc(dev));
	/* for 7430 must be 1, for 7431 must be external phy */
	if (mgb_wait_for_bits(sc, MGB_MII_ACCESS, 0, MGB_MII_BUSY)
	    == MGB_STS_TIMEOUT)
		return EIO;
	/* XXX Endian  */
	mii_access = (phy & MGB_MII_PHY_ADDR_MASK) << MGB_MII_PHY_ADDR_SHIFT;
	mii_access |= (reg & MGB_MII_REG_ADDR_MASK) << MGB_MII_REG_ADDR_SHIFT;
	mii_access |= MGB_MII_BUSY | MGB_MII_READ;
	CSR_WRITE_REG(sc, MGB_MII_ACCESS, mii_access);
	if (mgb_wait_for_bits(sc, MGB_MII_ACCESS, 0, MGB_MII_BUSY)
	    == MGB_STS_TIMEOUT)
		return EIO;
	return (CSR_READ_2_BYTES(sc, MGB_MII_DATA));
}

static int
mgb_miibus_writereg(device_t dev, int phy, int reg, int data)
{

	struct mgb_softc *sc;
	int mii_access;

	sc = iflib_get_softc(device_get_softc(dev));

	if (mgb_wait_for_bits(sc, MGB_MII_ACCESS, 0, MGB_MII_BUSY)
	    == MGB_STS_TIMEOUT)
		return EIO;
	/* XXX Endian  */
	mii_access = (phy & MGB_MII_PHY_ADDR_MASK) << MGB_MII_PHY_ADDR_SHIFT;
	mii_access |= (reg & MGB_MII_REG_ADDR_MASK) << MGB_MII_REG_ADDR_SHIFT;
	mii_access |= MGB_MII_BUSY | MGB_MII_WRITE;
	CSR_WRITE_REG(sc, MGB_MII_DATA, data);
	CSR_WRITE_REG(sc, MGB_MII_ACCESS, mii_access);
	if(mgb_wait_for_bits(sc, MGB_MII_ACCESS, 0, MGB_MII_BUSY) == MGB_STS_TIMEOUT)
		return EIO;
	return 0;
}

/* XXX: Should probably lock these up */
static void
mgb_miibus_statchg(device_t dev)
{
	struct mgb_softc *sc;
	struct mii_data *miid;

	sc = iflib_get_softc(device_get_softc(dev));
	miid = device_get_softc(sc->miibus);
	/* Update baudrate in iflib */
	sc->baudrate = ifmedia_baudrate(miid->mii_media_active);
	iflib_link_state_change(sc->ctx, sc->link_state, sc->baudrate);
}

static void
mgb_miibus_linkchg(device_t dev)
{
	struct mgb_softc *sc;
	struct mii_data *miid;
	int link_state;

	sc = iflib_get_softc(device_get_softc(dev));
	miid = device_get_softc(sc->miibus);
	/* XXX: copied from miibus_linkchg **/
	if (miid->mii_media_status & IFM_AVALID) {
		if (miid->mii_media_status & IFM_ACTIVE)
			link_state = LINK_STATE_UP;
		else
			link_state = LINK_STATE_DOWN;
	} else
		link_state = LINK_STATE_UNKNOWN;
	sc->link_state = link_state;
	iflib_link_state_change(sc->ctx, sc->link_state, sc->baudrate);
}

static void
mgb_timer(if_ctx_t ctx, uint16_t qid) {
/* 	mgb_dump_all_stats(iflib_get_softc(ctx)); */
}
