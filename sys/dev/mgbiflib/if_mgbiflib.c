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
	PVID(MGB_MICROCHIP_VENDOR_ID, MGB_LAN7430_DEVICE_ID, "Microchip LAN7430 PCIe Gigabit Ethernet Controller"),
	PVID(MGB_MICROCHIP_VENDOR_ID, MGB_LAN7431_DEVICE_ID, "Microchip LAN7431 PCIe Gigabit Ethernet Controller"),
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

static ifdi_msix_intr_assign_t		mgb_msix_intr_assign;
static ifdi_tx_queue_intr_enable_t	mgb_tx_queue_intr_enable;
static ifdi_rx_queue_intr_enable_t	mgb_rx_queue_intr_enable;
static ifdi_intr_enable_t		mgb_intr_enable_all;
static ifdi_intr_disable_t		mgb_intr_disable_all;

/* IFLIB_TXRX methods */
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

/* Helper/Test functions */
static int				mgb_test_bar(struct mgb_softc *);

static int				mgb_alloc_regs(struct mgb_softc *);
static int				mgb_release_regs(struct mgb_softc *);

static int				mgb_dma_init(struct mgb_softc *);

static int				mgb_wait_for_bits(struct mgb_softc *,
					    int, int, int);

static int				mgb_hw_init(struct mgb_softc *);
static int				mgb_hw_reset(struct mgb_softc *);
static int				mgb_mac_init(struct mgb_softc *);
static int				mgb_dmac_reset(struct mgb_softc *);
static int				mgb_phy_reset(struct mgb_softc *);

static int 				mgb_dma_tx_ring_init(struct mgb_softc *);
static int 				mgb_dma_rx_ring_init(struct mgb_softc *);
static int				mgb_dmac_control(struct mgb_softc *, int, int,
					    enum mgb_dmac_cmd);
static int				mgb_fct_control(struct mgb_softc *, int, int,
					    enum mgb_fct_cmd);
#if 0 /* UNUSED_MGB_FUNCTIONS_1 */
/* MSI Interrupts support */
/* Interrupt helper functions */
static void 			mgb_intr_enable(struct mgb_softc *);
static void 			mgb_intr_disable(struct mgb_softc *);

static void			mgb_rxeof(struct mgb_softc *sc);
static void			mgb_txeof(struct mgb_softc *sc);
/* MAC support */
static void			mgb_get_ethaddr(struct mgb_softc *, struct ether_addr *);


/* MII MEDIA support */
static int			mgb_ifmedia_upd(struct ifnet *);
static void			mgb_ifmedia_sts(struct ifnet *,
				    struct ifmediareq *);

/* IFNET methods */
static int			mgb_transmit_init(if_t, struct mbuf *);
static void			mgb_qflush(if_t);
static int			mgb_ioctl(if_t, u_long, caddr_t);

/* DMA helper functions*/
static int			mgb_dma_alloc(device_t);
static bus_dmamap_callback_t	mgb_ring_dmamap_bind;
static bus_dmamap_callback_t	mgb_dmamap_bind;
static void			mgb_dma_teardown(struct mgb_softc *);


static void			mgb_stop(struct mgb_softc *);
static int			mgb_newbuf(struct mgb_softc *sc, int idx);
static void			mgb_discard_rxbuf(struct mgb_softc *sc, int idx);

#endif /* UNUSED_MGB_FUNCTIONS_1 */

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

	DEVMETHOD_END
};

static driver_t mgb_driver = {
	"mgb", mgb_methods, sizeof(struct mgb_softc)
};

devclass_t mgb_devclass;
DRIVER_MODULE(mgb, pci, mgb_driver, mgb_devclass, NULL, NULL);
IFLIB_PNP_INFO(pci, mgb, mgb_vendor_info_array);
MODULE_VERSION(mgb, 2);

#if 0 /* MIIBUS_DEBUG */
/* If MIIBUS debug stuff is in attach then order matters. Use below instead. */
DRIVER_MODULE_ORDERED(mgb, pci, mgb_driver, mgb_devclass, NULL, NULL,
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

	DEVMETHOD(ifdi_tx_queues_alloc, mgb_tx_queues_alloc),
	DEVMETHOD(ifdi_rx_queues_alloc, mgb_rx_queues_alloc),
	DEVMETHOD(ifdi_queues_free, mgb_queues_free),

	DEVMETHOD(ifdi_msix_intr_assign, mgb_msix_intr_assign),
	DEVMETHOD(ifdi_tx_queue_intr_enable, mgb_tx_queue_intr_enable),
	DEVMETHOD(ifdi_rx_queue_intr_enable, mgb_rx_queue_intr_enable),
	DEVMETHOD(ifdi_intr_enable, mgb_intr_enable_all),
	DEVMETHOD(ifdi_intr_disable, mgb_intr_disable_all),
#if 0 /* UNUSED_IFLIB_METHODS */

	DEVMETHOD(ifdi_stop, vmxnet3_stop),
	DEVMETHOD(ifdi_multi_set, vmxnet3_multi_set),
	DEVMETHOD(ifdi_mtu_set, vmxnet3_mtu_set),
	DEVMETHOD(ifdi_media_status, vmxnet3_media_status),
	DEVMETHOD(ifdi_media_change, vmxnet3_media_change),
	DEVMETHOD(ifdi_promisc_set, vmxnet3_promisc_set),
	DEVMETHOD(ifdi_get_counter, vmxnet3_get_counter),
	DEVMETHOD(ifdi_update_admin_status, vmxnet3_update_admin_status),
	DEVMETHOD(ifdi_timer, vmxnet3_txq_timer),

	DEVMETHOD(ifdi_link_intr_enable, vmxnet3_link_intr_enable),

	DEVMETHOD(ifdi_vlan_register, vmxnet3_vlan_register),
	DEVMETHOD(ifdi_vlan_unregister, vmxnet3_vlan_unregister),

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
#if 0 /* UNUSED_TXRX */
	.ift_txd_encap = vmxnet3_isc_txd_encap,
	.ift_txd_flush = vmxnet3_isc_txd_flush,
	.ift_txd_credits_update = vmxnet3_isc_txd_credits_update,
#endif /* UNUSED_TXRX */
	.ift_rxd_available = mgb_isc_rxd_available,
	.ift_rxd_pkt_get = mgb_isc_rxd_pkt_get, /* unimplemented */
	.ift_rxd_refill = mgb_isc_rxd_refill, /* unimplemented */
	.ift_rxd_flush = mgb_isc_rxd_flush, /* unimplemented */

	.ift_legacy_intr = mgb_legacy_intr
};

struct if_shared_ctx mgb_sctx_init = {
	.isc_magic = IFLIB_MAGIC,

	.isc_q_align = PAGE_SIZE,

	.isc_admin_intrcnt = 0,
	/*.isc_flags =  IFLIB_GEN_MAC| IFLIB_HAS_RXCQ | IFLIB_HAS_TXCQ, */

	.isc_vendor_info = mgb_vendor_info_array,
	.isc_driver_version = "1",
	.isc_driver = &mgb_iflib_driver,

	.isc_ntxqs = 2, /* One for wb, One for ring */

	.isc_tx_maxsize = MGB_DMA_MAXSEGS * MCLBYTES,
	/* .isc_tx_nsegments = MGB_DMA_MAXSEGS, */
	.isc_tx_maxsegsize = MCLBYTES,

	.isc_ntxd_min = {1, 1}, // Will want to make this bigger
	.isc_ntxd_max = {1, 1},
	.isc_ntxd_default = {1, 1},

	.isc_nrxqs = 2, /* One for wb, one for ring */

	.isc_rx_maxsize =  MCLBYTES,
	.isc_rx_nsegments = 1,
	.isc_rx_maxsegsize = MCLBYTES,

	.isc_nrxd_min = {1, 1}, // Will want to make this bigger
	.isc_nrxd_max = {1, 1},
	.isc_nrxd_default = {1, 1},

	.isc_nfl = 2, /* one free list for each receive command queue */
#if 0 /* UNUSED_CTX */


	.isc_tso_maxsize = VMXNET3_TSO_MAXSIZE + sizeof(struct ether_vlan_header),
	.isc_tso_maxsegsize = VMXNET3_TX_MAXSEGSIZE,


#endif /* UNUSED_CTX */
};

/*********************************************************************/

static void *
mgb_register(device_t dev)
{
	return (&mgb_sctx_init);
}

static int
mgb_attach_pre(if_ctx_t ctx)
{
	struct mgb_softc *sc;
	struct ifmedia *ifm;

	if_softc_ctx_t scctx;
	int error;

	struct mii_data *miid __unused;
	int phyaddr __unused;

	sc = iflib_get_softc(ctx);
	sc->ctx = ctx;
	sc->dev = iflib_get_dev(ctx);
	ifm = iflib_get_media(ctx);
	scctx = iflib_get_softc_ctx(ctx);


	/* IFLIB required setup */
	scctx->isc_txrx = &mgb_txrx;
	scctx->isc_tx_nsegments = MGB_DMA_MAXSEGS;
	/* Ring desc queues */
	scctx->isc_txqsizes[0] = sizeof(struct mgb_ring_desc) * scctx->isc_ntxd[0];
	scctx->isc_rxqsizes[0] = sizeof(struct mgb_ring_desc) * scctx->isc_nrxd[0];

	/* Head WB queues */
	scctx->isc_txqsizes[1] = sizeof(uint32_t) * scctx->isc_ntxd[1];
	scctx->isc_rxqsizes[1] = sizeof(uint32_t) * scctx->isc_nrxd[1];

	scctx->isc_nrxqsets = 1;
	scctx->isc_ntxqsets = 1;

	/* get the BAR */
	error = mgb_alloc_regs(sc);
	if(unlikely(error != 0)) {
		device_printf(sc->dev, "Unable to allocate bus resource: registers.\n");
		goto fail;
	}
	/** Verify that this is the correct BAR **/
	error = mgb_test_bar(sc);
	if(unlikely(error != 0))
		goto fail;

	error = mgb_hw_init(sc);
	if (unlikely(error != 0)) {
		device_printf(sc->dev, "MGB device init failed. (err: %d)\n", error);
		goto fail;
	}

#if 0
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
	/* fails because of issue with memes */
	error = mii_attach(sc->dev, &sc->miibus, sc->ifp,
	    ifm->ifm_change, ifm->ifm_status,
	    BMSR_DEFCAPMASK, phyaddr, MII_OFFSET_ANY, MIIF_DOPAUSE);
	if(unlikely(error != 0)) {
		device_printf(sc->dev, "Failed to attach MII interface\n");
		goto fail;
	}

       	/* modify the miibus media to be the ifm one?*/
	miid = device_get_softc(sc->miibus);
	ifm = &miid->mii_media;

	/* this should work */
	scctx->isc_msix_bar = pci_msix_table_bar(sc->dev);
	/* this is what I did the first time */
	scctx->isc_msix_bar = -1;
	/* This also doesn't work ... */
	scctx->isc_msix_bar = -1;
	scctx->isc_disable_msix = true;
#endif
	/* iflib_gen_mac(ctx); */
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
	return (0);
}

static int
mgb_detach(if_ctx_t ctx)
{
	struct mgb_softc *sc;
	int error;

	sc = iflib_get_softc(ctx);
	/* Release IRQs */
	iflib_irq_free(ctx, &sc->rx_irq);
	iflib_irq_free(ctx, &sc->admin_irq);

	if (sc->miibus != NULL)
		device_delete_child(sc->dev, sc->miibus);
	error = mgb_release_regs(sc);

	return (error);
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
	struct mii_data *miid __unused;

	sc = iflib_get_softc(ctx);
	/* miid = device_get_softc(sc->miibus); */

	mgb_dma_init(sc);
	/* If an error occurs then stop! */

	/* mii_mediachg(miid); */
}

static int
mgb_legacy_intr(void *xsc)
{
	struct mgb_softc *sc;

	sc = xsc;
	iflib_admin_intr_deferred(sc->ctx);
	return (FILTER_SCHEDULE_THREAD);
}

static int
mgb_rxq_intr(void *xsc)
{
	return (FILTER_SCHEDULE_THREAD);
}

static int
mgb_admin_intr(void *xsc)
{
	struct mgb_softc *sc;
	uint32_t intr_sts, intr_en;

	sc = xsc;
	device_printf(sc->dev, "Oh ... an interrupt occured.\n");
	intr_sts = CSR_READ_REG(sc, MGB_INTR_STS);
	intr_en = CSR_READ_REG(sc, MGB_INTR_ENBL_SET);
	device_printf(sc->dev, "sts =  0x%x, en = 0x%x\n", intr_sts, intr_en);

	intr_sts &= intr_en;
	/* XXX: Do test even if not UP ? */
	if((intr_sts & MGB_INTR_STS_ANY) == 0)
		return (FILTER_HANDLED);
	if(intr_sts &  MGB_INTR_STS_TEST) {
		sc->isr_test_flag = true;
		CSR_WRITE_REG(sc, MGB_INTR_STS, MGB_INTR_STS_TEST);
	}
	/* TODO: shouldn't continue if suspended! */
	return (FILTER_SCHEDULE_THREAD);
}

static int
mgb_msix_intr_assign(if_ctx_t ctx, int msix)
{
	struct mgb_softc *sc;
	if_softc_ctx_t scctx;
	int error, i;
	char irq_name[16];

	sc = iflib_get_softc(ctx);
	scctx = iflib_get_softc_ctx(ctx);

	if(scctx->isc_nrxqsets == scctx->isc_ntxqsets == 1) {}
	else {
		device_printf(iflib_get_dev(ctx), "Assumption that rxqsets and txqsets == 1 is false.\n");
		return ENXIO;
	}

	for (i = 0; i < scctx->isc_nrxqsets; i++) {
		snprintf(irq_name, sizeof(irq_name), "rxq%d", i);
		error = iflib_irq_alloc_generic(ctx, &sc->rx_irq, i + 1,
		    IFLIB_INTR_RX, mgb_rxq_intr, sc, i, irq_name);
		if (error) {
			device_printf(iflib_get_dev(ctx),
			    "Failed to register rxq %d interrupt handler\n", i);
			return (error);
		}
	}

	for (i = 0; i < scctx->isc_ntxqsets; i++) {
		snprintf(irq_name, sizeof(irq_name), "txq%d", i);
		iflib_softirq_alloc_generic(ctx, NULL, IFLIB_INTR_TX, NULL, i,
		    irq_name);
	}

	error = iflib_irq_alloc_generic(ctx, &sc->admin_irq,
	    scctx->isc_nrxqsets + 1, IFLIB_INTR_ADMIN, mgb_admin_intr, sc, 0,
	    "admin");
	if (error) {
		device_printf(iflib_get_dev(ctx),
		    "Failed to register event interrupt handler\n");
		return (error);
	}

	return (0);
}

static void
mgb_intr_enable_all(if_ctx_t ctx)
{
	struct mgb_softc *sc;
	if_softc_ctx_t scctx;
	int i, dmac_enable = 0, intr_sts = 0;

	sc = iflib_get_softc(ctx);
	scctx = iflib_get_softc_ctx(ctx);
	CSR_WRITE_REG(sc, MGB_INTR_ENBL_SET, MGB_INTR_STS_ANY);

	for (i = 0; i < scctx->isc_nrxqsets; i++) {
		intr_sts |= MGB_INTR_STS_RX(i);
		dmac_enable |= MGB_DMAC_RX_INTR_ENBL(i);
	}
	for (i = 0; i < scctx->isc_ntxqsets; i++) {
		intr_sts |= MGB_INTR_STS_TX(i);
		dmac_enable |= MGB_DMAC_TX_INTR_ENBL(i);
	}
	CSR_WRITE_REG(sc, MGB_INTR_ENBL_SET, intr_sts);
	CSR_WRITE_REG(sc, MGB_DMAC_INTR_ENBL_SET, dmac_enable);
}

static void
mgb_intr_disable_all(if_ctx_t ctx)
{
	struct mgb_softc *sc;

	sc = iflib_get_softc(ctx);

	CSR_WRITE_REG(sc, MGB_INTR_ENBL_CLR, ~0);
	CSR_WRITE_REG(sc, MGB_INTR_STS, ~0);

	CSR_WRITE_REG(sc, MGB_DMAC_INTR_STS, ~0);
	CSR_WRITE_REG(sc, MGB_DMAC_INTR_ENBL_CLR, ~0);
}

static int
mgb_rx_queue_intr_enable(if_ctx_t ctx, uint16_t qid)
{
	struct mgb_softc *sc;

	sc = iflib_get_softc(ctx);
	CSR_WRITE_REG(sc, MGB_INTR_ENBL_SET, MGB_INTR_STS_RX(qid));
	CSR_WRITE_REG(sc, MGB_DMAC_INTR_STS, MGB_DMAC_RX_INTR_ENBL(qid));
	CSR_WRITE_REG(sc, MGB_DMAC_INTR_ENBL_SET, MGB_DMAC_RX_INTR_ENBL(qid));
	return (0);
}

static int
mgb_tx_queue_intr_enable(if_ctx_t ctx, uint16_t qid)
{
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
	CSR_WRITE_REG(sc, MGB_INTR_ENBL_SET, MGB_INTR_STS_TEST);
	CSR_WRITE_REG(sc, MGB_INTR_SET, MGB_INTR_STS_TEST);
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
mgb_isc_rxd_available(void *xsc, uint16_t rxqid, qidx_t idx, qidx_t budget)
{
	struct mgb_softc *sc;

	sc = xsc;
	device_printf(sc->dev, "Call to unimplemented txrx func => '%s'\n", __func__);
	return (0);
}

static int
mgb_isc_rxd_pkt_get(void *xsc, if_rxd_info_t ri)
{
	struct mgb_softc *sc;

	sc = xsc;
	device_printf(sc->dev, "Call to unimplemented txrx func => '%s'\n", __func__);
	return (0);
}

static void
mgb_isc_rxd_refill(void *xsc, if_rxd_update_t iru)
{
	struct mgb_softc *sc;

	sc = xsc;
	device_printf(sc->dev, "Call to unimplemented txrx func => '%s\n'", __func__);
	return;
}
static void
mgb_isc_rxd_flush(void *xsc, uint16_t rxqid, uint8_t flid, qidx_t pidx)
{
	struct mgb_softc *sc;

	sc = xsc;
	device_printf(sc->dev, "Call to unimplemented txrx func => '%s\n'", __func__);
	return;
}

static int
mgb_test_bar(struct mgb_softc *sc)
{
	/* Equivalent to chip_check_id */
	/* XXX Endian */
	uint32_t id_rev = CSR_READ_REG(sc, 0) >> 16;
	if (id_rev == MGB_LAN7430_DEVICE_ID || id_rev == MGB_LAN7431_DEVICE_ID) {
		device_printf(sc->dev, "ID check passed (ID: 0x%x)\n", id_rev);
		return 0;
	}
	else {
		device_printf(sc->dev, "ID check failed (ID: 0x%x)\n", id_rev);
		return ENXIO;
	}
}

static int
mgb_alloc_regs(struct mgb_softc *sc)
{
	int rid;

	rid = PCIR_BAR(MGB_BAR);
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
	return error;
}

#if 00 /* UNUSED_MGB_FUNCTIONS_2 */
static int
mgb_probe(device_t dev)
{
	struct mgb_vendor_info *currvendor;
	currvendor = mgb_vendor_info_array;

	while(currvendor->name != NULL) {
		if((pci_get_vendor(dev) == currvendor->vid) &&
		   (pci_get_device(dev) == currvendor->did)) {
			device_set_desc(dev, currvendor->name);
			return (BUS_PROBE_DEFAULT);
		}
		currvendor++;
	}

	return (ENXIO); /* No such device or address */
}

static void
mgb_intr(void * arg)
{
	struct mgb_softc *sc;
	uint32_t intr_sts, intr_en;

	sc = arg;
	/* TODO: should lock this up */
	device_printf(sc->dev, "Oh ... an interrupt occured.");
	intr_sts = CSR_READ_REG(sc, MGB_INTR_STS);
	intr_en = CSR_READ_REG(sc, MGB_INTR_ENBL_SET);
	device_printf(sc->dev, "sts =  0x%x, en = 0x%x\n", intr_sts, intr_en);

	/* turn off interrupts */
	/* mgb_intr_disable(sc); */

	intr_sts &= intr_en;
	/* XXX: Do test even if not UP ? */
	if((intr_sts & MGB_INTR_STS_ANY) == 0)
		return;
	if(intr_sts &  MGB_INTR_STS_TEST) {
		sc->isr_test_flag = true;
		CSR_WRITE_REG(sc, MGB_INTR_STS, MGB_INTR_STS_TEST);
	}
	/* TODO: shouldn't continue if suspended! */
	if ((if_getflags(sc->ifp) & IFF_UP) == 0)
		return;
	/* RUNNING should also be set if up */
	if(intr_sts &  MGB_INTR_STS_TX) {
		mgb_txeof(sc);
	}
	if(intr_sts &  MGB_INTR_STS_RX) {
		mgb_rxeof(sc);
	}

	/* re-enable interrupts */
	/* CSR_WRITE_REG(sc, MGB_INTR_ENBL_SET, intr_en); */

}

static void
mgb_rxeof(struct mgb_softc *sc)
{
	struct mgb_buffer_desc *desc;
	struct mbuf *m;
	int head = *sc->rx_ring_data.head_wb;
	int len;

	device_printf(sc->dev, "rxeof()\n");

	/* Flush ring */
	bus_dmamap_sync(sc->rx_ring_data.tag, sc->rx_ring_data.dmamap,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	while (head != sc->rx_ring_data.last_head) {
		desc = &sc->rx_buffer_data.desc[sc->rx_ring_data.last_head];

		if ((desc->ring_desc->ctl & MGB_DESC_CTL_OWN) != 0)
			break;

		/*
		 * Multi-packet are not handled yet.
		 *
		 * Would involve chaining the mbufs and some h/w work
		 */
		if ((desc->ring_desc->ctl &
		    (MGB_DESC_CTL_FS | MGB_DESC_CTL_LS)) == 0)
			break;
		len = MGB_DESC_GET_FRAME_LEN(desc->ring_desc);

		/* forward to OS */
		{
			m = desc->m;
			/* copy length */
			m->m_len = len - ETHER_CRC_LEN;
			m->m_pkthdr.len = m->m_len;
			/* copy net if */
			m->m_pkthdr.rcvif = sc->ifp;
			/* do checksumming ... */
			/* add vtag ... */
			/* unlock before running this */
			if_input(sc->ifp, m);
			/* relock */
		}


		if(mgb_newbuf(sc, sc->rx_ring_data.last_head) != 0) {
			if_inc_counter(sc->ifp, IFCOUNTER_IQDROPS, 1);
			mgb_discard_rxbuf(sc, sc->rx_ring_data.last_head);
			goto next;
		}

		sc->rx_ring_data.last_tail = sc->rx_ring_data.last_head;
		sc->rx_ring_data.last_tail =
		    MGB_NEXT_RING_IDX(sc->rx_ring_data.last_head);
next:
		head = *sc->rx_ring_data.head_wb;
	}

	/* Flush ring */
	bus_dmamap_sync(sc->rx_ring_data.tag, sc->rx_ring_data.dmamap,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	CSR_WRITE_REG(sc, MGB_DMA_RX_TAIL(0), sc->rx_ring_data.last_tail);
}

static void
mgb_txeof(struct mgb_softc *sc)
{
	/* do nothing */
}


static int
mgb_attach(device_t dev)
{
	struct mgb_softc *sc;
	struct ether_addr ethaddr;
	int error;
    	int msic, msixc;


	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Prep mutex */
	mtx_init(&sc->mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK, MTX_DEF);
	callout_init_mtx(&sc->watchdog, &sc->mtx, 0);

	pci_enable_busmaster(dev);
	/* Allocate bus resources for using PCI bus */



	sc->ifp = if_alloc(IFT_ETHER);
	if(unlikely(sc->ifp == NULL)) {
		device_printf(dev, "Unable to allocate ifnet structure.\n");
		error = ENXIO;
		goto fail;
	}

	error = mgb_dma_alloc(dev);
	if (unlikely(error != 0)) {
		device_printf(dev, "MGB device DMA allocations failed.\n");
		goto fail;
	}

	mgb_get_ethaddr(sc, &ethaddr);
	if (ETHER_IS_BROADCAST(ethaddr.octet)) {
		ether_fakeaddr(&ethaddr);
		/* fakeaddr says it's unicast and local ... but it lies! */
		ethaddr.octet[0] &= ~0x1;
		ethaddr.octet[0] |= 0x2;
	}
	/** XXX: Disable any packet filtering test **/

	/* set perfect filtering addr */
	CSR_WRITE_REG(sc, 0x400, 0);
	CSR_WRITE_REG(sc, 0x404,
		ethaddr.octet[0] |
		(ethaddr.octet[1] << 8) |
		(ethaddr.octet[2] << 16) |
		(ethaddr.octet[3] << 24)
	);

	CSR_WRITE_REG(sc, 0x400,
		ethaddr.octet[4] |
		(ethaddr.octet[5] << 8) |
		0x80000000
	);

	/* enable perfect filtering */
	CSR_UPDATE_REG(sc, 0x508, 0x2);

	/* disable perfect filtering */
	CSR_WRITE_REG(sc, 0x508, CSR_READ_REG(sc, 0x508) & (~0x2));
	/* enable broadcast recieve */
	CSR_UPDATE_REG(sc, 0x508, 0x400);


	/** XXX: turn on link autonegotiation */
	mgb_miibus_writereg(dev, 1, 0, 0x1200);


	rid = 0; /* use INTx interrupts by default */
	msic = pci_msi_count(dev);
	msixc = pci_msix_count(dev);

	/* MSIX > MSI > INTx */
	if (msixc > 0) {
		msixc = 1; /* Request only one line */
		if (pci_alloc_msix(dev, &msixc) == 0) {
			if (msixc == 1) {
				sc->flags |= MGB_FLAG_MSIX;
				rid = 1;
				device_printf(dev, "Using 1 MSI-X lane.");
			}
			else {
				pci_release_msi(dev);
				msixc = 0;
			}
		}
	}

	if (msixc == 0 && msic > 0) {
		msic = 1; /* Request only one line */
		if (pci_alloc_msi(dev, &msic) == 0) {
			if (msic == 1) {
				sc->flags |= MGB_FLAG_MSI;
				rid = 1;
				device_printf(dev, "Using 1 MSI lane.");
			}
			else {
				pci_release_msi(dev);
				msic = 0;
			}
		}
	}

	sc->irq.res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &rid, RF_SHAREABLE | RF_ACTIVE);
	if (unlikely(sc->irq.res == NULL)) {
		device_printf(dev, "Unable to allocate bus resource: interrupts\n");
		error = ENXIO;
		goto fail;
	}

	/*
	 * Should be sysctl tunable
	 */
	if_initname(sc->ifp, device_get_name(dev), device_get_unit(dev));
	if_setdev(sc->ifp, dev);
	if_setsoftc(sc->ifp, sc);

	if_setflags(sc->ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setinitfn(sc->ifp, mgb_init);
	if_settransmitfn(sc->ifp, mgb_transmit_init);
	if_setqflushfn(sc->ifp, mgb_qflush);
	if_setioctlfn(sc->ifp, mgb_ioctl);
	/* if_setgetcounterfn */

	/* if_setsendqlen( */
	/* if_setsendqready */
	if_setmtu(sc->ifp, ETHERMTU);
	if_setbaudrate(sc->ifp, IF_Mbps(1000));
	if_setcapabilities(sc->ifp, IFCAP_HWCSUM |
	    IFCAP_VLAN_MTU | IFCAP_VLAN_HWCSUM | IFCAP_VLAN_HWTAGGING);
	if_setcapenable(sc->ifp, if_getcapabilities(sc->ifp));

	mgb_intr_enable(sc);
	/* Add IRQ handler */

	ether_ifattach(sc->ifp, ethaddr.octet);

fail:
	if (error)
		mgb_detach(dev);

	return (error);
}

static int
mgb_ifmedia_upd(struct ifnet *ifp)
{
	struct mii_data *miid;
	struct mii_softc *miisc;
	struct mgb_softc *sc;

	sc = if_getsoftc(ifp);
	miid = device_get_softc(sc->miibus);
	LIST_FOREACH(miisc, &miid->mii_phys, mii_list)
		PHY_RESET(miisc);

	return (mii_mediachg(miid));
}

static void
mgb_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct mgb_softc *sc;
	struct mii_data *miid;

	sc = if_getsoftc(ifp);
	miid = device_get_softc(sc->miibus);
	if ((if_getflags(ifp) & IFF_UP) == 0)
		return;

	mii_pollstat(miid);
	ifmr->ifm_active = miid->mii_media_active;
	ifmr->ifm_status = miid->mii_media_status;
}

static int
mgb_detach(device_t dev)
{
	struct mgb_softc *sc;
	int uses_msi;

	sc = device_get_softc(dev);
	uses_msi = (sc->flags & MGB_INTR_FLAG_MASK) != MGB_FLAG_INTX;

	if (device_is_attached(dev)) {
		ether_ifdetach(sc->ifp);
		mgb_stop(sc);
		callout_drain(&sc->watchdog);
	}

	if (sc->miibus)
		device_delete_child(dev, sc->miibus);
	bus_generic_detach(dev);

	if (sc->regs)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    PCIR_BAR(MGB_BAR), sc->regs);

	if (sc->irq.handler)
		bus_teardown_intr(dev, sc->irq.res, sc->irq.handler);
	if (sc->irq.res)
		bus_release_resource(dev, SYS_RES_IRQ,
		    uses_msi ? 1 : 0, sc->irq.res);
	if (uses_msi)
		pci_release_msi(dev);

	if (sc->ifp)
		if_free(sc->ifp);
	/* DMA free */
	mgb_dma_teardown(sc);
	mtx_destroy(&sc->mtx);
	kdb_enter(KDB_WHY_UNSET, "Something failed in MGB so entering debugger...");
	return (0);
}

static void
mgb_stop(struct mgb_softc *sc)
{
	struct mgb_buffer_desc *desc;
	int i;

	/* turn off interrupts */
	mgb_intr_disable(sc);

	/* stop fct and dmac */
	/* XXX: Could potentially timeout */
	mgb_dmac_control(sc, MGB_DMAC_RX_START, 0, DMAC_STOP);
	mgb_fct_control(sc, MGB_FCT_RX_CTL, 0, FCT_DISABLE);
	mgb_dmac_control(sc, MGB_DMAC_TX_START, 0, DMAC_STOP);
	mgb_fct_control(sc, MGB_FCT_TX_CTL, 0, FCT_DISABLE);

	/* txeof ? */
	/* Free floating mbufs */
	for (i = 0; i < MGB_DMA_RING_SIZE; i++)
	{
		desc = &sc->rx_buffer_data.desc[i];
		if (desc->m != NULL) {
			bus_dmamap_sync(sc->rx_ring_data.tag, desc->dmamap,
			    BUS_DMASYNC_POSTREAD);
			bus_dmamap_unload(sc->rx_ring_data.tag, desc->dmamap);

			m_freem(desc->m);
			desc->m = NULL;
		}
	}

	for (i = 0; i < MGB_DMA_RING_SIZE; i++)
	{
		desc = &sc->tx_buffer_data.desc[i];
		if (desc->m != NULL) {
			bus_dmamap_sync(sc->tx_ring_data.tag, desc->dmamap,
			    BUS_DMASYNC_POSTREAD);
			bus_dmamap_unload(sc->tx_ring_data.tag, desc->dmamap);

			m_freem(desc->m);
			desc->m = NULL;
			if_inc_counter(sc->ifp, IFCOUNTER_OERRORS, 1);
		}
	}
}

static void
mgb_get_ethaddr(struct mgb_softc *sc, struct ether_addr *dest)
{
	CSR_READ_REG_BYTES(sc, MGB_MAC_ADDR_BASE_L, &dest->octet[0], 4);
	CSR_READ_REG_BYTES(sc, MGB_MAC_ADDR_BASE_H, &dest->octet[4], 2);
}

static void
mgb_init(void *arg)
{
	struct mgb_softc *sc;
	struct mii_data *miid;

	sc = (struct mgb_softc *)arg;
	miid = device_get_softc(sc->miibus);
	/*
	 * TODO: Should lock this up!
	 * (will want to run this other places so should split
	 * functionality and locking)
	 */

	if (mgb_dma_init(sc) != 0)
		return;
	/* If an error occurs then stop! */

	if_setdrvflagbits(sc->ifp, IFF_DRV_RUNNING, IFF_DRV_OACTIVE);
	mii_mediachg(miid);
}

static int
mgb_transmit_init(if_t ifp, struct mbuf *m)
{
	struct mgb_softc *sc;

	sc = if_getsoftc(ifp);

	/* XXX: ?????????
	Don't do anything if Running and !Active
	*/
	for (enq = 0; !IFQ_DRV_IS_EMPTY(&ifp->if_snd) &&
	    sc->vge_cdata.vge_tx_cnt < VGE_TX_DESC_CNT - 1; ) {
		IFQ_DRV_DEQUEUE(&ifp->if_snd, m_head);
		if (m_head == NULL)
			break;
		/*
		 * Pack the data into the transmit ring. If we
		 * don't have room, set the OACTIVE flag and wait
		 * for the NIC to drain the ring.
		 */
		if (mgb_encap(sc, &m_head)) {
			if (m_head == NULL)
				break;
			IFQ_DRV_PREPEND(&ifp->if_snd, m_head);
			ifp->if_drv_flags |= IFF_DRV_OACTIVE;
			break;
		}

		enq++;
		/*
		 * If there's a BPF listener, bounce a copy of this frame
		 * to him.
		 */
		ETHER_BPF_MTAP(ifp, m_head);
	}

	if enq == 0 then done
	otherwise
		bus_dma_sync(tx_ring)
		mgb_dma_tx_start()
		start watchdog timer (usually set to 5)
	device_printf(sc->dev, "transmit_init()\n");
	return 0;
}

static void
mgb_qflush(if_t ifp)
{
	struct mgb_softc *sc;

	sc = if_getsoftc(ifp);
	device_printf(sc->dev, "qflush()\n");
	device_printf(
	    sc->dev, "RX_DROPPED_FRAMES: %d\n"
	    "RX_TOTAL_FRAMES: %d\n"
	    "RX_BROADCAST_FRAMES: %d\n"
	    "RX_BROADCAST_BYTES: %d\n"
	    "err1: %d\n"
	    "err2: %d\n"
	    "err3: %d\n"
	    "err4: %d\n"
	    "err5: %d\n"
	    "err6: %d\n",
	    CSR_READ_REG(sc, MGB_MAC_STAT_RX_DROPPED_CNT),
	    CSR_READ_REG(sc, MGB_MAC_STAT_RX_FRAME_CNT),
	    CSR_READ_REG(sc, MGB_MAC_STAT_RX_BROADCAST_CNT1),
	    CSR_READ_REG(sc, MGB_MAC_STAT_RX_BROADCAST_CNT),
	    CSR_READ_REG(sc, MGB_MAC_STAT_RX_FCS_ERR_CNT),
	    CSR_READ_REG(sc, MGB_MAC_STAT_RX_ALIGN_ERR_CNT),
	    CSR_READ_REG(sc, MGB_MAC_STAT_RX_FRAG_ERR_CNT),
	    CSR_READ_REG(sc, MGB_MAC_STAT_RX_JABBER_ERR_CNT),
	    CSR_READ_REG(sc, MGB_MAC_STAT_RX_UNDER_ERR_CNT),
	    CSR_READ_REG(sc, MGB_MAC_STAT_RX_OVER_ERR_CNT)
	);
}

static int
mgb_ioctl(if_t ifp, u_long command, caddr_t data)
{
	int error = 0;
	struct ifreq *ifr;
	struct mgb_softc *sc;
	struct mii_data *miid;

	sc = if_getsoftc(ifp);
	miid = device_get_softc(sc->miibus);
	ifr  = (struct ifreq *)data;

	switch (command) {
	case SIOCSIFFLAGS:
		if_printf(ifp, "Does nothing for SET IF FLAGS\n");
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		if_printf(ifp, "Does nothing for ADD/DEL MULTI\n");
		break;
	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr,  &miid->mii_media, command);
		break;
	case SIOCSIFCAP:
		if_printf(ifp, "Does nothing for SET IF CAPABILITIES\n");
		break;
	case SIOCSIFMTU:
		/* Default may be sufficient */
	default:
		error = ether_ioctl(ifp, command, data);
		break;
	}

	return (error);
}

static int
mgb_newbuf(struct mgb_softc *sc, int idx)
{
	struct mbuf *m = NULL;
	struct mgb_buffer_desc *desc;
	bus_dma_segment_t segs[1];
	bus_dmamap_t map;
	int error = 0, /*i,*/ nsegs;

	error = bus_dmamap_load_mbuf_sg(
	    sc->rx_buffer_data.tag, sc->rx_buffer_data.sparemap,
	    m, segs, &nsegs, 0);
	if(error != 0) {
		m_freem(m);
		return (ENOBUFS);
	}

	KASSERT(nsegs == 1,
	    ("%s: expected 1 DMA segment, found %d!", __func__, nsegs));

	desc = &sc->rx_buffer_data.desc[idx];

	if (desc->m != NULL) {
		bus_dmamap_sync(sc->rx_buffer_data.tag, desc->dmamap,
		    BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(sc->rx_buffer_data.tag, desc->dmamap);
	}
	map = desc->dmamap;
	desc->dmamap = sc->rx_buffer_data.sparemap;
	sc->rx_buffer_data.sparemap = desc->dmamap;
	bus_dmamap_sync(sc->rx_buffer_data.tag, desc->dmamap,
	    BUS_DMASYNC_PREREAD);
	desc->m = m;

	desc->ring_desc->sts = 0;
	/* XXX: Endian */
	desc->ring_desc->addr.low = CSR_TRANSLATE_ADDR_LOW32(segs[0].ds_addr);
	desc->ring_desc->addr.high = CSR_TRANSLATE_ADDR_HIGH32(segs[0].ds_addr);
	desc->ring_desc->ctl = MGB_DESC_CTL_OWN |
	    (segs[0].ds_len & MGB_DESC_CTL_BUFLEN_MASK);
	return (error);
}

static void
mgb_discard_rxbuf(struct mgb_softc *sc, int idx)
{
	/* Fill with zeros */
	memset(sc->rx_buffer_data.desc[idx].ring_desc,
	    0, sizeof(struct mgb_ring_desc));
}

static void
mgb_ring_dmamap_bind(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
	struct mgb_ring_data *ring_data;

	if (error != 0)
		return;

	KASSERT(nsegs == 1,
	    ("%s: expected 1 DMA segment, found %d!", __func__, nsegs));

	ring_data = (struct mgb_ring_data *)arg;
	if (segs[0].ds_addr != 0) {
		ring_data->head_wb_bus_addr = segs[0].ds_addr;
		ring_data->ring_bus_addr = segs[0].ds_addr + sizeof(uint32_t);
	}
	else {
		ring_data->head_wb_bus_addr = 0;
		ring_data->ring_bus_addr = 0;
	}
}

/* TODO: unused since ring_dmamap_bind does all necessary work */
static void
mgb_dmamap_bind(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
	bus_addr_t *addr;

	if (error != 0)
		return;

	KASSERT(nsegs == 1,
	    ("%s: expected 1 DMA segment, found %d!", __func__, nsegs));

	addr = (bus_addr_t *)arg;
	*addr = segs[0].ds_addr;
}

static int
mgb_dma_alloc(device_t dev)
{
	struct mgb_softc *sc;
	int error = 0, i;
	struct mgb_buffer_desc *desc;

	sc = device_get_softc(dev);
	/*
	 * `lowaddr` is BUS_SPACE_MAXADDR all the time
	 * since this device will always be connected via PCIE
	 */
	error = bus_dma_tag_create(bus_get_dma_tag(dev),/* parent */
	    1, 0,			/* algnmnt, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    BUS_SPACE_MAXSIZE,	/* maxsize */
	    0,				/* nsegments */
	    BUS_SPACE_MAXSIZE,	/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->dma_parent_tag);
	if (error != 0) {
		device_printf(dev, "Couldn't create parent DMA tag.\n");
		goto fail;
	}

	/* TX mbufs */
	error = bus_dma_tag_create(sc->dma_parent_tag,/* parent */
	    1, 0,			/* algnmnt, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MGB_DMA_RING_INFO_SIZE,	/* maxsize */
	    1,				/* nsegments */
	    MGB_DMA_RING_INFO_SIZE,	/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->tx_ring_data.tag);
	if (error != 0) {
		device_printf(dev, "Couldn't create TX ring DMA tag.\n");
		goto fail;
	}

	/* RX mbufs */
	error = bus_dma_tag_create(sc->dma_parent_tag,/* parent */
	    1, 0,			/* algnmnt, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MGB_DMA_RING_INFO_SIZE,	/* maxsize */
	    1,				/* nsegments */
	    MGB_DMA_RING_INFO_SIZE,	/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rx_ring_data.tag);
	if (error != 0) {
		device_printf(dev, "Couldn't create RX ring DMA tag.\n");
		goto fail;
	}

	/* RX */
	error = bus_dmamem_alloc(sc->rx_ring_data.tag,
	    (void **)&sc->rx_ring_data.ring_info,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT,
	    &sc->rx_ring_data.dmamap);
	if (error != 0) {
		device_printf(dev, "Couldn't allocate RX DMA ring.\n");
		goto fail;
	}
	error = bus_dmamap_load(sc->rx_ring_data.tag, sc->rx_ring_data.dmamap,
	    &sc->rx_ring_data.ring_info, MGB_DMA_RING_INFO_SIZE,
	    mgb_ring_dmamap_bind, &sc->rx_ring_data,
	    BUS_DMA_NOWAIT);
	if (error != 0 || sc->rx_ring_data.ring_bus_addr == 0) {
		device_printf(dev, "Couldn't load RX DMA ring.\n");
		error = ENOMEM; /* in the case that ring_bus_addr = 0 */
		goto fail;
	}
	sc->rx_ring_data.head_wb = MGB_HEAD_WB_PTR(sc->rx_ring_data.ring_info);
	sc->rx_ring_data.ring = MGB_RING_PTR(sc->rx_ring_data.ring_info);

	/* TX */
	error = bus_dmamem_alloc(sc->tx_ring_data.tag,
	     (void **)&sc->tx_ring_data.ring_info,
	     BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT,
	     &sc->tx_ring_data.dmamap);
	if (error != 0) {
		device_printf(dev, "Couldn't allocate TX DMA ring.\n");
		goto fail;
	}
	error = bus_dmamap_load(sc->tx_ring_data.tag, sc->tx_ring_data.dmamap,
	    &sc->tx_ring_data.ring_info, MGB_DMA_RING_INFO_SIZE,
	    mgb_ring_dmamap_bind, &sc->tx_ring_data,
	    BUS_DMA_NOWAIT);
	if (error != 0 || sc->tx_ring_data.ring_bus_addr == 0) {
		device_printf(dev, "Couldn't load RX DMA ring.\n");
		error = ENOMEM; /* in the case that ring_bus_addr = 0 */
		goto fail;
	}
	/* translate pointer for the ring */
	sc->tx_ring_data.head_wb = MGB_HEAD_WB_PTR(sc->tx_ring_data.ring_info);
	sc->tx_ring_data.ring = MGB_RING_PTR(sc->tx_ring_data.ring_info);
	/* TX mbufs */
	error = bus_dma_tag_create(sc->dma_parent_tag,/* parent */
	    1, 0,			/* algnmnt, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MCLBYTES * MGB_DMA_MAXSEGS,	/* maxsize */
	    MGB_DMA_MAXSEGS,		/* nsegments */
	    MCLBYTES,			/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->tx_buffer_data.tag);
	if (error != 0) {
		device_printf(dev, "Couldn't create TX ring DMA tag.\n");
		goto fail;
	}

	/* RX mbufs */
	error = bus_dma_tag_create(sc->dma_parent_tag,/* parent */
	    1, 0,			/* algnmnt, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MCLBYTES,			/* maxsize */
	    1,				/* nsegments */
	    MCLBYTES,			/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rx_buffer_data.tag);
	if (error != 0) {
		device_printf(dev, "Couldn't create RX ring DMA tag.\n");
		goto fail;
	}

	error = bus_dmamap_create(sc->rx_buffer_data.tag,
	    0, &sc->rx_buffer_data.sparemap);
	if (error != 0) {
		device_printf(dev, "Couldn't create spare RX dmamap.\n");
		goto fail;
	}

	for (i = 0; i < MGB_DMA_RING_SIZE; i++) {
		desc = &sc->rx_buffer_data.desc[i];
		desc->m = NULL;
		desc->dmamap = NULL;
		error = bus_dmamap_create(sc->rx_buffer_data.tag,
		    0, &desc->dmamap);
		if (error != 0) {
			device_printf(dev, "Could not create RX buffer dmamap\n");
			goto fail;
		}
	}

	for (i = 0; i < MGB_DMA_RING_SIZE; i++) {
		desc = &sc->tx_buffer_data.desc[i];
		desc->m = NULL;
		desc->dmamap = NULL;
		error = bus_dmamap_create(sc->tx_buffer_data.tag,
		    0, &desc->dmamap);
		if (error != 0) {
			device_printf(dev, "Could not create TX buffer dmamap\n");
			goto fail;
		}
	}
fail:
	return (error);
}

static void
mgb_dma_teardown(struct mgb_softc *sc)
{
	struct mgb_buffer_desc *desc;
	int i;

	/* XXX: May have to reset values to NULL */

	if (sc->dma_parent_tag != NULL) {
		if (sc->rx_ring_data.tag != NULL) {
			if (sc->rx_ring_data.head_wb_bus_addr != 0)
				bus_dmamap_unload(sc->rx_ring_data.tag,
				    sc->rx_ring_data.dmamap);
			if (sc->rx_ring_data.ring_info != NULL)
				bus_dmamem_free(sc->rx_ring_data.tag,
				    sc->rx_ring_data.ring_info,
				    sc->rx_ring_data.dmamap);
			bus_dma_tag_destroy(sc->rx_ring_data.tag);
		}
		if (sc->tx_ring_data.tag != NULL) {
			if (sc->tx_ring_data.tag != NULL) {
				if (sc->tx_ring_data.head_wb_bus_addr != 0)
					bus_dmamap_unload(sc->tx_ring_data.tag,
					    sc->tx_ring_data.dmamap);
				if (sc->tx_ring_data.ring_info != NULL)
					bus_dmamem_free(sc->tx_ring_data.tag,
					     sc->tx_ring_data.ring_info,
					     sc->tx_ring_data.dmamap);
				bus_dma_tag_destroy(sc->tx_ring_data.tag);
			}
		}

		if (sc->rx_buffer_data.tag != NULL) {
			if (sc->rx_buffer_data.sparemap != NULL)
				bus_dmamap_destroy(sc->rx_buffer_data.tag,
				    sc->rx_buffer_data.sparemap);
			for (i = 0; i < MGB_DMA_RING_SIZE; i++) {
				desc = &sc->rx_buffer_data.desc[i];

				if (desc->dmamap != NULL)
					bus_dmamap_destroy(sc->rx_buffer_data.tag,
					    desc->dmamap);
			}
			bus_dma_tag_destroy(sc->rx_buffer_data.tag);
		}

		if (sc->tx_buffer_data.tag != NULL) {
			for (i = 0; i < MGB_DMA_RING_SIZE; i++) {
				desc = &sc->tx_buffer_data.desc[i];

				if (desc->dmamap != NULL)
					bus_dmamap_destroy(sc->tx_buffer_data.tag,
					    desc->dmamap);
			}
			bus_dma_tag_destroy(sc->tx_buffer_data.tag);
		}
		bus_dma_tag_destroy(sc->dma_parent_tag);
	}
}

#endif /* UNUSED_MGB_FUNCTIONS_2 */

static int
mgb_dma_init(struct mgb_softc *sc)
{
	int error = 0;

	error = mgb_dma_tx_ring_init(sc);
	if (error != 0)
		goto fail;

	error = mgb_dma_rx_ring_init(sc);
	if (error != 0)
		goto fail;

fail:
	return error;
}

static int
mgb_dma_rx_ring_init(struct mgb_softc *sc)
{
	int ring_config, error = 0;
#if 0
	struct mgb_buffer_desc *desc;
	memset(sc->rx_ring_data.ring, 0, MGB_DMA_RING_LIST_SIZE);

	for (i = 0; i < MGB_DMA_RING_SIZE; i++) {
		desc = &sc->rx_buffer_data.desc[i];
		desc->m = NULL;
		desc->ring_desc = &sc->rx_ring_data.ring[i];
		if (i == 0)
			desc->prev = &sc->rx_buffer_data.desc[MGB_DMA_RING_SIZE -1];
		else
			desc->prev = &sc->rx_buffer_data.desc[i - 1];

		if (mgb_newbuf(sc, i) != 0)
			return (ENOBUFS);
	}

	bus_dmamap_sync(sc->rx_buffer_data.tag,
	    sc->rx_ring_data.dmamap,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
#endif

	mgb_dmac_control(sc, MGB_DMAC_RX_START, 0, DMAC_RESET);

	/* write ring address */
	if (sc->rx_ring_data.ring_bus_addr == 0) {
		device_printf(sc->dev, "Invalid ring bus addr.\n");
		goto fail;
	}
	CSR_WRITE_REG(sc, MGB_DMA_RX_BASE_H(0),
	    CSR_TRANSLATE_ADDR_HIGH32(sc->rx_ring_data.ring_bus_addr));
	CSR_WRITE_REG(sc, MGB_DMA_RX_BASE_L(0),
	    CSR_TRANSLATE_ADDR_LOW32(sc->rx_ring_data.ring_bus_addr));

	/* write head pointer writeback address */
	if (sc->rx_ring_data.head_wb_bus_addr == 0) {
		device_printf(sc->dev, "Invalid head wb bus addr.\n");
		goto fail;
	}
	CSR_WRITE_REG(sc, MGB_DMA_RX_BASE_H(0),
	    CSR_TRANSLATE_ADDR_HIGH32(sc->rx_ring_data.head_wb_bus_addr));
	CSR_WRITE_REG(sc, MGB_DMA_RX_BASE_L(0),
	    CSR_TRANSLATE_ADDR_LOW32(sc->rx_ring_data.head_wb_bus_addr));

	/* Enable interrupt on completion and head pointer writeback */
	CSR_WRITE_REG(sc, MGB_DMA_RX_CONFIG0(0), MGB_DMA_HEAD_WB_ENBL);

	ring_config = CSR_READ_REG(sc, MGB_DMA_RX_CONFIG1(0));
	/*  ring size */
	ring_config &= ~MGB_DMA_RING_LEN_MASK;
	ring_config |= (MGB_DMA_RING_SIZE & MGB_DMA_RING_LEN_MASK);
	/* packet padding  (PAD_2 is better for IP header alignment ...) */
	ring_config &= ~MGB_DMA_RING_PAD_MASK;
	ring_config |= (MGB_DMA_RING_PAD_0 & MGB_DMA_RING_PAD_MASK);

	CSR_WRITE_REG(sc, MGB_DMA_RX_CONFIG1(0), ring_config);

	sc->rx_ring_data.last_tail = MGB_DMA_RING_SIZE - 1;
	CSR_WRITE_REG(sc, MGB_DMA_RX_TAIL(0), sc->rx_ring_data.last_tail);
	sc->rx_ring_data.last_head = CSR_READ_REG(sc, MGB_DMA_RX_HEAD(0));

	mgb_fct_control(sc, MGB_FCT_RX_CTL, 0, FCT_RESET);
	if (error != 0) {
		device_printf(sc->dev, "Failed to reset RX FCT.\n");
		goto fail;
	}
	mgb_fct_control(sc, MGB_FCT_RX_CTL, 0, FCT_ENABLE);
	if (error != 0) {
		device_printf(sc->dev, "Failed to enable RX FCT.\n");
		goto fail;
	}
	mgb_dmac_control(sc, MGB_DMAC_RX_START, 0, DMAC_START);
	if (error != 0)
		device_printf(sc->dev, "Failed to start RX DMAC.\n");
fail:
	return (error);
}

static int
mgb_dma_tx_ring_init(struct mgb_softc *sc)
{
	int ring_config, error = 0;

	error = mgb_fct_control(sc, MGB_FCT_TX_CTL, 0, FCT_RESET);
	if (error != 0) {
		device_printf(sc->dev, "Failed to reset TX FCT.\n");
		goto fail;
	}

	error = mgb_fct_control(sc, MGB_FCT_TX_CTL, 0, FCT_ENABLE);
	if (error != 0) {
		device_printf(sc->dev, "Failed to enable TX FCT.\n");
		goto fail;
	}
	error = mgb_dmac_control(sc, MGB_DMAC_TX_START, 0, DMAC_RESET);
	if (error != 0) {
		device_printf(sc->dev, "Failed to reset TX DMAC.\n");
		goto fail;
	}
#if 0
	struct mgb_buffer_desc *desc;
	memset(sc->tx_ring_data.ring, 0, MGB_DMA_RING_LIST_SIZE);

	for (i = 0; i < MGB_DMA_RING_SIZE; i++) {
		desc = &sc->tx_buffer_data.desc[i];
		desc->m = NULL;
		desc->ring_desc = &sc->tx_ring_data.ring[i];
		if (i == 0)
			desc->prev = &sc->tx_buffer_data.desc[MGB_DMA_RING_SIZE -1];
		else
			desc->prev = &sc->tx_buffer_data.desc[i - 1];

	}
	bus_dmamap_sync(sc->tx_buffer_data.tag,
	    sc->tx_ring_data.dmamap,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
#endif

	/* write ring address */
	if (sc->tx_ring_data.ring_bus_addr == 0) {
		device_printf(sc->dev, "Invalid ring bus addr.\n");
		goto fail;
	}
	CSR_WRITE_REG(sc, MGB_DMA_TX_BASE_H(0),
	    CSR_TRANSLATE_ADDR_HIGH32(&sc->tx_ring_data.ring_bus_addr));
	CSR_WRITE_REG(sc, MGB_DMA_TX_BASE_L(0),
	    CSR_TRANSLATE_ADDR_LOW32(sc->tx_ring_data.ring_bus_addr));

	/* write ring size */
	ring_config = CSR_READ_REG(sc, MGB_DMA_TX_CONFIG1(0));
	ring_config &= ~MGB_DMA_RING_LEN_MASK;
	ring_config |= (1 & MGB_DMA_RING_LEN_MASK);
	CSR_WRITE_REG(sc, MGB_DMA_TX_CONFIG1(0), ring_config);

	/* Enable interrupt on completion and head pointer writeback */
	ring_config = (MGB_DMA_IOC_ENBL | MGB_DMA_HEAD_WB_ENBL);
	CSR_WRITE_REG(sc, MGB_DMA_TX_CONFIG0(0), ring_config);

	/* write head pointer writeback address */
	if (sc->tx_ring_data.head_wb_bus_addr == 0) {
		device_printf(sc->dev, "Invalid head wb bus addr.\n");
		goto fail;
	}
	CSR_WRITE_REG(sc, MGB_DMA_TX_BASE_H(0),
	    CSR_TRANSLATE_ADDR_HIGH32(&sc->tx_ring_data.head_wb_bus_addr));
	CSR_WRITE_REG(sc, MGB_DMA_TX_BASE_L(0),
	    CSR_TRANSLATE_ADDR_LOW32(sc->tx_ring_data.head_wb_bus_addr));

	/* TODO: assert that MGB_DMA_TX_HEAD(0) is 0 */
	sc->tx_ring_data.last_tail = 0;
	CSR_WRITE_REG(sc, MGB_DMA_TX_TAIL(0), sc->tx_ring_data.last_tail);

	error = mgb_dmac_control(sc, MGB_DMAC_TX_START, 0, DMAC_START);
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

static int
mgb_miibus_readreg(device_t dev, int phy, int reg)
{
	struct mgb_softc *sc;
	int mii_access;

	sc = device_get_softc(dev);
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

	sc = device_get_softc(dev);

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
