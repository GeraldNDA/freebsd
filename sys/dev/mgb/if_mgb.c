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

/* Needed for PCI support */
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

/* Needed for MII support */
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include "miibus_if.h"

/* Registers and structures for MGB driver */
#include <dev/mgb/if_mgb.h>

static struct mgb_vendor_info mgb_vendor_info_array[] = {
	{ PCI_VENDOR_ID_MICROCHIP, PCI_DEVICE_ID_LAN7430, "Microchip LAN7430 PCIe Gigabit Ethernet Controller" },
	{ PCI_VENDOR_ID_MICROCHIP, PCI_DEVICE_ID_LAN7431, "Microchip LAN7431 PCIe Gigabit Ethernet Controller" },
	{ 0, 0, NULL }
};

/* PCI methods */
static device_probe_t		mgb_probe;
static device_attach_t		mgb_attach;
static device_detach_t		mgb_detach;
#if 0
static device_shutdown_t	mgb_shutdown;
static device_suspend_t		mgb_suspend;
static device_resume_t		mgb_resume;
#endif

/* MSI Interrupts support */
static int			mgb_test_bar(struct mgb_softc *);
/* Interrupt helper functions */
static void 			mgb_intr_enable(struct mgb_softc *);
static void 			mgb_intr_disable(struct mgb_softc *);
static bool 			mgb_intr_test(struct mgb_softc *);

/* MAC support */
static void			mgb_get_ethaddr(struct mgb_softc *, caddr_t);

/* MII methods */
static int			mgb_miibus_readreg(device_t, int, int);
static int			mgb_miibus_writereg(device_t, int, int, int);

/* MII MEDIA support */
static int			mgb_ifmedia_upd(struct ifnet *);
static void			mgb_ifmedia_sts(struct ifnet *, struct ifmediareq *);

/* IFNET methods */
static void			mgb_init(void *);
static int			mgb_transmit_init(if_t, struct mbuf *);
static void			mgb_qflush(if_t);
static int			mgb_ioctl(if_t, u_long, caddr_t);

/* DMA helper functions*/
static int			mgb_dma_init(device_t);
static void			mgb_dma_teardown(device_t);

/* HW reset helper functions */
static int			mgb_wait_for_bits(struct mgb_softc *, int, int, int);
static int			mgb_hw_init(device_t);
static int			mgb_hw_reset(struct mgb_softc *);
static int			mgb_mac_init(struct mgb_softc *);
static int			mgb_dmac_reset(struct mgb_softc *);
static int			mgb_phy_reset(struct mgb_softc *);

/*
 * Probe for a mgb device. This is done by checking the device list.
 * If found, the name of the device is returned.
 */
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

static int
mgb_test_bar(struct mgb_softc *sc)
{
	/* Equivalent to chip_check_id */
	/* XXX Endian */
	uint32_t id_rev = CSR_READ_REG(sc, 0) >> 16;
	if (id_rev == PCI_DEVICE_ID_LAN7430 || id_rev == PCI_DEVICE_ID_LAN7431) {
		device_printf(sc->dev, "ID CHECK PASSED with ID (0x%x)\n", id_rev);
		return 0;
	} else {
		device_printf(sc->dev, "ID CHECK FAILED with ID (0x%x)\n", id_rev);
		return ENXIO;
	}
}

static void
mgb_intr(void * arg)
{
	struct mgb_softc *sc;
	uint32_t intr_sts, intr_en;


	sc = arg;
	intr_sts = CSR_READ_REG(sc, MGB_INTR_STS);
	intr_en = CSR_READ_REG(sc, MGB_INTR_ENBL_SET);

	intr_sts &= intr_en;
	if((intr_sts & MGB_INTR_STS_ANY) == 0)
		return;
	if(intr_sts &  MGB_INTR_STS_TEST) {
		sc->isr_test_flag = true;
		CSR_WRITE_REG(sc, MGB_INTR_STS, MGB_INTR_STS_TEST);
	}

#if 0
	if(intr_sts &  MGB_INTR_STS_TX) {
		/* Do TX Stuff */
	}
	if(intr_sts &  MGB_INTR_STS_RX) {
		/* Do RX Stuff */
	}
#endif
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

#if 0
	clear status
	enable intr
	trigger intr
	wait on flag to be changed
	disable interrupts
	clear statsus
#endif
	return sc->isr_test_flag;
}

static void
mgb_intr_enable(struct mgb_softc *sc)
{
	/* Ensure interrupts disabled */
	mgb_intr_disable(sc);
#if 0
	/* Register IRQ */
#endif
	/* Enable IRQs */
	CSR_WRITE_REG(sc, MGB_INTR_ENBL_SET, MGB_INTR_STS_ANY);
}
static void
mgb_intr_disable(struct mgb_softc *sc)
{
	CSR_WRITE_REG(sc, MGB_INTR_ENBL_CLR, ~0);
	CSR_WRITE_REG(sc, MGB_INTR_STS, ~0);
}

static int
mgb_attach(device_t dev)
{
	struct mgb_softc *sc;
	uint8_t ethaddr[ETHER_ADDR_LEN];
	int error = 0, rid;
	int msic, msixc;


	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Prep mutex */
	mtx_init(&sc->mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK, MTX_DEF);
	callout_init_mtx(&sc->watchdog, &sc->mtx, 0);

	/* Allocate bus resources for using PCI bus */
	rid = PCIR_BAR(MGB_BAR);
	sc->regs = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &rid, RF_ACTIVE);
	if (unlikely(sc->regs == NULL)) {
		device_printf(dev, "Unable to allocate bus resource: registers.\n");
		error = ENXIO;
		goto fail;
	}

	/** Verify that this is the correct BAR **/
	error = mgb_test_bar(sc);
	if(unlikely(error != 0))
		goto fail;

	error = mgb_hw_init(dev);
	if (unlikely(error != 0)) {
		device_printf(dev, "MGB device init failed. (err: %d)\n", error);
		goto fail;
	}

	sc->ifp = if_alloc(IFT_ETHER);
	if(unlikely(sc->ifp == NULL)) {
		device_printf(dev, "Unable to allocate ifnet structure.\n");
		error = ENXIO;
		goto fail;
	}

	mgb_get_ethaddr(sc, (caddr_t)ethaddr);

	/* Attach MII Interface */
	int phyaddr;
	switch(pci_get_device(dev))
	{
	case PCI_DEVICE_ID_LAN7430:
		phyaddr = 1;
		break;
	case PCI_DEVICE_ID_LAN7431:
	default:
		phyaddr = MII_PHY_ANY;
		break;
	}

	error = mii_attach(dev, &sc->miibus, sc->ifp, mgb_ifmedia_upd,
	    mgb_ifmedia_sts, BMSR_DEFCAPMASK, phyaddr, MII_OFFSET_ANY, MIIF_DOPAUSE);
	if(unlikely(error != 0)) {
		device_printf(dev, "Failed to attach MII interface\n");
		goto fail;
	}


	rid = 0; /* use INTx interrupts by default */
	msic = pci_msi_count(dev);
	msixc = pci_msix_count(dev);

	/* MSIX > MSI > INTx */
	if (msixc > 0) {
		msixc = 1; /* Request only one line */
		if(pci_alloc_msix(dev, &msixc) == 0) {
			if(msixc == 1) {
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
		if(pci_alloc_msi(dev, &msic) == 0) {
			if(msic == 1) {
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

	sc->irq.res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_SHAREABLE | RF_ACTIVE);
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
	if_setcapabilities(sc->ifp, IFCAP_HWCSUM | IFCAP_VLAN_MTU | IFCAP_VLAN_HWCSUM | IFCAP_VLAN_HWTAGGING ); /* Doesn't show up on ifconfig? */
	if_setcapenable(sc->ifp, if_getcapabilities(sc->ifp));

	mgb_intr_enable(sc);
	/* Add IRQ handler */
	bus_setup_intr(sc->dev, sc->irq.res, INTR_TYPE_NET|INTR_MPSAFE, NULL, mgb_intr, sc, &sc->irq.handler);
	device_printf(sc->dev, "Interrupt test: %s\n", (mgb_intr_test(sc) ? "PASS" : "FAIL"));

	ether_ifattach(sc->ifp, ethaddr);

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
	if((if_getflags(ifp) & IFF_UP) == 0)
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
#if 0
	turn off device
	 -> turn off interrupts
	 -> clear sts/ctrl registers
	 -> clear/remove buffers
	remove watchdogs
#endif
	uses_msi = (sc->flags & MGB_INTR_FLAG_MASK) != MGB_FLAG_INTX;
	if(device_is_attached(dev)) {
		ether_ifdetach(sc->ifp);
		mgb_intr_disable(sc);
		callout_drain(&sc->watchdog);
	}

	if(sc->miibus)
		device_delete_child(dev, sc->miibus);
	bus_generic_detach(dev);

	if(sc->regs)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    PCIR_BAR(MGB_BAR), sc->regs);

	if(sc->irq.handler)
		bus_teardown_intr(dev, sc->irq.res, sc->irq.handler);
	if(sc->irq.res)
		bus_release_resource(dev, SYS_RES_IRQ,
		    uses_msi ? 1 : 0, sc->irq.res);
	if(uses_msi)
		pci_release_msi(dev);

	if(sc->ifp)
		if_free(sc->ifp);
	/* DMA free */
	mgb_dma_teardown(dev);
	mtx_destroy(&sc->mtx);
#if 0
	kdb_enter(KDB_WHY_UNSET, "Something failed in MGB so entering debugger...");
#endif
	return (0);


}

static void
mgb_get_ethaddr(struct mgb_softc *sc, caddr_t dest)
{
	CSR_READ_REG_BYTES(sc, MGB_MAC_ADDR_BASE_L, dest, 4);
	CSR_READ_REG_BYTES(sc, MGB_MAC_ADDR_BASE_H, dest + 4, 2);
}

static void
mgb_init(void *arg)
{
	struct mgb_softc *sc;
	struct mii_data *miid;

	sc = (struct mgb_softc *)arg;
	miid = device_get_softc(sc->miibus);
	/*
	 * TODO: Should separate alloc and init and
	 * do alloc related at attach!
	 *
	 * TODO: Should lock this up!
	 * (will want to run this other places so should split
	 * functionality and locking)
	 */
	/* error = */mgb_dma_init(dev);
	/* If an error occurs then stop! */

	if_setdrvflagbits(sc->ifp, IFF_DRV_RUNNING, IFF_DRV_OACTIVE);
	mii_mediachg(miid);

}

static int
mgb_transmit_init(if_t ifp, struct mbuf *m)
{
	struct mgb_softc *sc;

	sc = if_getsoftc(ifp);

#if 0
	XXX: ?????????
	Don't do anything if Running and !Active

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
#endif
	device_printf(sc->dev, "transmit_init()\n");
	return 0;
}

static void
mgb_qflush(if_t ifp)
{
	struct mgb_softc *sc;

	sc = if_getsoftc(ifp);
	device_printf(sc->dev, "qflush()\n");
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
mgb_newbuf()
{
	struct mbuf *m;
	struct mgb_buffer_desc *desc;
	bus_dma_segment_t segs[1];
	bus_dmamap_t map;
	int error, i, nsegs;

	if(bus_dmamap_load_mbuf_sg(sc->rx_buffer_data.tag, <SPAREMAP>, m, segs, &nsegs, 0) != 0) {
		m_freem(m);
		return (ENOBUFS);
	}
	KASSERT(nsegs == 1, ("%s: expected 1 DMA segment, found %d!", __func__, nsegs));

	desc = &sc->rx_buffer_data.desc[idx];

	if (desc->m != NULL) {
		bus_dmamap_sync(sc->rx_buffer_data.tag, desc->dmamap,
		    BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(sc->rx_buffer_data.tag, desc->dmamap);
	}
	map = desc->dmamap;
	desc->dmamap = <SPAREMAP>;
	<SPAREMAP> = desc->dmamap;
	bus_dmamap_sync(sc->rx_buffer_data.tag, rxd->rx_dmamap,
	    BUS_DMASYNC_PREREAD);
	desc->m = m;

	desc->ring_desc->sts = 0;
	/* XXX: Endian */
	desc->ring_desc->addr.low = CSR_TRANSLATE_ADDR_LOW32(segs[0].ds_addr);
	desc->ring_desc->addr.high = CSR_TRANSLATE_ADDR_HIGH32(segs[0].ds_addr);
	desc->ring_desc->ctl = (MGB_DESC_CTL_OWN | (segs[0].ds_len & MGB_DESC_CTL_BUFLEN_MASK));
	return 0;
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
		DELAY(10); /* > 5us delay */
		val = CSR_READ_REG(sc, reg);
		if ((val & set_bits) == set_bits &&
		    (val & clear_bits) == 0)
			return MGB_STS_OK;
	} while(i++ < MGB_TIMEOUT);

	return MGB_STS_TIMEOUT;
}

static void
mgb_ring_dmamap_bind(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
	struct mgb_ring_data *ring_data;

	if (error != 0)
		return;

	KASSERT(nsegs == 1, ("%s: expected 1 DMA segment, found %d!", __func__, nsegs));

	ring_data = (struct mgb_ring_data *)arg;
	ring_data->head_wb_bus_addr = segs[0].ds_addr;
	ring_data->ring_bus_addr = ring_data->head_wb_bus_addr + sizeof(uint32_t);
}

static void
mgb_dmamap_bind(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
	bus_addr_t *addr;

	if (error != 0)
		return;

	KASSERT(nsegs == 1, ("%s: expected 1 DMA segment, found %d!", __func__, nsegs));

	addr = (bus_addr_t *)arg;
	*addr = segs[0].ds_addr;
}

static int
mgb_dma_init(device_t dev)
{
	struct mgb_softc *sc;
	int error, i;
	struct mgb_buffer_desc *desc;


	sc = device_get_softc(dev);

	error = bus_dma_tag_create(bus_get_dma_tag(dev),/* parent */
	    1, 0,			/* algnmnt, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr (will always be PCIE so can use 64-bit) */
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
	    BUS_SPACE_MAXADDR,		/* lowaddr (will always be PCIE so can use 64-bit) */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MGB_DMA_RING_INFO_SIZE,	/* maxsize */
	    0,				/* nsegments */
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
	    BUS_SPACE_MAXADDR,		/* lowaddr (will always be PCIE so can use 64-bit) */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MGB_DMA_RING_INFO_SIZE,	/* maxsize */
	    0,				/* nsegments */
	    MGB_DMA_RING_INFO_SIZE,	/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rx_ring_data.tag);
	if (error != 0) {
		device_printf(dev, "Couldn't create RX ring DMA tag.\n");
		goto fail;
	}

	/* RX */
	error = bus_dmamem_alloc(sc->rx_ring_data.tag, (void **)&sc->rx_ring_data.ring_info, BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT, &sc->rx_ring_data.dmamap);
	error = bus_dmamap_load(sc->rx_ring_data.tag, sc->rx_ring_data.dmamap, &sc->rx_ring_data.ring_info, MGB_DMA_RING_INFO_SIZE, mgb_ring_dmamap_bind, &sc->rx_ring_data, BUS_DMA_NOWAIT);
	sc->rx_ring_data.head_wb = MGB_HEAD_WB_PTR(sc->rx_ring_data.ring_info);
	sc->rx_ring_data.ring = MGB_RING_PTR(sc->rx_ring_data.ring_info);

	/* TX */
	error = bus_dmamem_alloc(sc->tx_ring_data.tag, (void **)&sc->tx_ring_data.ring_info, BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT, &sc->tx_ring_data.dmamap);
	error = bus_dmamap_load(sc->tx_ring_data.tag, sc->tx_ring_data.dmamap, &sc->tx_ring_data.ring_info, MGB_DMA_RING_INFO_SIZE, mgb_ring_dmamap_bind, &sc->tx_ring_data, BUS_DMA_NOWAIT);
	/* translate pointer for the ring */
	sc->tx_ring_data.head_wb = MGB_HEAD_WB_PTR(sc->tx_ring_data.ring_info);
	sc->tx_ring_data.ring = MGB_RING_PTR(sc->tx_ring_data.ring_info);
	/* TX mbufs */
	error = bus_dma_tag_create(sc->dma_parent_tag,/* parent */
	    1, 0,			/* algnmnt, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr (will always be PCIE so can use 64-bit) */
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
	    BUS_SPACE_MAXADDR,		/* lowaddr (will always be PCIE so can use 64-bit) */
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

	for (i = 0; i < MGB_DMA_RING_SIZE; i++) {
		desc = &sc->rx_buffer_data.desc[i];
		desc->m = NULL;
		desc->dmamap = NULL;
		error = bus_dmamap_create(sc->rx_buffer_data.tag, 0, &desc->dmamap);
		if (error != 0) {
			device_printf(dev, "Could not RX buffer dmamap\n");
			goto fail;
		}
	}

	for (i = 0; i < MGB_DMA_RING_SIZE; i++) {
		desc = &sc->tx_buffer_data.desc[i];
		desc->m = NULL;
		desc->dmamap = NULL;
		error = bus_dmamap_create(sc->tx_buffer_data.tag, 0, &desc->dmamap);
		if (error != 0) {
			device_printf(dev, "Could not TX buffer dmamap\n");
			goto fail;
		}
	}

	mgb_dma_tx_ring_init();
	mgb_dma_rx_ring_init();
}


fail:
	return (error);
}

static void
mgb_dma_rx_ring_init(struct mgb_softc *sc)
{
	struct mgb_buffer_desc *desc;
	int ring_config, i;

	memset(sc->rx_ring_data.ring, 0, MGB_DMA_RING_LIST_SIZE);

	for (i = 0; i < MGB_DMA_RING_SIZE; i++) {
		desc = sc->rx_buffer_data.desc[i];
		desc->m = NULL;
		desc->ring_desc = &sc->rx_ring_data.ring[i];
		if (i == 0)
			desc->prev = &sc->rx_buffer_data.desc[MGB_DMA_RING_SIZE -1];
		else
			desc->prev = &sc->rx_buffer_data.desc[i - 1];

		if (mgb_newbuf(sc, i) != 0)
			return /*ENOBUFS*/;
	}

	bus_dmamap_sync(sc->rx_buffer_data.tag,
	    sc->rx_buffer_data.dmamap,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	mgb_dmac_rx_control(sc, 0, DMAC_RESET);

	/* write ring address */
	CSR_WRITE_REG(sc, MGB_DMA_RX_BASE_H(0),
	    CSR_TRANSLATE_ADDR_HIGH32(&sc->rx_ring_data.ring_bus_addr));
	CSR_WRITE_REG(sc, MGB_DMA_RX_BASE_L(0),
	    CSR_TRANSLATE_ADDR_LOW32(sc->rx_ring_data.ring_bus_addr));

	/* write head pointer writeback address */
	CSR_WRITE_REG(sc, MGB_DMA_RX_BASE_H(0),
	    CSR_TRANSLATE_ADDR_HIGH32(&sc->rx_ring_data.head_wb_bus_addr));
	CSR_WRITE_REG(sc, MGB_DMA_RX_BASE_L(0),
	    CSR_TRANSLATE_ADDR_LOW32(sc->rx_ring_data.head_wb_bus_addr));

	/* Enable interrupt on completion and head pointer writeback */
	CSR_WRITE_REG(sc, MGB_DMA_RX_CONFIG0(0), MGB_DMA_HEAD_WB_ENBL);

	ring_config = CSR_READ_REG(MGB_DMA_RX_CONFIG1(0));
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

	/* enable interrupts and so forth */
	CSR_WRITE_REG(MGB_INTR_SET, MGB_INTR_STS_RX);
	CSR_WRITE_REG(MGB_DMAC_INTR_STS, MGB_DMA_RX_INTR);
	CSR_WRITE_REG(MGB_DMAC_INTR_ENBL_SET, MGB_DMA_RX_INTR);

	mgb_fct_control(sc, MGB_FCT_RX_CTL, 0, FCT_RESET);
	mgb_fct_control(sc, MGB_FCT_RX_CTL, 0, FCT_ENABLE);
	mgb_dmac_rx_control(sc, 0, DMAC_START);
}

static void
mgb_dma_tx_ring_init(struct mgb_softc *sc)
{
	int ring_config, i;

	mgb_fct_control(sc, MGB_FCT_TX_CTL, 0, FCT_RESET);

	memset(sc->tx_ring_data.ring, 0, MGB_DMA_RING_LIST_SIZE);

	for (i = 0; i < MGB_DMA_RING_SIZE; i++) {
		desc = sc->tx_buffer_data.desc[i];
		desc->m = NULL;
		desc->ring_desc = &sc->tx_ring_data.ring[i];
		if (i == 0)
			desc->prev = &sc->tx_buffer_data.desc[MGB_DMA_RING_SIZE -1];
		else
			desc->prev = &sc->tx_buffer_data.desc[i - 1];

	}

	bus_dmamap_sync(sc->tx_buffer_data.tag,
	    sc->tx_buffer_data.dmamap,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	mgb_dmac_tx_control(sc, 0, DMAC_RESET);

	/* write ring address */
	CSR_WRITE_REG(sc, MGB_DMA_TX_BASE_H(0),
	    CSR_TRANSLATE_ADDR_HIGH32(&sc->tx_ring_data.ring_bus_addr));
	CSR_WRITE_REG(sc, MGB_DMA_TX_BASE_L(0),
	    CSR_TRANSLATE_ADDR_LOW32(sc->tx_ring_data.ring_bus_addr));

	/* write ring size */
	ring_config = CSR_READ_REG(MGB_DMA_TX_CONFIG1(0));
	ring_config &= ~MGB_DMA_RING_LEN_MASK;
	ring_config |= (MGB_DMA_RING_SIZE & MGB_DMA_RING_LEN_MASK);
	CSR_WRITE_REG(sc, MGB_DMA_TX_CONFIG1(0), ring_config);

	/* Enable interrupt on completion and head pointer writeback */
	ring_config = (MGB_DMA_IOC_ENBL | MGB_DMA_HEAD_WB_ENBL);
	CSR_WRITE_REG(sc, MGB_DMA_TX_CONFIG0(0), ring_config);

	/* write head pointer writeback address */
	CSR_WRITE_REG(sc, MGB_DMA_TX_BASE_H(0),
	    CSR_TRANSLATE_ADDR_HIGH32(&sc->tx_ring_data.head_wb_bus_addr));
	CSR_WRITE_REG(sc, MGB_DMA_TX_BASE_L(0),
	    CSR_TRANSLATE_ADDR_LOW32(sc->tx_ring_data.head_wb_bus_addr));

	/* TODO: assert that MGB_DMA_TX_HEAD(0) is 0 */
	sc->tx_ring_data.last_tail = 0;
	CSR_WRITE_REG(sc, MGB_DMA_TX_TAIL(0), sc->tx_ring_data.last_tail);

	/* enable interrupts and so forth */
	CSR_WRITE_REG(MGB_INTR_SET, MGB_INTR_STS_TX);
	CSR_WRITE_REG(MGB_DMAC_INTR_ENBL_SET, MGB_DMA_TX_INTR);

	mgb_dmac_tx_control(sc, 0, DMAC_START);
}

static void
mgb_dma_teardown(device_t dev)
{
	struct mgb_buffer_desc *desc;
	struct mgb_softc *sc;
	int i;

	/* XXX: May have to reset values to NULL */

	sc = device_get_softc(dev);
	if(sc->dma_parent_tag != NULL) {
		if(sc->rx_ring_data.tag != NULL) {
			if(sc->rx_ring_data.busaddr != 0)
				bus_dmamap_unload(sc->rx_ring_data.tag, sc->rx_ring_data.dmamap);
			if(sc->rx_ring_data.ring_info != NULL)
				bus_dmamem_free(sc->rx_ring_data.tag, sc->rx_ring_data.ring_info, sc->rx_ring_data.dmamap);
			bus_dma_tag_destroy(sc->rx_ring_data.tag);
		}
		if(sc->tx_ring_data.tag != NULL) {
			if(sc->tx_ring_data.tag != NULL) {
				if(sc->tx_ring_data.busaddr != 0)
					bus_dmamap_unload(sc->tx_ring_data.tag, sc->tx_ring_data.dmamap);
				if(sc->tx_ring_data.ring_info != NULL)
					bus_dmamem_free(sc->tx_ring_data.tag, sc->tx_ring_data.ring_info, sc->tx_ring_data.dmamap);
				bus_dma_tag_destroy(sc->tx_ring_data.tag);
			}
		}

		if (sc->rx_buffer_data.tag != NULL) {
			for (i = 0; i < MGB_DMA_RING_SIZE; i++) {
				desc = &sc->rx_buffer_data.desc[i];

				if (desc->dmamap != NULL)
					bus_dmamap_destroy(sc->rx_buffer_data.tag, desc->dmamap);
			}
			bus_dma_tag_destroy(sc->rx_buffer_data.tag);
		}

		if (sc->tx_buffer_data.tag != NULL) {
			for (i = 0; i < MGB_DMA_RING_SIZE; i++) {
				desc = &sc->tx_buffer_data.desc[i];

				if (desc->dmamap != NULL)
					bus_dmamap_destroy(sc->tx_buffer_data.tag, desc->dmamap);
			}
			bus_dma_tag_destroy(sc->tx_buffer_data.tag);
		}
		bus_dma_tag_destroy(sc->dma_parent_tag);
	}
}

static void
mgb_dmac_tx_control(struct mgb_softc *sc, int channel, enum mgb_dmac_cmd cmd)
{

	switch (cmd) {
	case DMAC_RESET:
		CSR_WRITE_REG(sc, MGB_DMAC_CMD, MGB_DMAC_TX_RESET(channel));
		mgb_wait_for_bits(sc, MGB_DMAC_CMD,MGB_DMAC_TX_RESET(channel), 0);
		break;

	case DMAC_START:
		/* NOTE: this simplifies the logic, since it will never
		 * try to start in STOP_PENDING, but it also increases work.
		 */
		mgb_dmac_tx_control(sc, channel, DMAC_STOP);
		CSR_WRITE_REG(sc, MGB_DMAC_CMD, MGB_DMAC_TX_START(channel));
		break;

	case DMAC_STOP:
		CSR_WRITE_REG(sc, MGB_DMAC_CMD, MGB_DMAC_TX_STOP(channel));
		mgb_wait_for_bits(sc, MGB_DMAC_CMD, MGB_DMAC_TX_STOP(channel), MGB_DMAC_TX_START(channel));
		break;
	}
}

static void
mgb_dmac_rx_control(struct mgb_softc *sc, int channel, enum mgb_dmac_cmd cmd)
{

	switch (cmd) {
	case DMAC_RESET:
		CSR_WRITE_REG(sc, MGB_DMAC_CMD, MGB_DMAC_RX_RESET(channel));
		mgb_wait_for_bits(sc, MGB_DMAC_CMD,MGB_DMAC_RX_RESET(channel), 0);
		break;

	case DMAC_START:
		/* NOTE: this simplifies the logic, since it will never
		 * try to start in STOP_PENDING, but it also increases work.
		 */
		mgb_dmac_rx_control(sc, channel, DMAC_STOP);
		CSR_WRITE_REG(sc, MGB_DMAC_CMD, MGB_DMAC_RX_START(channel));
		break;

	case DMAC_STOP:
		CSR_WRITE_REG(sc, MGB_DMAC_CMD, MGB_DMAC_RX_STOP(channel));
		mgb_wait_for_bits(sc, MGB_DMAC_CMD, MGB_DMAC_RX_STOP(channel), MGB_DMAC_RX_START(channel));
		break;
	}
}

static void
mgb_fct_control(struct mgb_softc *sc, int reg, int channel, enum mgb_fct_cmd cmd)
{
	switch (cmd) {
	case FCT_RESET:
		CSR_WRITE_REG(sc, reg, MGB_FCT_TX_RESET(channel));
		mgb_wait_for_bits(sc, reg, 0, MGB_FCT_TX_RESET(channel));
		break;
	case FCT_ENABLE:
		CSR_WRITE_REG(sc, reg, MGB_FCT_TX_ENBL(channel));
		break;
	case FCT_DISABLE:
		CSR_WRITE_REG(sc, reg, MGB_FCT_TX_DSBL(channel));
		mgb_wait_for_bits(sc, reg, 0, MGB_FCT_TX_ENBL(channel));
		break;
	}
}

static int
mgb_hw_init(device_t dev)
{
	struct mgb_softc *sc;
	int error = 0;

	sc = device_get_softc(dev);
	error = mgb_hw_reset(sc);
	if(unlikely(error != 0))
		goto fail;

	mgb_mac_init(sc);

	error = mgb_phy_reset(sc);
	if(unlikely(error != 0))
		goto fail;

	error = mgb_dmac_reset(sc);
	if(unlikely(error != 0))
		goto fail;

	/**ring initializations **/
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
	CSR_UPDATE_REG(
		sc,
		MGB_MAC_CR,
		MGB_MAC_ADD_ENBL | MGB_MAC_ASD_ENBL
	);
	return MGB_STS_OK;
}

static int
mgb_phy_reset(struct mgb_softc *sc)
{
	CSR_UPDATE_BYTE(
		sc,
		MGB_PMT_CTL,
		MGB_PHY_RESET
	);
	if(mgb_wait_for_bits(sc, MGB_PMT_CTL, 0, MGB_PHY_RESET) == MGB_STS_TIMEOUT)
		return MGB_STS_TIMEOUT;
	return (mgb_wait_for_bits(sc, MGB_PMT_CTL, MGB_PHY_READY, 0));
}

static int
mgb_dmac_reset(struct mgb_softc *sc)
{
	CSR_WRITE_REG(sc, MGB_DMAC_CMD, MGB_DMAC_RESET);
	return (mgb_wait_for_bits(sc, MGB_DMAC_CMD, 0, MGB_DMAC_RESET));
}

static int
mgb_miibus_readreg(device_t dev, int phy, int reg)
{
	struct mgb_softc *sc;
	int mii_access;

	sc = device_get_softc(dev);

	/* for 7430 must be 1, for 7431 must be external phy */
	if(mgb_wait_for_bits(sc, MGB_MII_ACCESS, 0, MGB_MII_BUSY) == MGB_STS_TIMEOUT)
		return EIO;
	/* XXX Endian  */
	mii_access = (phy & MGB_MII_PHY_ADDR_MASK) << MGB_MII_PHY_ADDR_SHIFT;
	mii_access |= (reg & MGB_MII_REG_ADDR_MASK) << MGB_MII_REG_ADDR_SHIFT;
	mii_access |= MGB_MII_BUSY | MGB_MII_READ;
	CSR_WRITE_REG(sc, MGB_MII_ACCESS, mii_access);
	if(mgb_wait_for_bits(sc, MGB_MII_ACCESS, 0, MGB_MII_BUSY) == MGB_STS_TIMEOUT)
		return EIO;
	return (CSR_READ_2_BYTES(sc, MGB_MII_DATA));
}

static int
mgb_miibus_writereg(device_t dev, int phy, int reg, int data)
{

	struct mgb_softc *sc;
	int mii_access;

	sc = device_get_softc(dev);

	if(mgb_wait_for_bits(sc, MGB_MII_ACCESS, 0, MGB_MII_BUSY) == MGB_STS_TIMEOUT)
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

/*********************************************************************
 *  FreeBSD Device Interface Entry Points
 *********************************************************************/

static device_method_t mgb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mgb_probe),
	DEVMETHOD(device_attach,	mgb_attach),
	DEVMETHOD(device_detach,	mgb_detach),
	/* DEVMETHOD(device_shutdown,	mgb_shutdown), */
	/* DEVMETHOD(device_suspend,	mgb_suspend),  */
	/* DEVMETHOD(device_resume,	mgb_resume),   */


	/* MII Interface */
	DEVMETHOD(miibus_readreg,	mgb_miibus_readreg),
	DEVMETHOD(miibus_writereg,	mgb_miibus_writereg),

	DEVMETHOD_END
};

static driver_t mgb_driver = {
	"mgb",
	mgb_methods,
	sizeof(struct mgb_softc)
};

devclass_t mgb_devclass;
DRIVER_MODULE(mgb, pci, mgb_driver, mgb_devclass, NULL, NULL);
#if 0
/* If MIIBUS debug stuff is in attach then order matters. Use below instead. */
DRIVER_MODULE_ORDERED(mgb, pci, mgb_driver, mgb_devclass, NULL, NULL,
    SI_ORDER_ANY);
#endif
DRIVER_MODULE(miibus, mgb, miibus_driver, miibus_devclass, NULL, NULL);

MODULE_PNP_INFO("U16:vendor;U16:device", pci, mgb, mgb_vendor_info_array,
    nitems(mgb_vendor_info_array) - 1);

MODULE_DEPEND(mgb, pci, 1, 1, 1);
MODULE_DEPEND(mgb, ether, 1, 1, 1);
MODULE_DEPEND(mgb, miibus, 1, 1, 1);

/*********************************************************************/

