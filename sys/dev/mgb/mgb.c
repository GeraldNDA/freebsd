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
 */

/*
 *
 * <More info about device>
 *
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

#include <dev/mgb/mgb.h>

/*
 * List here so that you can easily do device recon
 */
static struct mgb_vendor_info mgb_vendor_info_array[] = {
	{ PCI_VENDOR_ID_MICROCHIP, PCI_DEVICE_ID_LAN7430, "Microchip LAN7430 PCIe Gigabit Ethernet Controller" },
	{ PCI_VENDOR_ID_MICROCHIP, PCI_DEVICE_ID_LAN7431, "Microchip LAN7431 PCIe Gigabit Ethernet Controller" },
	{ 0, 0, NULL }
};

/* PCI methods */
static int	mgb_probe(device_t);
static int	mgb_attach(device_t);
static int	mgb_detach(device_t);
/* static int	mgb_shutdown(device_t); */
/* static int	mgb_suspend(device_t); */
/* static int	mgb_resume(device_t); */

/* MSI Interrupts support */
static int	mgb_test_bar(struct mgb_softc *);

/* MAC support */
static void	mgb_get_ethaddr(struct mgb_softc *, caddr_t);


/* MII methods */
static int	mgb_miibus_readreg(device_t, int, int);
static int	mgb_miibus_writereg(device_t, int, int, int);

/* MII MEDIA support */
static int	mgb_ifmedia_upd(struct ifnet *);
static void	mgb_ifmedia_sts(struct ifnet *, struct ifmediareq *);

/* IFNET methods */
static void	mgb_init(void *);
static int	mgb_transmit_init(if_t, struct mbuf *);
static void	mgb_qflush(if_t);
static int	mgb_ioctl(if_t, u_long, caddr_t);

/* Interrupt helper functions */
static void 	mgb_intr_enable(struct mgb_softc *);
static void 	mgb_intr_disable(struct mgb_softc *);
static int 	mgb_intr_test(struct mgb_softc *);

/* HW reset helper functions */
static int	mgb_hw_init(device_t);
static int	mgb_hw_reset(struct mgb_softc *);
static int	mgb_mac_init(struct mgb_softc *);
static int	mgb_dmac_reset(struct mgb_softc *);
static int	mgb_phy_reset(struct mgb_softc *);

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
	uint32_t id_rev = CSR_READ_REG(sc, 0x0) >> 16;
	if (id_rev == PCI_DEVICE_ID_LAN7430 || id_rev == PCI_DEVICE_ID_LAN7431) {
		device_printf(sc->dev, "ID CHECK PASSED with ID (0x%x)\n", id_rev);
		return 0;
	} else {
		device_printf(sc->dev, "ID CHECK FAILED with ID (0x%x)\n", id_rev);
		return ENXIO;
	}
}

static int isr_test_flag = 0;

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
		isr_test_flag = 1;
		CSR_WRITE_REG(sc, MGB_INTR_STS, MGB_INTR_STS_TEST);
	}



#if 0
	read status and enable registers
	ensure that the inerrupt is for this device
	check status bit for which intr handler to call
	INT_BIT_TX0 -> transmit
	INT_BIT_SW_GP -> test
	and so foth
#endif
}


static int
mgb_intr_test(struct mgb_softc *sc)
{
	int i;
	isr_test_flag = 0;

	CSR_WRITE_REG(sc, MGB_INTR_STS, MGB_INTR_STS_TEST);
	CSR_WRITE_REG(sc, MGB_INTR_ENBL_SET, MGB_INTR_STS_TEST);
	CSR_WRITE_REG(sc, MGB_INTR_SET, MGB_INTR_STS_TEST);
	for (i = 0; i < MGB_TIMEOUT; i++) {
		DELAY(10);
		if(isr_test_flag != 0)
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
	return isr_test_flag;
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
	CSR_WRITE_REG(sc, MGB_INTR_ENBL_CLR, ~0x0);
	CSR_WRITE_REG(sc, MGB_INTR_STS, ~0x0);
}
/*
 * Attach to a mgb device.
 * => Initialize many a variable
 */
static int
mgb_attach(device_t dev)
{
	struct mgb_softc *sc;
	uint8_t ethaddr[ETHER_ADDR_LEN];
	int rid, error;
	int msic, msixc;


	error = 0;
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
	sc->miibus = NULL;

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
	/* if_setqflushfn */
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
	device_printf(sc->dev, "Interrupt test: %s\n", (mgb_intr_test(sc) == 1 ? "PASS" : "FAIL"));

	ether_ifattach(sc->ifp, ethaddr);
	device_printf(dev, "DEVICE ATTACHED SUCCESSFULLY\n");

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
	mtx_destroy(&sc->mtx);
#if 0
	kdb_enter(KDB_WHY_UNSET, "Something failed in MGB so entering debugger...");
#endif
	return (0);


}

static void
mgb_get_ethaddr(struct mgb_softc *sc, caddr_t dest)
{
	/* This should read the bytes of mac address one at a time
	 * so endianness shouldn't be an issue ... (each bus_read method)
	 * is defined at the device level.
	 */
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
	device_printf(sc->dev, "Running device init ...\n");
#if 0
	Lock SC
	Interrupts, DMA queues, buffer init, load station addr etc.
	Unlock SC

	These lines cause the IF interface to start being used ...!
	if_setdrvflagbits(sc->ifp, IFF_DRV_RUNNING, IFF_DRV_OACTIVE)
#endif
	sc->ifp->if_drv_flags |= IFF_DRV_RUNNING;
	sc->ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	mii_mediachg(miid);
	if(LIST_EMPTY(&miid->mii_phys))
		device_printf(sc->dev, "NO PHYS!\n");
	device_printf(sc->dev, "Device should be running now.\n");

}

static int
mgb_transmit_init(if_t ifp, struct mbuf *m)
{
	struct mgb_softc *sc;

	sc = if_getsoftc(ifp);
	/* Run (3+ times) on mgb0 up */
	device_printf(sc->dev, "Initialized transmit queues and whatnot.\n");
	return 0;
}

static void
mgb_qflush(if_t ifp)
{
	struct mgb_softc *sc;

	sc = if_getsoftc(ifp);
	/* Run (1+ times) on mgb0 down and kldunload mgb */
	device_printf(sc->dev, "Running qflush...\n");
}

static int
mgb_ioctl(if_t ifp, u_long command, caddr_t data)
{
	int error;
	struct ifreq *ifr;
	struct mgb_softc *sc;
	struct mii_data *miid;

	sc = if_getsoftc(ifp);
	miid = device_get_softc(sc->miibus);
	if(miid == NULL)
		device_printf(sc->dev, "miid is NULL");

	ifr  = (struct ifreq *)data;

	switch (command) {
	case SIOCSIFFLAGS:
		error = 0;
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		error = 0;
		break;
	case SIOCSIFMEDIA:
		device_printf(sc->dev, "Attempted to set ifmedia\n");
		error = ifmedia_ioctl(ifp, ifr,  &miid->mii_media, command);
		break;
	case SIOCGIFMEDIA:
		device_printf(sc->dev, "Attempting to get ifmedia\n");
		struct ifmedia *ifm = &miid->mii_media;
		struct ifmediareq *ifmr = (struct ifmediareq *) ifr;
		struct ifmedia_entry *ep;
		int i;

		if (ifmr->ifm_count < 0)
			return (EINVAL);
		if (ifm->ifm_cur) {
			device_printf(sc->dev, "Converting to compat media\n");
			device_printf(sc->dev, "IFM: %s -> IFM_CUR: %s -> IFM_MEDIA: %s",
					ifm ? "good": "null",
					ifm && ifm->ifm_cur ? "good": "null",
					ifm && ifm->ifm_cur && ifm->ifm_cur->ifm_media ? "good": "null"
					);
			i = ifm->ifm_cur->ifm_media;
			device_printf(sc->dev, "GOT MEDIA TYPE\n");
			if (IFM_TYPE(i) == IFM_ETHER && IFM_SUBTYPE(i) > IFM_OTHER) {
				i &= ~(IFM_ETH_XTYPE|IFM_TMASK);
				i |= IFM_OTHER;
			}
			device_printf(sc->dev, "SUCCESS\n");
		}
		device_printf(sc->dev, "Setting values for ifmr\n");
		ifmr->ifm_active = ifmr->ifm_current = ifm->ifm_cur ? i: IFM_NONE;
		ifmr->ifm_mask = ifm->ifm_mask;
		ifmr->ifm_status = 0;
		device_printf(sc->dev, "SUCCESS\n");
		device_printf(sc->dev, "Calling ifm->ifm_status\n");
		(*ifm->ifm_status)(ifp, ifmr);
		device_printf(sc->dev, "SUCCESS\n");

		/*
		 * If there are more interfaces on the list, count
		 * them.  This allows the caller to set ifmr->ifm_count
		 * to 0 on the first call to know how much space to
		 * allocate.
		 */
		device_printf(sc->dev, "Doing other stuff with ifm_list\n");
		i = 0;
		LIST_FOREACH(ep, &ifm->ifm_list, ifm_list)
			if (i++ < ifmr->ifm_count) {
				error = copyout(&ep->ifm_media,
				    ifmr->ifm_ulist + i - 1, sizeof(int));
				if (error)
					break;
			}
		if (error == 0 && i > ifmr->ifm_count)
			error = ifmr->ifm_count ? E2BIG : 0;
		ifmr->ifm_count = i;
		device_printf(sc->dev, "SUCCESS\n");
		break;
	case SIOCSIFCAP:
		error = 0;
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
	int i;
	CSR_UPDATE_REG(sc, MGB_HW_CFG, MGB_LITE_RESET);
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < MGB_TIMEOUT; i++) {
		DELAY(10); /* > 5us delay */
		if(!(CSR_READ_REG(sc, MGB_HW_CFG) & MGB_LITE_RESET))
			break;
	}
	if (i == MGB_TIMEOUT)
		return 1;
	/* END OF CHECK_UNTIL_TIMEOUT */
	return MGB_STS_OK;
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
	int i;
	CSR_UPDATE_BYTE(
		sc,
		MGB_PMT_CTL,
		MGB_PHY_RESET
	);
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < MGB_TIMEOUT; i++) {
		DELAY(10); /* 2ms max */
		if(!(CSR_READ_BYTE(sc, MGB_PMT_CTL) & MGB_PHY_RESET))
			break;
	}
	if (i == MGB_TIMEOUT)
		return 21;
	/* END OF CHECK_UNTIL_TIMEOUT */
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < MGB_TIMEOUT; i++) {
		if(CSR_READ_BYTE(sc, MGB_PMT_CTL) & MGB_PHY_READY)
			break;
	}
	if (i == MGB_TIMEOUT)
		return 2;
	/* END OF CHECK_UNTIL_TIMEOUT */
	return MGB_STS_OK;
}

static int
mgb_dmac_reset(struct mgb_softc *sc)
{
	int i;
	CSR_WRITE_REG(sc, MGB_DMAC_CMD, MGB_DMAC_RESET);
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < MGB_TIMEOUT; i++) {
		DELAY(10);
		if(!(CSR_READ_REG(sc, MGB_DMAC_CMD) & MGB_DMAC_RESET))
			break;
	}
	if (i == MGB_TIMEOUT)
		return 3;
	/* END OF CHECK_UNTIL_TIMEOUT */
	return MGB_STS_OK;
}

static int
mgb_miibus_readreg(device_t dev, int phy, int reg)
{
	struct mgb_softc *sc;
	int i;
	int retval = -505;

	sc = device_get_softc(dev);

	/* for 7430 must be 1, for 7431 must be external phy */
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < MGB_TIMEOUT; i++) {
		DELAY(10);
		if(!(CSR_READ_REG(sc, MGB_MII_ACCESS) & MGB_MII_BUSY))
			break;
	}
	if (i == MGB_TIMEOUT) {
		retval = -4;
		goto done;
	}
	/* END OF CHECK_UNTIL_TIMEOUT */
	int mii_access = (phy << 11) & 0xF800;
	mii_access |= (reg << 6) & 0x7C0;
	mii_access |= 0x1;
	mii_access |= 0x0;
#if 0
	device_printf(dev, "Suggested mii_access (0x%x) vs mine (0x%x)\n", mii_access,
	    ((phy & MGB_MII_PHY_ADDR_MASK) << MGB_MII_PHY_ADDR_SHIFT) |
	    ((reg & MGB_MII_REG_ADDR_MASK) << MGB_MII_REG_ADDR_SHIFT) |
	    MGB_MII_READ | MGB_MII_BUSY
	    );
#endif
	CSR_WRITE_REG(sc, MGB_MII_ACCESS, mii_access);
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < MGB_TIMEOUT; i++) {
		DELAY(10);
		if(!(CSR_READ_REG(sc, MGB_MII_ACCESS) & MGB_MII_BUSY))
			break;
	}
	if (i == MGB_TIMEOUT) {
		retval = -5;
		goto done;
	}
	/* END OF CHECK_UNTIL_TIMEOUT */
#if 0
	retval = (int)(CSR_READ_2_BYTES(sc, MGB_MII_DATA));
#endif
	retval = CSR_READ_REG(sc, MGB_MII_DATA) & 0xFFFF;
done:
	device_printf(dev, "READ '0x%x' from REG#%d from PHY#%d\n", retval, reg, phy);
	return retval;
}

static int
mgb_miibus_writereg(device_t dev, int phy, int reg, int data)
{

	struct mgb_softc *sc;
	int i, error;

	sc = device_get_softc(dev);
	error = 0;

	/* for 7430 must be 1, for 7431 must be external phy */
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < MGB_TIMEOUT; i++) {
		DELAY(10);
		if(!(CSR_READ_REG(sc, MGB_MII_ACCESS) & MGB_MII_BUSY))
			break;
	}
	if (i == MGB_TIMEOUT)
		return EIO;
	/* END OF CHECK_UNTIL_TIMEOUT */
	int mii_access = (phy << 11) & 0xF800;
	mii_access |= (reg << 6) & 0x7C0;
	mii_access |= 0x1;
	mii_access |= 0x2;
#if 0
	device_printf(dev, "Suggested mii_access (0x%x) vs mine (0x%x)\n", mii_access,
	    ((phy & MGB_MII_PHY_ADDR_MASK) << MGB_MII_PHY_ADDR_SHIFT) |
	    ((reg & MGB_MII_REG_ADDR_MASK) << MGB_MII_REG_ADDR_SHIFT) |
	    MGB_MII_WRITE | MGB_MII_BUSY
	    );
#endif
	CSR_WRITE_REG(sc, MGB_MII_DATA, data);
	CSR_WRITE_REG(sc, MGB_MII_ACCESS, mii_access);
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < MGB_TIMEOUT; i++) {
		DELAY(10);
		if(!(CSR_READ_REG(sc, MGB_MII_ACCESS) & MGB_MII_BUSY))
			break;
	}
	if (i == MGB_TIMEOUT)
		return EIO;
	/* END OF CHECK_UNTIL_TIMEOUT */
	device_printf(dev, "WROTE '0x%x' from REG#%d from PHY#%d\n",data, reg, phy);
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

