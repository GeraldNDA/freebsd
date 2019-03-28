/*-
 * SPDX-License-Identifier: BSD-4-Clause
 *
 * Copyright (c) 2015-2017 Amazon.com, Inc. or its affiliates.
 * All rights reserved.
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

#include <dev/lan743x/lan743x.h>

/*
 * List here so that you can easily do device recon
 */
static struct lan743x_vendor_info lan743x_vendor_info_array[] = {
	{ PCI_VENDOR_ID_MICROCHIP, PCI_DEVICE_ID_LAN7430, "Microchip LAN7430 PCIe Gigabit Ethernet Controller" }, /* defined in reg.h */
	{ PCI_VENDOR_ID_MICROCHIP, PCI_DEVICE_ID_LAN7431, "Microchip LAN7431 PCIe Gigabit Ethernet Controller" },
	{ 0, 0, NULL }
};

/* PCI methods */
static int	lan743x_probe(device_t);
static int	lan743x_attach(device_t);
static int	lan743x_detach(device_t);
/* static int	lan743x_shutdown(device_t); */
/* static int	lan743x_suspend(device_t); */
/* static int	lan743x_resume(device_t); */

/* MSI Interrupts support */
static int	lan743x_test_bar(struct lan743x_softc *);

/* MAC support */
static void	lan743x_get_ethaddr(struct lan743x_softc *, caddr_t);


/* MII methods */
static int	lan743x_miibus_readreg(device_t, int, int);
static int	lan743x_miibus_writereg(device_t, int, int, int);

/* IFNET methods */
static void	lan743x_init(void *);
static int	lan743x_ioctl(if_t, u_long, caddr_t);

/* HW reset helper functions */
static int	lan743x_hw_init(device_t);
static int	lan743x_hw_reset(struct lan743x_softc *);
static int	lan743x_mac_init(struct lan743x_softc *);
static int	lan743x_dmac_reset(struct lan743x_softc *);
static int	lan743x_phy_reset(struct lan743x_softc *);


/*
 * Probe for a lan743x device. This is done by checking the device list.
 * If found, the name of the device is returned.
 *
 * IMPLEMENTS: (check_chip_id)
 */
static int
lan743x_probe(device_t dev)
{
	struct lan743x_vendor_info *currvendor;

	currvendor = lan743x_vendor_info_array;

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
lan743x_test_bar(struct lan743x_softc *sc)
{

	/* Alloc bar 0 */
	/* check_chip_id() */
	/* dealloc */
	/* ret if is bar 0 */
	/* Alloc bar 1 */
	/* check_chip_id() */
	/* dealloc */
	/* ret if is bar 1 */
	return (0);
}

/*
 * Attach to a lan743x device.
 * => Initialize many a variable
 */
static int
lan743x_attach(device_t dev)
{
	struct lan743x_softc *sc;
	uint8_t ethaddr[ETHER_ADDR_LEN];
	int rid, error;


	error = 0;
	sc = device_get_softc(dev);
	sc->dev = dev;
	/* Allocate bus resources for using PCI bus */
	/* pci_enable_busmaster(dev); */

	sc->dev = dev;

	rid = PCIR_BAR(LAN743X_BAR); /* combine PCI_BAR 0 and 1 */
	sc->regs = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &rid, RF_ACTIVE);
	if (unlikely(sc->regs == NULL)) {
		device_printf(dev, "Unable to allocate bus resource: registers.\n");
		goto fail;
	}
	
	/** Verify that this is the correct BAR **/
	uint32_t id_rev = CSR_READ_REG(sc, 0x0);
	if ((id_rev & 0xFFFF0000) == (0x7430 << 16) || (id_rev & 0xFFFF0000) == (0x7431 << 16)) {
		device_printf(dev, "ID CHECK PASSED with ID (0x%x)\n", id_rev);
	} else {
		device_printf(dev, "ID CHECK FAILED with ID (0x%x)\n", id_rev);
		error = ENXIO;
		goto fail;
	}

	error = lan743x_hw_init(dev);
	if (unlikely(error != 0)) {
		device_printf(dev, "LAN743X device init failed. (err: %d)\n", error);
		goto fail;
	}

	sc->ifp = if_alloc(IFT_ETHER);
	if(unlikely(sc->ifp == NULL)) {
		device_printf(dev, "Unable to allocate ifnet structure.");
	}
	lan743x_get_ethaddr(sc, (caddr_t)ethaddr);

	if_initname(sc->ifp, device_get_name(dev), device_get_unit(dev));
	if_setdev(sc->ifp, dev);
	if_setsoftc(sc->ifp, sc);

	if_setflags(sc->ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setinitfn(sc->ifp, lan743x_init);
	/* if_settransmitfn(sc->ifp, lan743x_mq_start); */
	/* if_setqflushfn(sc->ifp, lan743x_qflush); */
	if_setioctlfn(sc->ifp, lan743x_ioctl);
	/* if_setgetcounterfn(sc->ifp, ena_get_counter); */

	/* if_setsendqlen(sc->ifp, sc->tx_ring_size); */
	/* if_setsendqready(sc->ifp); */
	/* if_setmtu(sc->ifp, ETHERMTU); */
	if_setbaudrate(sc->ifp, 0);
	if_setcapabilities(sc->ifp, 0); /* ? */
	if_setcapenable(sc->ifp, 0);


	/* SHOULDN'T NEED BECAUSE MII IS USED
	 *
	ifmedia_init(&sc->media, IFM_IMASK,
	    lan743x_media_change, lan743x_media_status);
	ifmedia_add(&sc->media, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_set(&sc->media, IFM_ETHER | IFM_AUTO);
	*/

	ether_ifattach(sc->ifp, ethaddr);
	device_printf(dev, "DEVICE ATTACHED SUCCESSFULLY\n");

fail:
	if (error)
		lan743x_detach(dev);

	return (error);
}

static int
lan743x_detach(device_t dev)
{
	struct lan743x_softc *sc;

	sc = device_get_softc(dev);
	/* detach ethernet i/f */
	/* turn off device */
	/* -> turn off interrupts */
	/* -> clear sts/ctrl registers */
	/* -> clear/remove buffers */
	/* remove watchdogs */

#if 0
	if(sc->miibus)
		device_delete_child(dev, sc->miibus);
#endif
	bus_generic_detach(dev);

	if(sc->regs)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    PCIR_BAR(LAN743X_BAR), sc->regs);
	if(sc->ifp)
		if_free(sc->ifp);
	/* DMA free */
	/* mtx destroy */
#if 0
	kdb_enter(KDB_WHY_UNSET, "Something failed in LAN743X so entering debugger...");
#endif
	return (0);


}

static void
lan743x_get_ethaddr(struct lan743x_softc *sc, caddr_t dest)
{
	/* This should read the bytes of mac address one at a time
	 * so endianness shouldn't be an issue ... (each bus_read method)
	 * is defined at the device level.
	 */
	CSR_READ_REG_BYTES(sc, LAN743X_MAC_ADDR_BASE, dest, ETHER_ADDR_LEN);
}

static void
lan743x_init(void *arg)
{
	struct lan743x_softc *sc;

	sc = (struct lan743x_softc *)arg;
	/* Lock SC */
	/* Interrupts, DMA queues, buffer init, load station addr etc. */
	/* Unlock SC */
}

static int
lan743x_ioctl(if_t ifp, u_long command, caddr_t data)
{
	int error;
	/* get softc from ifp */
	/* get ifr from data */
	/* switch on `command` */
	error = ether_ioctl(ifp, command, data);
	return (error);
}

static int
lan743x_hw_init(device_t dev)
{
	struct lan743x_softc *sc;
	int error = 0;

	sc = device_get_softc(dev);
	error = lan743x_hw_reset(sc);
	if(unlikely(error != 0))
		goto fail;

	lan743x_mac_init(sc);

	error = lan743x_phy_reset(sc);
	if(unlikely(error != 0))
		goto fail;

	error = lan743x_dmac_reset(sc);
	if(unlikely(error != 0))
		goto fail;

	/**ring initializations **/
fail:
	return error;
}

static int
lan743x_hw_reset(struct lan743x_softc *sc)
{
	int i;
	CSR_UPDATE_REG(sc, LAN743X_HW_CFG, LAN743X_LITE_RESET);
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < LAN743X_TIMEOUT; i++) {
		DELAY(10); /* > 5us delay */
		if(!(CSR_READ_REG(sc, LAN743X_HW_CFG) & LAN743X_LITE_RESET))
			break;
	}
	if (i == LAN743X_TIMEOUT)
		return 1;
	/* END OF CHECK_UNTIL_TIMEOUT */
	return LAN743X_STS_OK;
}

static int
lan743x_mac_init(struct lan743x_softc *sc)
{
	/**
	 * enable automatic duplex detection and
	 * automatic speed detection
	 */
	CSR_UPDATE_REG(
		sc,
		LAN743X_MAC_CR,
		LAN743X_MAC_ADD_ENBL | LAN743X_MAC_ASD_ENBL
	);
	return LAN743X_STS_OK;
}


static int
lan743x_phy_reset(struct lan743x_softc *sc)
{
	int i;
	CSR_UPDATE_BYTE(
		sc,
		LAN743X_PMT_CTL,
		LAN743X_PHY_RESET
	);
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < LAN743X_TIMEOUT; i++) {
		DELAY(10); /* 2ms max */
		if(!(CSR_READ_BYTE(sc, LAN743X_PMT_CTL) & LAN743X_PHY_RESET))
			break;
	}
	if (i == LAN743X_TIMEOUT)
		return 21;
	/* END OF CHECK_UNTIL_TIMEOUT */
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < LAN743X_TIMEOUT; i++) {
		if(CSR_READ_BYTE(sc, LAN743X_PMT_CTL) & LAN743X_PHY_READY)
			break;
	}
	if (i == LAN743X_TIMEOUT)
		return 2;
	/* END OF CHECK_UNTIL_TIMEOUT */
	return LAN743X_STS_OK;
}

static int
lan743x_dmac_reset(struct lan743x_softc *sc)
{
	int i;
	CSR_WRITE_REG(sc, LAN743X_DMAC_CMD, LAN743X_DMAC_RESET);
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < LAN743X_TIMEOUT; i++) {
		DELAY(10);
		if(!(CSR_READ_REG(sc, LAN743X_DMAC_CMD) & LAN743X_DMAC_RESET))
			break;
	}
	if (i == LAN743X_TIMEOUT)
		return 3;
	/* END OF CHECK_UNTIL_TIMEOUT */
	return LAN743X_STS_OK;
}

static int
lan743x_miibus_readreg(device_t dev, int phy, int reg)
{
	struct lan743x_softc *sc;
	int i;

	sc = device_get_softc(dev);

	/* for 7430 must be 1, for 7431 must be external phy */
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < LAN743X_TIMEOUT; i++) {
		DELAY(10);
		if(!(CSR_READ_REG(sc, LAN743X_MII_ACCESS) & LAN743X_MII_BUSY))
			break;
	}
	if (i == LAN743X_TIMEOUT)
		return 4;
	/* END OF CHECK_UNTIL_TIMEOUT */
	CSR_WRITE_REG(sc, LAN743X_MII_ACCESS,
	    ((phy & LAN743X_MII_PHY_ADDR_MASK) << LAN743X_MII_PHY_ADDR_SHIFT) |
	    ((reg & LAN743X_MII_REG_ADDR_MASK) << LAN743X_MII_REG_ADDR_SHIFT) |
	    LAN743X_MII_READ | LAN743X_MII_BUSY
	);
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < LAN743X_TIMEOUT; i++) {
		DELAY(10);
		if(!(CSR_READ_REG(sc, LAN743X_MII_ACCESS) & LAN743X_MII_BUSY))
			break;
	}
	if (i == LAN743X_TIMEOUT)
		return 5;
	/* END OF CHECK_UNTIL_TIMEOUT */
	return (int)(CSR_READ_2_BYTES(sc, LAN743X_MII_DATA));
}

static int
lan743x_miibus_writereg(device_t dev, int phy, int reg, int data)
{

	struct lan743x_softc *sc;
	int i, error;

	sc = device_get_softc(dev);
	error = 0;

	/* for 7430 must be 1, for 7431 must be external phy */
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < LAN743X_TIMEOUT; i++) {
		DELAY(10);
		if(!(CSR_READ_REG(sc, LAN743X_MII_ACCESS) & LAN743X_MII_BUSY))
			break;
	}
	if (i == LAN743X_TIMEOUT)
		return EIO;
	/* END OF CHECK_UNTIL_TIMEOUT */
	CSR_WRITE_REG(sc, LAN743X_MII_DATA, data);
	CSR_WRITE_REG(sc, LAN743X_MII_ACCESS,
	    ((phy & LAN743X_MII_PHY_ADDR_MASK) << LAN743X_MII_PHY_ADDR_SHIFT) |
	    ((reg & LAN743X_MII_REG_ADDR_MASK) << LAN743X_MII_REG_ADDR_SHIFT) |
	    LAN743X_MII_WRITE | LAN743X_MII_BUSY
	);
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < LAN743X_TIMEOUT; i++) {
		DELAY(10);
		if(!(CSR_READ_REG(sc, LAN743X_MII_ACCESS) & LAN743X_MII_BUSY))
			break;
	}
	if (i == LAN743X_TIMEOUT)
		return EIO;
	/* END OF CHECK_UNTIL_TIMEOUT */
	return 0;
}



/*********************************************************************
 *  FreeBSD Device Interface Entry Points
 *********************************************************************/

static device_method_t lan743x_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		lan743x_probe),
	DEVMETHOD(device_attach,	lan743x_attach),
	DEVMETHOD(device_detach,	lan743x_detach),
	/* DEVMETHOD(device_shutdown,	lan743x_shutdown), */
	/* DEVMETHOD(device_suspend,	lan743x_suspend),  */
	/* DEVMETHOD(device_resume,	lan743x_resume),   */


	/* MII Interface */
	DEVMETHOD(miibus_readreg,	lan743x_miibus_readreg),
	DEVMETHOD(miibus_writereg,	lan743x_miibus_writereg),

	DEVMETHOD_END
};

static driver_t lan743x_driver = {
	"lan743x",
	lan743x_methods,
	sizeof(struct lan743x_softc)
};

devclass_t lan743x_devclass;
DRIVER_MODULE(lan743x, pci, lan743x_driver, lan743x_devclass, 0, 0);

MODULE_PNP_INFO("U16:vendor;U16:device", pci, lan743x, lan743x_vendor_info_array,
    nitems(lan743x_vendor_info_array) - 1);

MODULE_DEPEND(lan743x, pci, 1, 1, 1);
MODULE_DEPEND(lan743x, ether, 1, 1, 1);
MODULE_DEPEND(lan743x, miibus, 1, 1, 1);

/*********************************************************************/

