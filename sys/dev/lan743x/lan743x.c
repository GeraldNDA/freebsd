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

/* Needed for PCI support */
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>


#include <dev/lan743x/lan743x.h>

/*
 * List here so that you can easily do device recon
 */
static struct lan743x_vendor_info_t lan743x_vendor_info_array[] = {
	{ LAN743X_VENDORID, LAN7430_DEVICEID, "<DEVICE NICE NAME>" }, /* defined in reg.h */
	{ LAN743X_VENDORID, LAN7431_DEVICEID, "<DEVICE NICE NAME>" },
	{ 0, 0, NULL }
};

/* <FUNCTIONS> */
static int	lan743x_probe(device_t);
static int	lan743x_attach(device_t);
static int	lan743x_detach(device_t);
/* static int	lan743x_shutdown(device_t); */
/* static int	lan743x_suspend(device_t); */
/* static int	lan743x_resume(device_t); */



/*
 * Probe for a lan743x device. This is done by checking the device list.
 * If found, the name of the device is returned.
 *
 * IMPLEMENTS: (check_chip_id)
 */
static int
lan743x_probe(device_t dev)
{
	lan743x_vendor_info_t *currvendor;

	currvendor = lan743x_vendor_info_array;

	while(currvendor->lan743x_name != NULL) {
		if((pci_get_vendor(dev) == currvendor->lan743x_vid) &&
		   (pci_get_device(dev) == currvendor->lan743x_did)) {
			device_set_desc(dev, currvendor->lan743x_name);
			return (BUS_PROBE_DEFAULT);
		}
		currvendor++;
	}

	return (ENXIO); /* No such device or address */

}

static int
lan743x_test_bar(device_t dev)
{
	sc = device_get_softc(dev);
	/* Alloc bar 0 */
	/* check_chip_id() */
	/* dealloc */
	/* ret if is bar 0 */
	/* Alloc bar 1 */
	/* check_chip_id() */
	/* dealloc */
	/* ret if is bar 1 */
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
	int error = 0;

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

	rc = lan743x_hw_init(dev);
	if (unlikely(rc != 0)) {
		device_printf(dev, "LAN743X device init failed. (err: %d)\n", rc);
		goto fail;
	}

	sc->ifp = if_alloc(IFT_ETHER);
	if(unlikely(sc->ifp == NULL) {
		device_printf(dev, "Unable to allocate ifnet structure.")
	}
	lan743x_get_ethaddr(sc, (caddr_t)ethaddr);

	/* IMPLEMENTED */
	if_initname(sc->ifp, device_get_name(dev), device_get_unit(dev));
	if_setdev(sc->ifp, dev);
	if_setsoftc(sc->ifp, sc);

	/* NOT IMPLEMENTED */
	if_setflags(sc->ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setinitfn(sc->ifp, ena_init);
	if_settransmitfn(sc->ifp, ena_mq_start);
	if_setqflushfn(sc->ifp, ena_qflush);
	if_setioctlfn(sc->ifp, ena_ioctl);
	if_setgetcounterfn(sc->ifp, ena_get_counter);

	if_setsendqlen(sc->ifp, sc->tx_ring_size);
	if_setsendqready(sc->ifp);
	if_setmtu(sc->ifp, ETHERMTU);
	if_setbaudrate(sc->ifp, 0);
	if_setcapabilities(sc->ifp, 0);
	if_setcapenable(sc->ifp, 0);
	caps = ena_get_dev_offloads(feat);
	if_setcapabilitiesbit(sc->ifp, caps, 0);

	sc->ifp->if_hw_tsomax = ENA_TSO_MAXSIZE -
	    (ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN);
	sc->ifp->if_hw_tsomaxsegcount = sc->max_tx_sgl_size - 1;
	sc->ifp->if_hw_tsomaxsegsize = ENA_TSO_MAXSIZE;

	if_setifheaderlen(sc->ifp, sizeof(struct ether_vlan_header));
	if_setcapenable(sc->ifp, if_getcapabilities(sc->ifp));

	ifmedia_init(&sc->media, IFM_IMASK,
	    ena_media_change, ena_media_status);
	ifmedia_add(&sc->media, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_set(&sc->media, IFM_ETHER | IFM_AUTO);

	/* IMPLEMENTED */
	ether_ifattach(ifp, ethaddr);

fail:
	if (error)
		lan743x_detach();

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


	if(sc->miibus)
		device_delete_child(dev, sc->miibus);
	bus_generic_detach(dev);

	if(sc->regs)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    PCIR_BAR(LAN743X_BAR), sc->regs);
	if(sc->ifp)
		if_free(ifp);
	/* DMA free */
	/* mtx destroy */


	return (0);


}

static int
lan743x_get_ethaddr(lan743x_softc *sc, caddr_t dest)
{
	/* This should read the bytes of mac address one at a time
	 * so endianness shouldn't be an issue ... (each bus_read method)
	 * is defined at the device level.
	 */
	CSR_READ_REG_BYTES(sc, LAN74X_MAC_ADDR_BASE, dest, ETHER_ADDR_LEN);
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
		if(!CSR_READ_REG(sc, LAN743X_HW_CFG) & LAN743X_LITE_RESET)
			break;
	}
	if (i == LAN743X_TIMEOUT)
		return LAN743X_STS_TIMEOUT;
	/* END OF CHECK_UNTIL_TIMEOUT */
	return LAN743X_STS_OK;
}

static int
lan743x_mac_init(struct lan743x-softc *sc)
{
	/**
	 * enable automatic duplex detection and
	 * automatic speed detection
	 */
	CSR_UPDATE_REG(
		sc,
		LAN743X_MAC_CR,
		LAN743x_MAC_ADD_ENBL | LAN743X_MAC_ASD_ENBL
	);
	return LAN743X_STS_OK;
}


static int
lan743x_phy_reset(struct lan743x_softc *sc)
{
	int i;
	CSR_UPDATE_REG(
		sc,
		LAN743X_PMT_CTL,
		LAN743X_PHY_RESET
	);
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < LAN743X_TIMEOUT; i++) {
		DELAY(10); /* 2ms max */
		if(!(CSR_READ_REG(sc, LAN743X_PMT_CTL) & LAN743X_PHY_RESET))
			break;
	}
	if (i == LAN743X_TIMEOUT)
		return LAN743X_STS_TIMEOUT;
	/* END OF CHECK_UNTIL_TIMEOUT */
	/* CHECK_UNTIL_TIMEOUT */
	for(i = 0; i < LAN743X_TIMEOUT; i++) {
		if(CSR_READ_REG(sc, LAN743X_PMT_CTL) & LAN743X_PHY_READY)
			break;
	}
	if (i == LAN743X_TIMEOUT)
		return LAN743X_STS_TIMEOUT;
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
		return LAN743X_STS_TIMEOUT;
	/* END OF CHECK_UNTIL_TIMEOUT */
	return LAN743X_STS_OK;
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
}

devclass_t lan743x_devclass;
DRIVER_MODULE(lan743x, pci, lan743x_driver, lan743x_devclass, 0, 0);

MODULE_PNP_INFO("U16:vendor;U16:device", pci, lan743x, lan743x_vendor_info_array,
    nitems(lan743x_vendor_info_array) - 1);

MODULE_DEPEND(ena, pci, 1, 1, 1);
MODULE_DEPEND(ena, ether, 1, 1, 1);

/*********************************************************************/

