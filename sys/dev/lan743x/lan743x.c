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
static int	lan743x_shutdown(device_t);
static int	lan743x_suspend(device_t);
static int	lan743x_resume(device_t);

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

/*
 * Attach to a lan743x device.
 * => 

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

