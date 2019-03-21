/*-
 * <LICENSE>
 *
 * $FreeBSD$
 */

/*
 *
 * <FILE_DESC>
 *
 */

#ifndef _IF_LAN743X_H_
#define _IF_LAN743X_H_

#define PCI_VENDOR_ID_MICROCHIP		0x1055
#define PCI_DEVICE_ID_LAN7430		0x7430
#define PCI_DEVICE_ID_LAN7431		0x7431


struct lan743x_softc {
	if_t	ifp;
	device_t	dev;

}

#endif /* _IF_LAN743X_H_ */
