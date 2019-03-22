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


#define CSR_READ_REG(sc, reg)		\
	bus_read_4(sc->registers, reg)

#define CSR_WRITE_REG(sc, reg, val)	\
	bus_write_4(sc->registers, reg, val)

struct lan743x_irq {
	struct resource			*res;
	driver_intr_t			*handler;
}

struct lan743x_softc {
	if_t				 ifp;
	device_t			 dev_info;

	struct resource			*registers;
	struct lan743x_irq		*irq;
}

#endif /* _IF_LAN743X_H_ */
