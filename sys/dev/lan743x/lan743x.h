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

#define LAN743X_TIMEOUT			(5)

#define LAN743X_BAR			1 /* PCI Base Address */

#define LAN743X_HW_CFG			0x10 /** H/W Configuration Register **/
#define LAN743X_LITE_RESET 		0x2

#define LAN743X_MAC_CR			0x0100 /** MAC Crontrol Register **/
#define LAN743X_MAC_ADD_ENBL		0x1000 /* Automatic Duplex Detection */
#define LAN743X_MAC_ASD_ENBL		0x0800 /* Automatic Speed Detection */

#define LAN743X_PMT_CTL			0x14 /** Power Management Control Register **/
#define LAN743X_PHY_RESET		0x10
#define LAN743X_PHY_READY		0x80


#define LAN743X_MII_ACCESS		0x120
#define LAN743X_MII_DATA		0x124
#define LAN743X_MII_PHY_ADDR_MASK	0x1F
#define LAN743X_MII_PHY_ADDR_SHIFT	6
#define LAN743X_MII_REG_ADDR_MASK	0x3F
#define LAN743X_MII_REG_ADDR_SHIFT	11
#define LAN743X_MII_READ		0x0
#define LAN743X_MII_WRITE		0x2
#define LAN743X_MII_BUSY		0x1

#define LAN743X_MAC_ADDR_BASE		0x118 /** MAC address (read) register **/

#define LAN743X_STS_OK			( 0 )
#define LAN743X_STS_TIMEOUT 		(-1 )

	/* CHECK_UNTIL_TIMEOUT(status, CHECK) */
	/***
	status = 0;
	for(i = 0; i < LAN743X_TIMEOUT; i++) {
		DELAY(10);
		if(CHECK())
			break;
	}
	if (i == LAN743X_TIMEOUT)
		status = -1;
	***/
	/* END OF CHECK_UNTIL_TIMEOUT */

#define CSR_READ_REG(sc, reg)		\
	bus_read_4(sc->regs, reg)

#define CSR_WRITE_REG(sc, reg, val)	\
	bus_write_4(sc->regs, reg, val)

#define CSR_UPDATE_REG(sc, reg, val)	\
	CSR_WRITE_REG(sc, CSR_READ_REG(sc, reg) | (val))

#define CSR_READ_2_BYTES(sc, reg)	\
	bus_read_2(sc->regs, reg)

#define CSR_READ_REG_BYTES(sc, reg, dest, cnt)	\
	bus_read_region_1(sc->regs, reg, dest, cnt)

struct lan743x_irq {
	struct resource			*res;
	driver_intr_t			*handler;
}

struct lan743x_softc {
	if_t				 ifp;
	device_t			 dev;

	struct resource			*regs;
	struct lan743x_irq		*irq;
}

#endif /* _IF_LAN743X_H_ */
