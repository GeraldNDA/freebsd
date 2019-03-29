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
#ifndef _IF_MGB_H_
#define _IF_MGB_H_

#define unlikely(x)                     __builtin_expect(!!(x), 0)


#define PCI_VENDOR_ID_MICROCHIP		0x1055
#define PCI_DEVICE_ID_LAN7430		0x7430
#define PCI_DEVICE_ID_LAN7431		0x7431

#define MGB_TIMEOUT			(500)

#define MGB_BAR			0 /* PCI Base Address */

#define MGB_HW_CFG			0x10 /** H/W Configuration Register **/
#define MGB_LITE_RESET 		0x2

#define MGB_MAC_CR			0x0100 /** MAC Crontrol Register **/
#define MGB_MAC_ADD_ENBL		0x1000 /* Automatic Duplex Detection */
#define MGB_MAC_ASD_ENBL		0x0800 /* Automatic Speed Detection */

#define MGB_MAC_ADDR_BASE_L		0x11C /** MAC address lower 4 bytes (read) register **/
#define MGB_MAC_ADDR_BASE_H		0x118 /** MAC address upper 2 bytes (read) register **/

#define MGB_PMT_CTL			0x14 /** Power Management Control Register **/
#define MGB_PHY_RESET		0x10
#define MGB_PHY_READY		0x80

#define MGB_DMAC_CMD		0xC0C
#define MGB_DMAC_RESET		0x80000000

#define MGB_MII_ACCESS		0x120
#define MGB_MII_DATA		0x124
#define MGB_MII_PHY_ADDR_MASK	0x1F
#define MGB_MII_PHY_ADDR_SHIFT	6
#define MGB_MII_REG_ADDR_MASK	0x3F
#define MGB_MII_REG_ADDR_SHIFT	11
#define MGB_MII_READ		0x0
#define MGB_MII_WRITE		0x2
#define MGB_MII_BUSY		0x1

#define MGB_STS_OK			( 0 )
#define MGB_STS_TIMEOUT 		(-1 )

	/* CHECK_UNTIL_TIMEOUT(status, CHECK) */
	/***
	status = 0;
	for(i = 0; i < MGB_TIMEOUT; i++) {
		DELAY(10);
		if(CHECK())
			break;
	}
	if (i == MGB_TIMEOUT)
		status = -1;
	***/
	/* END OF CHECK_UNTIL_TIMEOUT */

#define CSR_READ_BYTE(sc, reg)		\
	bus_read_1(sc->regs, reg)

#define CSR_WRITE_BYTE(sc, reg, val)	\
	bus_write_1(sc->regs, reg, val)

#define CSR_UPDATE_BYTE(sc, reg, val)	\
	CSR_WRITE_BYTE(sc, reg, CSR_READ_BYTE(sc, reg) | val)

#define CSR_READ_REG(sc, reg)		\
	bus_read_4(sc->regs, reg)

#define CSR_WRITE_REG(sc, reg, val)	\
	bus_write_4(sc->regs, reg, val)

#define CSR_UPDATE_REG(sc, reg, val)	\
	CSR_WRITE_REG(sc, reg, CSR_READ_REG(sc, reg) | (val))

#define CSR_READ_2_BYTES(sc, reg)	\
	bus_read_2(sc->regs, reg)

#define CSR_READ_REG_BYTES(sc, reg, dest, cnt)	\
	bus_read_region_1(sc->regs, reg, dest, cnt)

struct mgb_vendor_info {
	uint16_t 	vid;
	uint16_t 	did;
	char 		*name;
} mgb_vendor_info;

struct mgb_irq {
	struct resource			*res;
	driver_intr_t			*handler;
};

struct mgb_softc {
	if_t				 ifp;
	device_t			 dev;

	struct resource			*regs;
	struct mgb_irq			*irq;

	device_t			 miibus;
};

#endif /* _IF_MGB_H_ */
