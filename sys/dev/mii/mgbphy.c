/*
 * BSD LICENSE
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * driver for generic unknown PHYs
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/errno.h>
#include <sys/module.h>
#include <sys/bus.h>

#include <net/if.h>
#include <net/if_media.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include "miidevs.h"

#include "miibus_if.h"

static int mgbphy_probe(device_t);
static int mgbphy_attach(device_t);

static device_method_t mgbphy_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe,		mgbphy_probe),
	DEVMETHOD(device_attach,	mgbphy_attach),
	DEVMETHOD(device_detach,	mii_phy_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),
	DEVMETHOD_END
};

static devclass_t mgbphy_devclass;

static driver_t mgbphy_driver = {
	"mgbphy",
	mgbphy_methods,
	sizeof(struct mii_softc)
};

DRIVER_MODULE(mgbphy, miibus, mgbphy_driver, mgbphy_devclass, 0, 0);

static int	mgbphy_service(struct mii_softc *, struct mii_data *, int);

static const struct mii_phydesc mgbphys[] = {
	MII_PHY_DESC(MICROCHIP, LAN7430),
	MII_PHY_END
};

static const struct mii_phy_funcs mgbphy_funcs = {
	mgbphy_service,
	ukphy_status,
	mii_phy_reset
};

static int
mgbphy_probe(device_t dev)
{

	/*
	 * We know something is here, so always match at a low priority.
	 */
	return (mii_phy_dev_probe(dev, mgbphys, BUS_PROBE_DEFAULT));
}

static int
mgbphy_attach(device_t dev)
{
	struct mii_softc *sc;

	sc = device_get_softc(dev);

	mii_phy_dev_attach(dev, MIIF_NOMANPAUSE, &mgbphy_funcs, 1);
	mii_phy_setmedia(sc);

	return (0);
}

static int
mgbphy_service(struct mii_softc *sc, struct mii_data *mii, int cmd)
{

	switch (cmd) {
	case MII_POLLSTAT:
		break;

	case MII_MEDIACHG:
		mii_phy_setmedia(sc);
		break;

	case MII_TICK:
		if (mii_phy_tick(sc) == EJUSTRETURN)
			return (0);
		break;
	}

	/* Update the media status. */
	PHY_STATUS(sc);

	/* Callback if something changed. */
	mii_phy_update(sc, cmd);
	return (0);
}
