// SPDX-License-Identifier: GPL-2.0+
/**
 * Driver for AC200 Ethernet PHY
 *
 * Copyright (c) 2019 Jernej Skrabec <jernej.skrabec@gmail.com>
 */

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>

#define AC200_EPHY_ID			0x00441400
#define AC200_EPHY_ID_MASK		0x0ffffff0

static int ac200_ephy_config_init(struct phy_device *phydev)
{
	phy_write(phydev, 0x1f, 0x0100);	/* Switch to Page 1 */
	phy_write(phydev, 0x12, 0x4824);	/* Disable APS */

	phy_write(phydev, 0x1f, 0x0200);	/* Switch to Page 2 */
	phy_write(phydev, 0x18, 0x0000);	/* PHYAFE TRX optimization */

	phy_write(phydev, 0x1f, 0x0600);	/* Switch to Page 6 */
	phy_write(phydev, 0x14, 0x708f);	/* PHYAFE TX optimization */
	phy_write(phydev, 0x13, 0xF000);	/* PHYAFE RX optimization */
	phy_write(phydev, 0x15, 0x1530);

	phy_write(phydev, 0x1f, 0x0800);	/* Switch to Page 8 */
	phy_write(phydev, 0x18, 0x00bc);	/* PHYAFE TRX optimization */

	phy_write(phydev, 0x1f, 0x0100);	/* switch to page 1 */
	phy_clear_bits(phydev, 0x17, BIT(3));	/* disable intelligent EEE */

	/* disable 802.3az EEE */
	phy_write(phydev, 0x1f, 0x0200);	/* switch to page 2 */
	phy_write(phydev, 0x18, 0x0000);
	phy_write(phydev, 0x1f, 0x0000);	/* switch to page 0 */
	phy_clear_bits_mmd(phydev, 0x7, 0x3c, BIT(1));

	/* FIXME: This is probably H6 specific */
	phy_set_bits(phydev, 0x13, BIT(12));

	return 0;
}

static int ac200_ephy_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct clk *clk;

	clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "Failed to request clock\n");

	return 0;
}

/*
 * The AC200 and AC300 EPHYs report the same PHY ID (0x00441400). Make the
 * AC200 driver decline a node that declares itself an AC300, so that the
 * AC300 driver (which needs its own power-up/calibration sequence) gets to
 * bind instead of losing the race on registration order.
 */
static int ac200_ephy_match_phy_device(struct phy_device *phydev,
				       const struct phy_driver *phydrv)
{
	if (phydev->mdio.dev.of_node &&
	    of_device_is_compatible(phydev->mdio.dev.of_node,
				     "allwinner,sun50i-h618-ac300-ephy"))
		return 0;

	return phy_id_compare(phydev->phy_id, phydrv->phy_id, phydrv->phy_id_mask);
}

static struct phy_driver ac200_ephy_driver[] = {
	{
		.phy_id		= AC200_EPHY_ID,
		.phy_id_mask	= AC200_EPHY_ID_MASK,
		.name		= "Allwinner AC200 EPHY",
		.match_phy_device = ac200_ephy_match_phy_device,
		.soft_reset	= genphy_soft_reset,
		.config_init	= ac200_ephy_config_init,
		.probe		= ac200_ephy_probe,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	}
};
module_phy_driver(ac200_ephy_driver);

MODULE_AUTHOR("Jernej Skrabec <jernej.skrabec@gmail.com>");
MODULE_DESCRIPTION("AC200 Ethernet PHY driver");
MODULE_LICENSE("GPL");

static const struct mdio_device_id __maybe_unused ac200_ephy_phy_tbl[] = {
	{ AC200_EPHY_ID, AC200_EPHY_ID_MASK },
	{ }
};
MODULE_DEVICE_TABLE(mdio, ac200_ephy_phy_tbl);
