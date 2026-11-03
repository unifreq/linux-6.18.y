// SPDX-License-Identifier: GPL-2.0+

#include <media/rc-map.h>
#include <linux/module.h>

//
// Keytable for the CMCC remote control (NEC address 0x22)
// shipped with the YZD-S18 board
//

static struct rc_map_table yzd_s18[] = {
	{ 0x22dc, KEY_POWER },
	{ 0x229c, KEY_MUTE },
	{ 0x228d, KEY_SETUP },

	{ 0x2288, KEY_HOME },
	{ 0x2282, KEY_MENU },
	{ 0x2295, KEY_BACK },

	{ 0x22ca, KEY_UP },
	{ 0x2299, KEY_LEFT },
	{ 0x22ce, KEY_ENTER },
	{ 0x22c1, KEY_RIGHT },
	{ 0x22d2, KEY_DOWN },

	{ 0x2280, KEY_VOLUMEUP },
	{ 0x2281, KEY_VOLUMEDOWN },
	{ 0x2285, KEY_PAGEUP },		/* CH+ */
	{ 0x2286, KEY_PAGEDOWN },	/* CH- */

	{ 0x2292, KEY_1 },
	{ 0x2293, KEY_2 },
	{ 0x22cc, KEY_3 },
	{ 0x228e, KEY_4 },
	{ 0x228f, KEY_5 },
	{ 0x22c8, KEY_6 },
	{ 0x228a, KEY_7 },
	{ 0x228b, KEY_8 },
	{ 0x22c4, KEY_9 },
	{ 0x22d0, KEY_NUMERIC_STAR },
	{ 0x2287, KEY_0 },
	{ 0x22da, KEY_NUMERIC_POUND },

	{ 0x22f0, KEY_BACKSPACE },	/* M */
};

static struct rc_map_list yzd_s18_map = {
	.map = {
		.scan     = yzd_s18,
		.size     = ARRAY_SIZE(yzd_s18),
		.rc_proto = RC_PROTO_NEC,
		.name     = RC_MAP_YZD_S18,
	}
};

static int __init init_rc_map_yzd_s18(void)
{
	return rc_map_register(&yzd_s18_map);
}

static void __exit exit_rc_map_yzd_s18(void)
{
	rc_map_unregister(&yzd_s18_map);
}

module_init(init_rc_map_yzd_s18)
module_exit(exit_rc_map_yzd_s18)

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("YZD-S18 CMCC remote controller keytable");
