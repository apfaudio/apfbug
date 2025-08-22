/*
 * Lattice ECP5 JTAG commands and device definitions
 * 
 * Originally from ecpprog project:
 * https://github.com/gregdavill/ecpprog
 * 
 * Copyright and license terms from original project apply.
 */

#include "lattice_cmds.h"

const struct device_id_pair ecp_devices[] =
{
	{"LFE5U-12"   , 0x21111043 },
	{"LFE5U-25"   , 0x41111043 },
	{"LFE5U-45"   , 0x41112043 },
	{"LFE5U-85"   , 0x41113043 },
	{"LFE5UM-25"  , 0x01111043 },
	{"LFE5UM-45"  , 0x01112043 },
	{"LFE5UM-85"  , 0x01113043 },
	{"LFE5UM5G-25", 0x81111043 },
	{"LFE5UM5G-45", 0x81112043 },
	{"LFE5UM5G-85", 0x81113043 }
};

const int ecp_device_count = ARRAY_SIZE(ecp_devices);
