/*
 * Copyright (c) 2020 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * make sure overlay file is modified correctly for D/A on PA4.
 * see 
 * https://github.com/biomimetics/MRIRobotProject/tree/main/NucleoF446RE-DAC
 * /
*/

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/dac.h>
#include "stm32_std.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if (DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_channel_id) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_resolution))
#define DAC_NODE DT_PHANDLE(ZEPHYR_USER_NODE, dac)
#define DAC_CHANNEL_ID DT_PROP(ZEPHYR_USER_NODE, dac_channel_id)
#define DAC_RESOLUTION DT_PROP(ZEPHYR_USER_NODE, dac_resolution)
#else
#error "Unsupported board: see README and check /zephyr,user node"
#define DAC_NODE DT_INVALID_NODE
#define DAC_CHANNEL_ID 0
#define DAC_RESOLUTION 0
#endif

#define DAC_MAX (1U << DAC_RESOLUTION)-1

static const struct device *dac_dev = DEVICE_DT_GET(DAC_NODE);

static const struct dac_channel_cfg dac_ch_cfg = {
	.channel_id  = DAC_CHANNEL_ID,
	.resolution  = DAC_RESOLUTION
};




/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void dac_init(void);
void set_dac(int);


void dac_init(void)
{	int ret;
	if (!device_is_ready(dac_dev)) {
		printk("# DAC device %s is not ready\n", dac_dev->name);
		return;
	}

	ret = dac_channel_setup(dac_dev, &dac_ch_cfg);
	if (ret != 0) {
		printk("# Setting up of DAC channel failed with code %d\n", ret);
		return;
	}

    #ifdef DEBUG_PRINT
	printk("# End DAC init. Using PA4. DAC_MAX=%d and DAC channel %d\n",
            DAC_MAX, DAC_CHANNEL_ID );
    #endif
  
}

void set_dac(int dac_value)
{   int ret;
	if (dac_value < 0) dac_value =0; // positive only
    if (dac_value > DAC_MAX) dac_value= DAC_MAX; // clip at 12 bits
	ret = dac_write_value(dac_dev, DAC_CHANNEL_ID, dac_value);
			if (ret != 0) {
				printk("# dac_write_value() failed with code %d\n", ret);
				return;
			}  
}
