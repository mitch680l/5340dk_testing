/*
 * Copyright (c) 2018 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <hal/nrf_power.h>
#include <zephyr/sys/printk.h>

#define VREGHVOUT_ADDRESS 0x00FF8010  // UICR.VREGHVOUT address
#define VREGHOUT_DEFAULT  0xFFFFFFFF  // Default value for the register if it is unwritten.
#define VOLTAGE_3V3       0x00000005  // Value for 3.3V

static int board_nrf5340_hp_nrf5340_init(void)
{

	/* if the nrf52840dongle_nrf52840 board is powered from USB
	 * (high voltage mode), GPIO output voltage is set to 1.8 volts by
	 * default and that is not enough to turn the green and blue LEDs on.
	 * Increase GPIO voltage to 3.0 volts.
	 */
	uint32_t *vreghvout = (uint32_t *)VREGHVOUT_ADDRESS;
	
//	if (*vreghvout == VOLTAGE_3V3) {
//        	return 0;  // Already set to 3.3V
//    	}
	
//	else if (*vreghvout != 0xFFFFFFFF) {
        	// Register already has a different value
        	// Would need to erase UICR first
//        	return 0;
//    	}
	
//	else {
	if (*vreghvout == VREGHOUT_DEFAULT) {
	
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
			;
		}

		NRF_UICR_S->VREGHVOUT = VOLTAGE_3V3;

		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
			;
		}

		/* a reset is required for changes to take effect */
		NVIC_SystemReset();
	}

	return 0;
}

SYS_INIT(board_nrf5340_hp_nrf5340_init, POST_KERNEL,CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

// PRE_KERNEL_1
// PRE_KERNEL_2
// POST_KERNEL
// APPLICATION