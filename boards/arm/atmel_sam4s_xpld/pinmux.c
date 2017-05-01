/* pinmux.c - Arduino SAM4S Xplained */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <device.h>
#include <init.h>
#include <kernel.h>
#include <pinmux.h>
#include <soc.h>
#include <sys_io.h>

#include "pinmux/pinmux.h"

/**
 * @brief Pinmux driver for Atmel SAM4S Xplained
 *
 * The SAM4S16 on SAM4S Xplained has 3 PIO controllers. These controllers
 * are responsible for pin muxing, input/output, pull-up, etc.
 *
 * All PIO controller pins are flatten into sequentially incrementing
 * pin numbers:
 *   Pins  0 -  31 are for PIOA
 *   Pins 32 -  63 are for PIOB
 *   Pins 64 -  95 are for PIOC
 *
 * The squashing is done by the OS code in the SoC part of the code.
 *
 * For all the pin descriptions, refer to the Atmel datasheet, and
 * the SAM4S Xplained information at the website
 * http://www.atmel.com/webdoc/sam4s16xplained/index.html.
 */

/*
 * This is the mapping from the board pins to PIO controllers.
 * This mapping is created from the Atmel SAM4S Xplained schematics.
 * Refer to the official schematics for the actual mapping,
 * as the following may not be accurate.
 */

#define N_PIOA 0
#define N_PIOB 1
#define N_PIOC 2

/*
 * This function sets the default for the following:
 * - Pin mux (peripheral A or B)
 * - Set pin as input or output
 * - Enable pull-up for pins
 *
 * At boot, all pins are outputs with pull-up enabled, and are set to be
 * peripheral A (with value 0). So only the peripherals that need to be
 * set to B (value 1) will be declared explicitly below.
 *
 * Note that all pins are set to be controlled by the PIO controllers
 * by default. For peripherals to work (e.g. UART), the PIO has to
 * be disabled for that pin (e.g. UART to take over those pins).
 */
static void __pinmux_defaults(void)
{
	uint32_t ab_select[3];	/* A/B selection   */
	uint32_t cd_select[3];	/* C/D selection   */
	uint32_t output_en[3];	/* output enabled  */
	uint32_t pull_up[3];	/* pull-up enabled */
	uint32_t pio_ctrl[3];	/* PIO enable      */

	/* Read defaults at boot, as the bootloader may have already
	 * configured some pins. The __values come from
	 */
	ab_select[N_PIOA] = __PIOA->abcdsr1;
	ab_select[N_PIOB] = __PIOB->abcdsr1;
	ab_select[N_PIOC] = __PIOC->abcdsr1;

	cd_select[N_PIOA] = __PIOA->abcdsr2;
	cd_select[N_PIOB] = __PIOB->abcdsr2;
	cd_select[N_PIOC] = __PIOC->abcdsr2;

	output_en[N_PIOA] = __PIOA->osr;
	output_en[N_PIOB] = __PIOB->osr;
	output_en[N_PIOC] = __PIOC->osr;

	pio_ctrl[N_PIOA] = __PIOA->psr;
	pio_ctrl[N_PIOB] = __PIOB->psr;
	pio_ctrl[N_PIOC] = __PIOC->psr;

	/* value 1 means pull-up disabled, so need to invert */
	pull_up[N_PIOA] = ~(__PIOA->pusr);
	pull_up[N_PIOB] = ~(__PIOB->pusr);
	pull_up[N_PIOC] = ~(__PIOC->pusr);

	/*
	 * Now modify to meet the board schematic.
	 */

	/* UART console:
	 * PB2 (RX)
	 * PB3 (TX)
	 */
	pio_ctrl[N_PIOB] &= ~(BIT(2) | BIT(3));
	ab_select[N_PIOB] &= ~(BIT(2) | BIT(3));
	cd_select[N_PIOB] &= ~(BIT(2) | BIT(3));

	/*
	 * Write modifications back to those registers
	 */

	__PIOA->abcdsr1 = ab_select[N_PIOA];
	__PIOB->abcdsr1 = ab_select[N_PIOB];
	__PIOC->abcdsr1 = ab_select[N_PIOC];

	__PIOA->abcdsr2 = cd_select[N_PIOA];
	__PIOB->abcdsr2 = cd_select[N_PIOB];
	__PIOC->abcdsr2 = cd_select[N_PIOC];

	/* set output enable */
	__PIOA->oer = output_en[N_PIOA];
	__PIOB->oer = output_en[N_PIOB];
	__PIOC->oer = output_en[N_PIOC];

	/* set output disable */
	__PIOA->odr = ~(output_en[N_PIOA]);
	__PIOB->odr = ~(output_en[N_PIOB]);
	__PIOC->odr = ~(output_en[N_PIOC]);

	/* set PIO disable */
	__PIOA->pdr = ~(pio_ctrl[N_PIOA]);
	__PIOB->pdr = ~(pio_ctrl[N_PIOB]);
	__PIOC->pdr = ~(pio_ctrl[N_PIOC]);

	/* set pull-up enable */
	__PIOA->puer = pull_up[N_PIOA];
	__PIOB->puer = pull_up[N_PIOB];
	__PIOC->puer = pull_up[N_PIOC];

	/* set pull-up disable */
	__PIOA->pudr = ~(pull_up[N_PIOA]);
	__PIOB->pudr = ~(pull_up[N_PIOB]);
	__PIOC->pudr = ~(pull_up[N_PIOC]);
}

static int pinmux_init(struct device *port)
{
	ARG_UNUSED(port);

	__pinmux_defaults();

	return 0;
}

SYS_INIT(pinmux_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
