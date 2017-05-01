
#include <device.h>
#include <init.h>
#include <nanokernel.h>
#include <pinmux.h>
#include <soc.h>
#include <sys_io.h>

#define SMC_SETUP_NCS_RD_SETUP_POS (24)
#define SMC_SETUP_NRD_SETUP_POS (16)
#define SMC_SETUP_NCS_WR_SETUP_POS (8)
#define SMC_SETUP_NWE_SETUP_POS (0)

#define SMC_PULSE_NCS_RD_PULSE_POS (24)
#define SMC_PULSE_NRD_PULSE_POS (16)
#define SMC_PULSE_NCS_WR_PULSE_POS (8)
#define SMC_PULSE_NWE_PULSE_POS (0)

#define SMC_CYCLE_NRD_CYCLE_POS (16)
#define SMC_CYCLE_NWE_CYCLE_POS (0)

#define SMC_MODE_READ_MODE (1 << 0)
#define SMC_MODE_WRITE_MODE (1 << 1)
#define SMC_MODE_EXNW_MODE_POS (4)
#define SMC_MODE_TDF_CYCLE_POS (16)
#define SMC_MODE_TDF_MODE (1 << 20)
#define SMC_MODE_PMEN (1 << 24)
#define SMC_MODE_PS_POS (28)

static int external_ram_init(struct device *port)
{
	ARG_UNUSED(port);

	/*
	 * Using a local pointer because it allows us to easily change the
	 * cs in use for the SRAM.
	 */

	volatile struct __smc_cs *smc_cs = &__SMC->cs1;

	/* Remove the pins from the PIO controller. */

	__PIOC->pdr = 0xffffffff;
	__PIOA->pdr = 0x03dc0003;

	/* Set pins to their SMC peripheral function (Page 444, 26.4.1). */

	__PIOC->abcdsr1 = 0;
	__PIOC->abcdsr2 = 0;

	/* PA 0,1,18,19,20,22,23,24,25 */

	__PIOA->abcdsr1 &= ~(0x03dc0003);
	__PIOA->abcdsr2 |= 0x03dc0003;

	/* Turn on the clock for the SMC. */

	__PMC->pcer0 = BIT(10);

	/* Set the signal durations. */

	smc_cs->setup = (10 << SMC_SETUP_NCS_RD_SETUP_POS)
	                    | (10 << SMC_SETUP_NRD_SETUP_POS)
	                    | (10 << SMC_SETUP_NCS_WR_SETUP_POS)
	                    | (10 << SMC_SETUP_NWE_SETUP_POS);

	smc_cs->pulse = (10 << SMC_PULSE_NCS_RD_PULSE_POS)
	                    | (10 << SMC_PULSE_NRD_PULSE_POS)
	                    | (10 << SMC_PULSE_NCS_WR_PULSE_POS)
	                    | (10 << SMC_PULSE_NWE_PULSE_POS);

	smc_cs->cycle = (25 << SMC_CYCLE_NRD_CYCLE_POS)
	                    | (25 << SMC_CYCLE_NWE_CYCLE_POS);

	/* Set the SMC mode. */

	smc_cs->mode = SMC_MODE_TDF_MODE | (2 << SMC_MODE_TDF_CYCLE_POS);

	/*
	 * Page 38, Figure 7-1, SAM4S Product Mapping
	 *
	 * External RAM
	 * SMC CS0 0x60000000 to 0x0x61000000
	 * SMC CS1 0x61000000 to 0x0x62000000
	 * SMC CS2 0x62000000 to 0x0x63000000
	 * SMC CS3 0x64000000 to 0x0x64000000
	 */

	return 0;
}

SYS_INIT(external_ram_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
