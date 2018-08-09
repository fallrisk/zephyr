/*
 * Copyright (c) 2018 Justin Watson
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <device.h>
#include <spi.h>
#include <soc.h>

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SPI_LEVEL
#include <logging/sys_log.h>

/* Device constant configuration parameters. */
struct spi_sam_cfg {
	Spi *regs;
	u32_t periph_id;

	/* The following pins are defined in section 40.1 of the SAM E70
	 * datasheet.
	 */
	struct soc_gpio_pin pin_spck;
	struct soc_gpio_pin pin_mosi;
	struct soc_gpio_pin pin_miso;
	struct soc_gpio_pin pin_npcs0; /* Only used if running as a slave. */
};

/* Device runtime data. */
struct spi_sam_data {
};

#define DEV_CFG(dev) \
	((const struct spi_sam_cfg *const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct spi_sam_data *const)(dev)->driver_data)

static int spi_sam_init(struct device *dev)
{
	const struct spi_sam_cfg *const cfg = DEV_CFG(dev);

	/* Enable SPI clock in PMC. */
	soc_pmc_peripheral_enable(cfg->periph_id);

	/* Connect pins to the peripheral */
	soc_gpio_configure(&cfg->pin_spck);
	soc_gpio_configure(&cfg->pin_mosi);
	soc_gpio_configure(&cfg->pin_miso);
	soc_gpio_configure(&cfg->pin_npcs0);

	return 0;
}

static int spi_sam_transceive(struct device *dev,
			       const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs)
{
	const struct spi_sam_cfg *const cfg = DEV_CFG(dev);
	Spi *const spi = cfg->regs;

	/* Copy of the operation parameter. */
	u32_t op = config->operation;

	if (op & SPI_TRANSFER_LSB) {
		SYS_LOG_ERR("SPI LSB first is not supported.\n");
		return -EINVAL;
	}

	if (config->frequency == 0) {
		SYS_LOG_ERR("SPI frequency cannot be 0.\n");
		return -EINVAL;
	}

	if (SPI_OP_MODE_GET(op) != SPI_OP_MODE_MASTER) {
		SYS_LOG_ERR("SPI slave mode not supported.\n");
		return -ENOTSUP;
	}

	if (op & SPI_LOCK_ON) {
		SYS_LOG_ERR("SPI locking is not supported.\n");
		return -ENOTSUP;
	}

	/* Reset and disable SPI. */
	spi->SPI_CR = SPI_CR_SWRST | SPI_CR_SPIDIS;

	/* Configure the SPI peripheral. */
	spi->SPI_MR = SPI_MR_MSTR;

	/* Calculate the SCBR value to go into the CSR register. The Serial
	 * Clock Bit Rate controls the SPI frequency.
	 */
	u8_t scbr = SOC_ATMEL_SAM_MCK_FREQ_HZ / config->frequency;

	spi->SPI_CSR[0] = ((SPI_WORD_SIZE_GET(op) - 8) << 4)
			  | ((op & SPI_MODE_CPOL) << 0)
			  | (scbr << 8);

	if (!(op & SPI_MODE_CPHA)) {
		spi->SPI_CSR[0] |= 0x2;
	}

	SYS_LOG_DBG("SPI CSR0 value after configuration: 0x%x",
		    spi->SPI_CSR[0]);

	/* Enable the SPI peripheral. */
	spi->SPI_CR = SPI_CR_SPIEN | (op & SPI_MODE_LOOP << 7);

	int frames_received = 0;
	u8_t buf_id;
	u8_t ele_id;

	for (buf_id = 0; buf_id < tx_bufs->count; ++buf_id) {
		uint8_t *tp8; /* Tx byte pointer. */
		uint16_t *tp16; /* Tx word pointer. */
		uint8_t *rp8; /* Rx byte pointer. */
		uint8_t *rp16; /* Rx word pointer. */
		int len;

		tp8 = tx_bufs->buffers[buf_id].buf;
		tp16 = tx_bufs->buffers[buf_id].buf;
		len = tx_bufs->buffers[buf_id].len;

		if (rx_bufs != NULL) {
			rp8 = rx_bufs->buffers[buf_id].buf;
			rp16 = rx_bufs->buffers[buf_id].buf;
		} else {
			rp8 = NULL;
			rp16 = NULL;
		}

		/* For each element in the buffer... */
		for (ele_id = 0; ele_id < len; ++ele_id) {
			while (!(spi->SPI_SR & SPI_SR_TDRE))
				;

			if (SPI_WORD_SIZE_GET(op) <= 8) {
				spi->SPI_TDR = tp8[ele_id];
			}
			 else {
				spi->SPI_TDR = tp16[ele_id];
			}

			/* Do we have a place to store the received data? */
#if 0
			if (rp8 != NULL && SPI_WORD_SIZE_GET(op) <= 8) {
				rp8[ele_id] = spi->SPI_RDR;
				++rp8;
				++frames_received;
			} else if (rp16 != NULL) {
				rp16[ele_id] = spi->SPI_RDR;
				++rp16;
				++frames_received;
			}
#endif
		}
	}

	return frames_received;
}

static const struct spi_driver_api spi_sam_driver_api = {
	.transceive = spi_sam_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = NULL,
#endif /* CONFIG_SPI_ASYNC */
	.release = NULL,
};

/* SPI0 */

#ifdef CONFIG_SPI_0_BASE_ADDRESS
static const struct spi_sam_cfg spi0_sam_config = {
	.regs = (Spi *)CONFIG_SPI_0_BASE_ADDRESS,
	.periph_id = CONFIG_SPI_0_PERIPHERAL_ID,
	.pin_spck = PIN_SPI0_SPCK,
	.pin_mosi = PIN_SPI0_MOSI,
	.pin_miso = PIN_SPI0_MISO,
	.pin_npcs0 = PIN_SPI0_NPCS0
};

static struct spi_sam_data spi0_sam_data;

DEVICE_AND_API_INIT(spi0_sam, CONFIG_SPI_0_NAME, spi_sam_init,
	&spi0_sam_data, &spi0_sam_config,
	POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
	&spi_sam_driver_api);
#endif /* CONFIG_SPI_0_BASE_ADDRESS */
