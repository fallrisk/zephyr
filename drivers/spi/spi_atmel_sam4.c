
#include <errno.h>
#include <spi.h>
#include <soc.h>

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SPI_LEVEL
#include <logging/sys_log.h>

/* SPI Register Structure
 * Found on page 704 of data sheet.
 */
struct _spi {
	uint32_t	cr; /* 0x00 Control Register */
	uint32_t	mr; /* 0x04 Mode Register */
	uint32_t	rdr; /* 0x08 Receive Data Register */
	uint32_t	tdr; /* 0x0C Transmit Data Register */
	uint32_t	sr; /* 0x10 Status Register */
	uint32_t	ier; /* 0x14 Interrupt Enable Register */
	uint32_t	idr; /* 0x18 Interrupt Disable Register */
	uint32_t	imr; /* 0x1C Interrupt Mask Register */
	uint32_t	rev0[4]; /* 0x20-0x2C Reserved */
	uint32_t	csr0; /* 0x30 Chip Select Register 0 */
	uint32_t	csr1; /* 0x34 Chip Select Register 1 */
	uint32_t	csr2; /* 0x38 Chip Select Register 2 */
	uint32_t	csr3; /* 0x3C Chip Select Register 3 */
	uint32_t	rev1[41]; /* 0x40-0xE0 Reserved */
	uint32_t	wpmr; /* 0xE4 Write Protection Mode Register */
	uint32_t	wpsr; /* 0xE4 Write Protection Status Register */
};

/* Bit definitions to make configuration more readable. */
#define SPI_CR_SPIEN (1 << 0)
#define SPI_CR_SPIDIS (1 << 1)
#define SPI_CR_SWRST (1 << 7)

#define SPI_MR_MSTR (1 << 0)

#define SPI_SR_RDRF (1 << 0)
#define SPI_SR_TDRE (1 << 1)

/******** Definitions */

struct spi_sam4_config {
	uint32_t base_addr;
};

struct spi_sam4_data {
};

/******** Functions */
int spi_sam4_init(struct device *dev)
{
	/* Enable SPI clock in the Power Management Controller PMC
	 * The bit that corresponds to the peripheral is found in "
	 * Peripheral Identifiers" Page 50.
	 */
	__PMC->pcer0 = BIT(21);

	/* Detach pins PA12, PA13, and PA14 from the PIO controller.
	 * PA12 is MISO, PA13 is MOSI, PA14 is SPCK (SPI clock).
	 */
	__PIOA->pdr = (BIT(12)) | (BIT(13)) | (BIT(14));
	__PIOA->abcdsr1 &= (BIT(12)) | (BIT(13)) | (BIT(14));
	__PIOA->abcdsr2 &= (BIT(12)) | (BIT(13)) | (BIT(14));
	return 0;
}

/******** SPI Driver API Functions */

static int spi_sam4_configure(struct device *dev,
	struct spi_config *spi_config)
{
	const struct spi_sam4_config *config = dev->config->config_info;

	/* Use the base address to create a _spi structure. */
	volatile struct _spi *spi = (volatile struct _spi *)(config->base_addr);

	if (spi_config->config & SPI_TRANSFER_LSB) {
		/* Print a message if it fails. */
		return -EINVAL;
	}

	/* Programming the divider to 0 is forbidden. */
	if (spi_config->max_sys_freq == 0) {
		return -EINVAL;
	}

	/* Enable SPI clock in the Power Management Controller PMC
	 * The bit that corresponds to the peripheral is found in
	 * "Peripheral Identifiers" Page 50.
	 */
	__PMC->pcer0 = BIT(21);

	/* Detach pins PA12, PA13, and PA14 from the PIO controller.
	 * PA12 is MISO, PA13 is MOSI, PA14 is SPCK (SPI clock).
	 */
	__PIOA->pdr = (BIT(12)) | (BIT(13)) | (BIT(14));
	__PIOA->abcdsr1 &= (BIT(12)) | (BIT(13)) | (BIT(14));
	__PIOA->abcdsr2 &= (BIT(12)) | (BIT(13)) | (BIT(14));

	/* Reset and disable SPI. */
	spi->cr = SPI_CR_SWRST | SPI_CR_SPIDIS;

	/* Configure the SPI peripheral. */
	spi->mr = SPI_MR_MSTR;

	if (spi_config->max_sys_freq < 1 || spi_config->max_sys_freq > 255) {
		return -EINVAL;
	}

	spi->csr0 = 0x02020000
		| ((SPI_WORD_SIZE_GET(spi_config->config) - 8) << 4)
		| ((spi_config->config & SPI_MODE_CPOL) << 0)
		| (spi_config->max_sys_freq << 8);

	if (!(spi_config->config & SPI_MODE_CPHA)) {
		spi->csr0 |= 0x2;
	}

	SYS_LOG_DBG("SPI CSR0: 0x%x", spi->csr0);

	/* Enable the SPI peripheral. */
	spi->cr = SPI_CR_SPIEN | (spi_config->config & SPI_MODE_LOOP << 7);

	return 0;
}

static int spi_sam4_transceive(struct device *dev,
	const void *tx_buf, uint32_t tx_buf_len, void *rx_buf,
	uint32_t rx_buf_len)
{
	const struct spi_sam4_config *config = dev->config->config_info;
	volatile struct _spi *spi = (volatile struct _spi *)(config->base_addr);

	/* Grab the chip select register to tell what to typecast the buffers
	 * too.
	 */
	/* uint8_t transfer_size = ((spi->csr0 >> 4) & 0xf) + 8; */

	uint16_t *tx_buf_l = (uint16_t *)tx_buf;
	uint16_t *rx_buf_l = (uint16_t *)rx_buf;

	/* Send a byte. */
	int i;

	for (i = 0; i < tx_buf_len; i++) {
		while (!(spi->sr & SPI_SR_TDRE))
			;
		spi->tdr = tx_buf_l[i];
	}

	/* Wait for a byte.
	 * Fix this. If you are reading you still have to send an empty byte or
	 * something.
	 */
	if (rx_buf_len > 0) {
		while (!(spi->sr & SPI_SR_RDRF))
			;
		rx_buf_l[0] = spi->rdr;
	}

	return 0;
}

/******** Driver API */

static const struct spi_driver_api spi_sam4_driver_api = {
	.configure = spi_sam4_configure,
	.slave_select = NULL,
	.transceive = spi_sam4_transceive,
};

/******** Configurations */

/* The define "CONFIG_SPI_0" is created by Kconfig when SPI_0 is selected. */
#ifdef CONFIG_SPI_0

/* DEVICE_DECLARE(spi_sam4_0); */

static const struct spi_sam4_config spi_sam4_config_0 = {
	/* Page 705, Section 33.8.1 */
	.base_addr = 0x40008000,
};

static struct spi_sam4_data spi_sam4_data_0;

/* https://www.zephyrproject.org/doc/1.7.0/api/device.html#c.DEVICE_INIT */
DEVICE_AND_API_INIT(spi_sam4_0, CONFIG_SPI_0_NAME, spi_sam4_init,
	&spi_sam4_data_0, &spi_sam4_config_0,
	POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
	&spi_sam4_driver_api);
#endif /* CONFIG_SPI_0 */
