/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2013-2015 Wind River Systems, Inc.
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

/**
 * @brief Driver for UART on Atmel SAM4 family processor.
 *
 * Note that there is only two UART controllers on the SoC.
 * Each has two wires for RX and TX, and does not have other such as
 * CTS or RTS. Also, the RX and TX are connected directly to
 * bit shifters and there is no FIFO.
 *
 * For full serial function, use the USART controller.
 *
 * (used uart_atmel_sam3.c as template)
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <misc/__assert.h>
#include <board.h>
#include <init.h>
#include <uart.h>
#include <sections.h>

/* UART registers struct from datasheet page 768. */
struct _uart {
	/* UART registers */
	uint32_t	cr;	/* 0x00 Control Register */
	uint32_t	mr;	/* 0x04 Mode Register */
	uint32_t	ier;	/* 0x08 Interrupt Enable Register */
	uint32_t	idr;	/* 0x0C Interrupt Disable Register */
	uint32_t	imr;	/* 0x10 Interrupt Mask Register */
	uint32_t	sr;	/* 0x14 Status Register */
	uint32_t	rhr;	/* 0x18 Receive Holding Register */
	uint32_t	thr;	/* 0x1C Transmit Holding Register */
	uint32_t	brgr;	/* 0x20 Baud Rate Generator Register */

	uint32_t	reserved[55];	/* 0x24 - 0xFF */

	/* Peripheral DMA Controller PDC related registers */
	uint32_t	pdc_rpr;	/* 0x100 Receive Pointer Reg */
	uint32_t	pdc_rcr;	/* 0x104 Receive Counter Reg */
	uint32_t	pdc_tpr;	/* 0x108 Transmit Pointer Reg */
	uint32_t	pdc_tcr;	/* 0x10C Transmit Counter Reg */
	uint32_t	pdc_rnpr;	/* 0x110 Receive Next Pointer */
	uint32_t	pdc_rncr;	/* 0x114 Receive Next Counter */
	uint32_t	pdc_tnpr;	/* 0x118 Transmit Next Pointer */
	uint32_t	pdc_tncr;	/* 0x11C Transmit Next Counter */
	uint32_t	pdc_ptcr;	/* 0x120 Transfer Control Reg */
	uint32_t	pdc_ptsr;	/* 0x124 Transfer Status Reg */
};

/* Device Data Structure */
struct uart_sam4_dev_data_t {
	uint32_t baud_rate;		/* Baud rate */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_t irq_cb;	/* Interrupt Callback */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

/* Convenience Macros */
#define DEV_CFG(dev) \
	((struct uart_device_config * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct uart_sam4_dev_data_t * const)(dev)->driver_data)
#define UART_STRUCT(dev) \
	((volatile struct _uart *)(DEV_CFG(dev))->base)

/* bits */
#define UART_CR_RSTRX	(1 << 2)
#define UART_CR_RSTTX	(1 << 3)
#define UART_CR_RXEN	(1 << 4)
#define UART_CR_RXDIS	(1 << 5)
#define UART_CR_TXEN	(1 << 6)
#define UART_CR_TXDIS	(1 << 7)
#define UART_CR_RSTSTA	(1 << 8)

#define UART_MR_PARTIY_MASK	(0x0E00)
#define UART_MR_PARITY_EVEN	(0 << 9)
#define UART_MR_PARITY_ODD	(1 << 9)
#define UART_MR_PARITY_SPACE	(2 << 9)
#define UART_MR_PARITY_MARK	(3 << 9)
#define UART_MR_PARITY_NO	(4 << 9)

#define UART_MR_CHMODE_MASK		(0xC000)
#define UART_MR_CHMODE_NORMAL		(0 << 14)
#define UART_MR_CHMODE_AUTOMATIC	(1 << 14)
#define UART_MR_CHMODE_LOCAL_LOOPBACK	(2 << 14)
#define UART_MR_CHMODE_REMOTE_LOOPBACK	(3 << 14)

/*
 * Since the bit positions are the same in the registers enable, disable,
 * mask, and status. These macros can be used for any of the mentioned
 * registers. Instead of macros for each register.
 */

#define UART_INT_RXRDY		(1 << 0)
#define UART_INT_TXRDY		(1 << 1)
#define UART_INT_ENDRX		(1 << 3)
#define UART_INT_ENDTX		(1 << 4)
#define UART_INT_OVRE		(1 << 5)
#define UART_INT_FRAME		(1 << 6)
#define UART_INT_PARE		(1 << 7)
#define UART_INT_TXEMPTY	(1 << 9)
#define UART_INT_TXBUFE		(1 << 11)
#define UART_INT_RXBUFF		(1 << 12)

#define UART_PDC_PTCR_RXTDIS	(1 << 1)
#define UART_PDC_PTCR_TXTDIS	(1 << 9)


static struct uart_driver_api uart_sam4_driver_api;

/*
 * @brief Set the baud rate
 *
 * This routine set the given baud rate for the UART.
 *
 * @param dev UART device struct
 * @param baudrate Baud rate
 * @param sys_clk_freq_hz System clock frequency in Hz
 *
 * @return N/A
 */
static void baudrate_set(struct device *dev,
			 uint32_t baudrate, uint32_t sys_clk_freq_hz)
{
	volatile struct _uart *uart = UART_STRUCT(dev);
	const struct uart_device_config * const dev_cfg = DEV_CFG(dev);
	struct uart_sam4_dev_data_t * const dev_data = DEV_DATA(dev);
	uint32_t divisor; /* Baudrate Divisor */

	if ((baudrate != 0) && (dev_cfg->sys_clk_freq != 0)) {
		/* Calculate baudrate divisor. */
		divisor = (dev_cfg->sys_clk_freq / baudrate) >> 4;
		divisor &= 0xFFFF;

		uart->brgr = divisor;

		dev_data->baud_rate = baudrate;
	}
}

/*
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return 0
 */
static int uart_sam4_init(struct device *dev)
{
	volatile struct _uart *uart = UART_STRUCT(dev);
	const struct uart_device_config * const dev_cfg = DEV_CFG(dev);

	/* Enable UART clock in PMC */
	__PMC->pcer0 = BIT(9);

	/* Detach pins PB2 and PB3 from PIO controller
	 * PB2 is the URXD1 and PB3 is the UTXD1.
	 */
	__PIOB->pdr = (BIT(2)) | (BIT(3));
	__PIOB->abcdsr1 &= (BIT(2) | BIT(3));
	__PIOB->abcdsr2 &= (BIT(2) | BIT(3));

	/* Disable PDC (DMA) */
	uart->pdc_ptcr = UART_PDC_PTCR_RXTDIS | UART_PDC_PTCR_TXTDIS;

	/* Reset and disable UART */
	uart->cr = UART_CR_RSTRX | UART_CR_RSTTX
		   | UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTSTA;

	/* No parity and normal mode */
	uart->mr = UART_MR_PARITY_NO | UART_MR_CHMODE_NORMAL;

	/* Set baud rate */
	baudrate_set(dev, DEV_DATA(dev)->baud_rate,
		     DEV_CFG(dev)->sys_clk_freq);

	/* Reset and disable UART */
	uart->cr = UART_CR_RSTRX | UART_CR_RSTTX
		   | UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTSTA;

	/* Enable receiver and transmitter */
	uart->cr = UART_CR_RXEN | UART_CR_TXEN;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	dev_cfg->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	return 0;
}

/*
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */
static int uart_sam4_poll_in(struct device *dev, unsigned char *c)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	if (!(uart->sr & UART_INT_RXRDY))
		return (-1);

	*c = (unsigned char)uart->rhr;

	return 0;
}

/*
 * @brief Output a character in polled mode.
 *
 * Checks if the transmitter is empty. If empty, a character is written to
 * the data register.
 *
 * @param dev UART device struct
 * @param c Character to send
 *
 * @return Sent character
 */
static unsigned char uart_sam4_poll_out(struct device *dev,
					unsigned char c)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	/* Wait for transmitter to be ready. */
	while ((uart->sr & UART_INT_TXRDY) == 0)
		;

	/* send a character */
	uart->thr = (uint32_t)c;
	return c;
}

static int uart_sam4_err_check(struct device *dev)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	int errors;

	errors = 0;

	if (uart->sr & UART_INT_OVRE) {
		errors |= UART_ERROR_OVERRUN;
	}

	if (uart->sr & UART_INT_PARE) {
		errors |= UART_ERROR_PARITY;
	}

	if (uart->sr & UART_INT_FRAME) {
		errors |= UART_ERROR_FRAMING;
	}

	return errors;
}


#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_sam4_fifo_fill(struct device *dev, const uint8_t *tx_data,
			       int size)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	/* Wait for transmitter to be ready. */
	while ((uart->sr & UART_INT_TXRDY) == 0)
		;

	uart->thr = *tx_data;

	return 1;
}

static int uart_sam4_fifo_read(struct device *dev, uint8_t *rx_data,
			       const int size)
{
	volatile struct _uart *uart = UART_STRUCT(dev);
	int bytes_read;

	bytes_read = 0;

	while (bytes_read < size) {
		if (uart->sr & UART_INT_RXRDY) {
			rx_data[bytes_read] = uart->rhr;
			bytes_read++;
		} else {
			break;
		}
	}

	return bytes_read;
}

static void uart_sam4_irq_tx_enable(struct device *dev)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	uart->ier = UART_INT_TXRDY;
}

static void uart_sam4_irq_tx_disable(struct device *dev)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	uart->idr = UART_INT_TXRDY;
}

static int uart_sam4_irq_tx_ready(struct device *dev)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	return (uart->sr & UART_INT_TXRDY);
}

static int uart_sam4_irq_tx_empty(struct device *dev)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	return (uart->sr & UART_INT_TXEMPTY);
}

static void uart_sam4_irq_rx_enable(struct device *dev)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	uart->ier = UART_INT_RXRDY;
}

static void uart_sam4_irq_rx_disable(struct device *dev)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	uart->idr = UART_INT_RXRDY;
}

static int uart_sam4_irq_rx_ready(struct device *dev)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	return (uart->sr & UART_INT_RXRDY);
}

static void uart_sam4_irq_err_enable(struct device *dev)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	uart->ier = UART_INT_OVRE | UART_INT_FRAME | UART_INT_PARE;
}

static void uart_sam4_irq_err_disable(struct device *dev)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	uart->idr = UART_INT_OVRE | UART_INT_FRAME | UART_INT_PARE;
}

static int uart_sam4_irq_is_pending(struct device *dev)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	return ((uart->sr & UART_INT_TXRDY) | (uart->sr & UART_INT_RXRDY));
}

static int uart_sam4_irq_update(struct device *dev)
{
	volatile struct _uart *uart = UART_STRUCT(dev);

	return (uart->sr & UART_INT_TXEMPTY);
}

static void uart_sam4_irq_callback_set(struct device *dev,
				       uart_irq_callback_t cb)
{
	volatile struct uart_sam4_dev_data_t *dev_data = DEV_DATA(dev);

	dev_data->irq_cb = cb;
}

static void uart_sam4_isr(void *arg)
{
	struct device *dev = arg;
	volatile struct uart_sam4_dev_data_t *dev_data = DEV_DATA(dev);

	if (dev_data->irq_cb) {
		dev_data->irq_cb(dev);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */



static struct uart_driver_api uart_sam4_driver_api = {
	.poll_in = uart_sam4_poll_in,
	.poll_out = uart_sam4_poll_out,
	.err_check = uart_sam4_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_sam4_fifo_fill,
	.fifo_read = uart_sam4_fifo_read,
	.irq_tx_enable = uart_sam4_irq_tx_enable,
	.irq_tx_disable = uart_sam4_irq_tx_disable,
	.irq_tx_ready = uart_sam4_irq_tx_ready,
	.irq_tx_empty = uart_sam4_irq_tx_empty,
	.irq_rx_enable = uart_sam4_irq_rx_enable,
	.irq_rx_disable = uart_sam4_irq_rx_disable,
	.irq_rx_ready = uart_sam4_irq_rx_ready,
	.irq_err_enable = uart_sam4_irq_err_enable,
	.irq_err_disable = uart_sam4_irq_err_disable,
	.irq_is_pending = uart_sam4_irq_is_pending,
	.irq_update = uart_sam4_irq_update,
	.irq_callback_set = uart_sam4_irq_callback_set,
#endif	/* CONFIG_UART_INTERRUPT_DRIVEN */
};



/* UART Device Definitions */


/* UART_1 */

DEVICE_DECLARE(uart_sam4_1);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_sam4_irq_config_func_1(struct device *port)
{
	/* IRQ_UART1 is defined in z/arch/arm/soc/atmel_sam4/soc.h. */

	IRQ_CONNECT(IRQ_UART1, CONFIG_UART_ATMEL_SAM4_IRQ_PRI,
		    uart_sam4_isr,
		    DEVICE_GET(uart_sam4_1), 0);
	irq_enable(IRQ_UART1);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

const static struct uart_device_config uart_sam4_dev_cfg_1 = {
	.port = 1,
	.base = (uint8_t *)UART1_ADDR,
	.sys_clk_freq = CONFIG_UART_ATMEL_SAM4_CLK_FREQ,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = uart_sam4_irq_config_func_1,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static struct uart_sam4_dev_data_t uart_sam4_dev_data_1 = {
	.baud_rate = CONFIG_UART_ATMEL_SAM4_BAUD_RATE,
};

DEVICE_AND_API_INIT(uart_sam4_1, CONFIG_UART_ATMEL_SAM4_NAME,
		    &uart_sam4_init,
		    &uart_sam4_dev_data_1, &uart_sam4_dev_cfg_1,
		    PRE_KERNEL_1, 10,
		    &uart_sam4_driver_api);
