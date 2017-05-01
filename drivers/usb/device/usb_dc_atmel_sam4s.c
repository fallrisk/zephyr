/* usb_dc_atmel_sam4s.c - USB Atmel SAM4S device controller driver */

/*
 * Copyright (c) 2017 Justin Watson
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Atmel SAM4S device controller driver using the USB Device Port
 * UDP peripheral.
 *
 * USB Atmel SAM4S  device controller driver. The driver implements the low
 * level control routines to deal directly with the hardware. On the SAM4S
 * the peripheral is called the USB Device Port UDP.
 *
 * Refer to https://www.zephyrproject.org/doc/subsystems/usb/usb.html
 * This file is the "USB device controller drivers" driver.
 */

#include <soc.h>
#include <string.h>
#include <stdio.h>
#include <misc/byteorder.h>
#include <usb/usb_dc.h>
#include <board.h>

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_USB_SAM4S_LEVEL
#include <logging/sys_log.h>

#define UDP_VBUS_PIN (9) /* PC9 */

#define UDP_FADDR_FEN (1 << 8)

#define UDP_GLB_STAT_FADDEN (1 << 0)
#define UDP_GLB_STAT_CONFIG (1 << 1)

#define UDP_IER_WAKEUP (1 << 13)
#define UDP_IER_SOFINT (1 << 11)
#define UDP_IER_EXTRSM (1 << 10)
#define UDP_IER_RXRSM (1 << 9)
#define UDP_IER_RXSUSP (1 << 8)
#define UDP_IER_EP7INT (1 << 7)
#define UDP_IER_EP6INT (1 << 6)
#define UDP_IER_EP5INT (1 << 5)
#define UDP_IER_EP4INT (1 << 4)
#define UDP_IER_EP3INT (1 << 3)
#define UDP_IER_EP2INT (1 << 2)
#define UDP_IER_EP1INT (1 << 1)
#define UDP_IER_EP0INT (1 << 0)

#define UDP_ISR_WAKEUP (1 << 13)
#define UDP_ISR_ENDBUSRES (1 << 12)
#define UDP_ISR_SOFINT (1 << 11)
#define UDP_ISR_EXTRSM (1 << 10)
#define UDP_ISR_RXRSM (1 << 9)
#define UDP_ISR_RXSUSP (1 << 8)
#define UDP_ISR_EP7INT (1 << 7)
#define UDP_ISR_EP6INT (1 << 6)
#define UDP_ISR_EP5INT (1 << 5)
#define UDP_ISR_EP4INT (1 << 4)
#define UDP_ISR_EP3INT (1 << 3)
#define UDP_ISR_EP2INT (1 << 2)
#define UDP_ISR_EP1INT (1 << 1)
#define UDP_ISR_EP0INT (1 << 0)

#define UDP_CSR_EPEDS (1 << 15)
#define UDP_CSR_DIR (1 << 7)
#define UDP_CSR_RX_DATA_BK1 (1 << 6)
#define UDP_CSR_FORCESTALL (1 << 5)
#define UDP_CSR_TXPKTRDY (1 << 4)
#define UDP_CSR_STALLSENT (1 << 3)
#define UDP_CSR_RXSETUP (1 << 2)
#define UDP_CSR_RX_DATA_BK0 (1 << 1)
#define UDP_CSR_TXCOMP (1 << 0)
#define UDP_CSR_TYPE_POS (8)

#define UDP_TXVC_PUON (1 << 9)
#define UDP_TXVC_TXVDIS (1 << 8)

#define CSR_NOEFFECT_BITS (UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1 | \
			   UDP_CSR_STALLSENT | UDP_CSR_RXSETUP | \
			   UDP_CSR_TXCOMP)



const int udp_base_addr = 0x40034000;

/* USB Device Port UDP Registers, Page 1045 */

struct _udp {
	uint32_t frm_num;	/* 0x00 Frame Number Register */
	uint32_t glb_stat;	/* 0x04 Global State Register */
	uint32_t faddr;		/* 0x08 Function Address Register */
	uint32_t res0;		/* 0x0C Reserved */
	uint32_t ier;		/* 0x10 Interrupt Enable Register */
	uint32_t idr;		/* 0x14 Interrupt Disable Register */
	uint32_t imr;		/* 0x18 Interrupt Mask Register */
	uint32_t isr;		/* 0x1C Interrupt Status Register */
	uint32_t icr;		/* 0x20 Interrupt Clear Register */
	uint32_t res1;		/* 0x24 Reserved */
	uint32_t rst_ep;	/* 0x28 Reset Endpoint Register */
	uint32_t res2;		/* 0x2C Reserved */
	uint32_t csr[8];	/* 0x30-0x4C Ep. Control/Status Registers */
	uint32_t fdr[8];	/* 0x50-0x6c Endpoint FIFO Data Registers */
	uint32_t res3;		/* 0x70 Reserved */
	uint32_t txvc;		/* 0x74 Transceiver Control Register */
	/* 0x78 to 0xFC Reserved */
};



/*
 * UDP Device Structure with device state and other utils to keep track of it
 */

enum usb_dc_sam4s_device_state_t {
	UDP_DEV_STATE_DEFAULT = 0
};

struct _usb_dc_sam4s_t {
	uint8_t attached;
	usb_dc_status_callback status_cb;
	uint8_t state;
};

static struct _usb_dc_sam4s_t usb_dc_sam4s;



/* UDP Endpoint structure. */

enum ep_state_t {
	UDP_EP_STATE_DISABLED = 0,
	UDP_EP_STATE_IDLE,
	UDP_EP_STATE_SENDING,
	UDP_EP_STATE_EP0_STATUS_IN,
	UDP_EP_STATE_EP0_ADDRESS
};

struct _usb_dc_sam4s_ep_t {
	usb_dc_ep_callback ep_cb;
	enum ep_state_t state;
	uint8_t ep_addr;
	bool pending;
	bool zlp_needed;
	bool zlp_sent;
	bool tx_busy;
	int last_bank;
};

static struct _usb_dc_sam4s_ep_t usb_dc_sam4s_endpoints[8];


uint32_t endpoint0_packet[64];
uint32_t endpoint0_packet_count;


enum set_address_state_t {
	ADDRESS_UNSET = 0,
	ADDRESS_PENDING_TXCOMP,
	ADDRESS_SET
};

static enum set_address_state_t set_address_state;
uint32_t new_device_address;


/* Helper functions */

/*
 * @param[in] epn endpoint number
 */
static void udp_csr_set(uint8_t epn, uint32_t set_bits)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	uint32_t csr;

	csr = udp->csr[epn];
	csr |= CSR_NOEFFECT_BITS;
	csr |= set_bits;
	udp->csr[epn] = csr;

	/* After a bit is changed in the CSR you must wait 1 UDPCK clock cycle
	 * and 1 peripheral clock cycle.
	 */

	int count;

	for (count = 0; count < 15; count++) {
		__asm__ __volatile__("nop");
	}
}

/*
 * @param[in] epn endpoing number
 */
static void udp_csr_clr(uint8_t epn, uint32_t clr_bits)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	uint32_t csr;

	csr = udp->csr[epn];
	csr |= CSR_NOEFFECT_BITS;
	csr &= ~clr_bits;
	udp->csr[epn] = csr;

	/* After a bit is changed in the CSR you must wait 1 UDPCK clock cycle
	 * and 1 peripheral clock cycle.
	 */

	int count;

	for (count = 0; count < 15; count++) {
		__asm__ __volatile__("nop");
	}
}



/* Private Functions */

/*
 * Read the FIFO for an endpoint and store it into an interal buffer.
 */
static void udp_read_ep(int epn)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	int i;
	int total;
	uint32_t csr;

	/* Copy the CSR register. */

	csr = udp->csr[epn];

	/* Copy data from the FIFO to an internal FIFO. */

	if (epn == 0) {
		/*
		 * Get the total number of bytes to read from the FIFO
		 * register.
		 */

		total = ((csr & 0xffff0000) >> 16);

		SYS_LOG_DBG("Reading %d bytes", total);

		/* Read the bytes from the FIFO. */

		for (i = 0; i < total; i++) {
			endpoint0_packet[i] = udp->fdr[epn];
		}

		endpoint0_packet_count = total;
	}
}

/*
 * Called only by the interrupt service routine if an endpoint had an
 * interrupt.
 *
 * @param[in] endpoint index
 */
static void udp_ep_isr(int epn)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	uint32_t csr;
	struct _usb_dc_sam4s_ep_t *endpoint;
	bool bk0;
	bool bk1;

	SYS_LOG_DBG("EP%d Interrupt", epn);

	/* Get the endpoint status. */

	csr = udp->csr[epn];

	/* Get the endpoint structure. */

	endpoint = &usb_dc_sam4s_endpoints[epn];

	/* TXCOMP: IN packet sent and acknowledged by the host. */

	if (csr & UDP_CSR_TXCOMP) {
		SYS_LOG_DBG("TXCOMP: IN");

		/* Clear the TXCOMP interrupt. */

		udp_csr_clr(epn, UDP_CSR_TXCOMP);

		/* Special case for SET_ADDRESS. The transfer of the ZLP
		 * must complete before the address is actully set.
		 */

		if (epn == 0 && set_address_state == ADDRESS_PENDING_TXCOMP) {

			/* Enable the address */

			udp->faddr |= UDP_FADDR_FEN | new_device_address;

			/* Go to the addressed by not configured state. */

			udp->glb_stat |= UDP_GLB_STAT_FADDEN;
			udp->glb_stat &= ~UDP_GLB_STAT_CONFIG;

			/* Set the new address state. */

			set_address_state = ADDRESS_SET;
		}

		/*
		 * Let upper layer know that the IN transaction is done on the
		 * EP.
		 */

		endpoint->ep_cb(USB_EP_DIR_IN | epn, USB_DC_EP_DATA_IN);
	}

	/* Check is we have received data. */

	bk0 = (csr & UDP_CSR_RX_DATA_BK0);
	bk1 = (csr & UDP_CSR_RX_DATA_BK1);

	if (bk0 || bk1) {

		/* Let the upper layer know there is data available. */

		endpoint->ep_cb(USB_EP_DIR_OUT | epn, USB_DC_EP_DATA_OUT);
	}

	/* STALL sent */

	if (csr & UDP_CSR_STALLSENT) {
		SYS_LOG_DBG("STALL sent.");
		udp_csr_clr(epn, UDP_CSR_STALLSENT);
	}

	/* SETUP packet received. */

	if (csr & UDP_CSR_RXSETUP) {
		SYS_LOG_DBG("SETUP packet received.");

		/* If a write request transfer was pending, complete it. */

		if (endpoint->state == UDP_EP_STATE_SENDING) {
			/* TODO sam_req_complete */
		}

		/* Copy the setup data from the UDP EP FIFO (UDP FIFO Data
		 * Register) to an endpoint buffer.
		 */

		udp_read_ep(epn);

		/*
		 * Look at the packet byte 0, bit 7 to tell what kind of CONTROL
		 * Transfer this is. A 0 is Host-to-device, a 1 is
		 * Device-to-host.
		 */

		bool direction_is_in;

		direction_is_in = (endpoint0_packet[0] & 0x80);

		/*
		 * Is this an OUT transaction, where the host is sending us
		 * data?
		 */

		if (!direction_is_in) {
			SYS_LOG_DBG("SETUP request is OUT");

			/*
			 * If it is an OUT request then we have to wait for the
			 * OUT data phase to complete before processing the
			 * SETUP command.
			 *
			 * Clear the CSR:DIR bit to support the host-to-device
			 * data OUT data transfer. This bit must be cleared
			 * before CSR:RXSETUP is cleared at the end of the SETUP
			 * stage.
			 */

			 udp_csr_clr(epn, UDP_CSR_DIR);

			 /* Clear the RXSETUP indication. RXSETUP cannot be
			  * cleared before the SETUP packet has been read in
			  * from the FIFO.  Otherwise, the USB device would
			  * accept the next Data OUT transfer and overwrite the
			  * SETUP packet in the FIFO (p1056).
			  */

			 udp_csr_clr(epn, UDP_CSR_RXSETUP);

			 /* Update the upper layers. */

			 endpoint->ep_cb(USB_EP_DIR_OUT | epn, USB_DC_EP_SETUP);

		}

		/*
		 * Is this a SETUP IN request or a SETUP IN with no data. An
		 * IN request means we have to send the host data.
		 */

		else {
			SYS_LOG_DBG("SETUP request is IN");

			/*
			 * Set the CSR:DIR bit to support the device-to-host
			 * data IN data transfer.  This bit must be set before
			 * CSR:RXSETUP is cleared at the end of the SETUP stage.
			 */

			udp_csr_set(epn, UDP_CSR_DIR);

			/* Clear the RXSETUP bit. */

			udp_csr_clr(epn, UDP_CSR_RXSETUP);

			/* Handle the SETUP out command now. */

			endpoint->ep_cb(USB_EP_DIR_OUT | epn, USB_DC_EP_SETUP);

		}
	}
}

/*
 * Reset and disable all the endpoints.
 */
static void udp_reset_endpoints(void)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	int epn; /* Endpoint Number */

	for (epn = 0; epn < 8; epn++) {

		/* Disable the endpoint interrupt. */

		udp->idr = (1 << epn);

		/*
		 * Reset the endpoint FIFO. Resetting the endpoint is a two-step
		 * process (p1058).
		 *
		 * 1. Set the EPx field.
		 * 2. Clear the EPx field.
		 */

		udp->rst_ep = (1 << epn);
		udp->rst_ep &= ~(1 << epn);

		/* Reset the endpoint status. */

		usb_dc_sam4s_endpoints[epn].state = UDP_EP_STATE_DISABLED;
		usb_dc_sam4s_endpoints[epn].pending = false;
		usb_dc_sam4s_endpoints[epn].zlp_needed = false;
		usb_dc_sam4s_endpoints[epn].zlp_sent = false;
		usb_dc_sam4s_endpoints[epn].tx_busy = false;
		usb_dc_sam4s_endpoints[epn].last_bank = 1;
	}
}

/*
 * Called when a RESET event happens.
 */
static void udp_reset_event(void)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);

	/* Select which PLL is providing the 48 MHz clock to the UDP. */

	__PMC->usb = 1;

	/* Enable the UDP transceiver clock (UDPCL, 48 MHz). */

	__PMC->scer = BIT(7);

	/* Enable the UDP peripheral clock (MCK). */

	__PMC->pcer1 = BIT(2);

	/* Tell the class driver we are disconnected. */

	usb_dc_sam4s.status_cb(USB_DC_DISCONNECTED);

	/*
	 * The device enters the Default state (un-addressed and un-configured).
	 */

	usb_dc_set_address(0);
	usb_dc_sam4s.state = UDP_DEV_STATE_DEFAULT;

	/* Reset and disable all endpoints. Then configure endpoint 0. */

	udp_reset_endpoints();

	/* Reset endpoint 0 CSR. */

	udp->csr[0] = 0;

	/* Disable endpoint 0 interrupt. */

	udp->idr = 1;

	/* Set the endpoint state to idle. */

	usb_dc_sam4s_endpoints[0].state = UDP_EP_STATE_IDLE;

	/* Enable endpoint 0 and set the type to control. */

	udp->csr[0] |= (UDP_CSR_EPEDS | (0 << UDP_CSR_TYPE_POS));

	/* Enable endpoint 0 interrupt. */

	udp->ier = 1;

	/* Clear all pending interrupts. */

	udp->icr = UDP_ISR_WAKEUP | UDP_ISR_ENDBUSRES | UDP_ISR_SOFINT
		   | UDP_ISR_RXSUSP;

	/* Enable normal operation interrupts including endpoint 0. */

	udp->ier = UDP_IER_WAKEUP | UDP_IER_RXSUSP | 1;
}

/*
 * Interrupt service routine for the UDP.
 */
static void udp_isr(void)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	uint32_t pending_isr;

	pending_isr = (udp->isr & udp->imr);

	while (pending_isr) {

		/* Suspend */

		if (pending_isr & UDP_ISR_RXSUSP) {
			SYS_LOG_DBG("Suspend");

			/*
			 * Disable the suspend interrupt and enable the
			 * wakeup/resume interrupts.
			 */

			udp->idr = UDP_IER_RXSUSP;
			udp->ier = UDP_IER_WAKEUP | UDP_ISR_RXRSM;

			/* Clear the pending suspend interrupts. */

			udp->icr = UDP_ISR_RXSUSP | UDP_ISR_WAKEUP;

			/* Notify the upper layers. */

			usb_dc_sam4s.status_cb(USB_DC_SUSPEND);
		}

		/* Start-of-Frame SOF */

		if (pending_isr & UDP_ISR_SOFINT) {
			SYS_LOG_DBG("SOF");

			/* Clear the pending interrupt */

			udp->icr = UDP_ISR_SOFINT;
		}

		/* Resume or wakeup */

		if (pending_isr & (UDP_ISR_WAKEUP | UDP_ISR_RXRSM)) {
			SYS_LOG_DBG("Resume");

			/* Clear the pending interrupt. */

			udp->icr = UDP_ISR_WAKEUP | UDP_ISR_RXRSM
				   | UDP_ISR_RXSUSP;

			/* Enable suspend interrupts. */

			udp->ier = UDP_ISR_WAKEUP | UDP_ISR_RXRSM;

			/* Let the upper layer know we have resumed. */

			usb_dc_sam4s.status_cb(USB_DC_RESUME);
		}

		/* Reset, end-of-reset interrupt. */

		if (pending_isr & UDP_ISR_ENDBUSRES) {
			SYS_LOG_DBG("Reset");

			/* Clear the reset interrupt. */

			udp->icr = UDP_ISR_ENDBUSRES;

			udp_reset_event();
		}

		/* Endpoint Interrupts */

		if (pending_isr & 0xff) {
			int epn;

			for (epn = 0; epn < 8; epn++) {
				if (pending_isr & (1 << epn)) {
					udp_ep_isr(epn);
				}
			}
		}

		/* Update pending interrupts check */

		pending_isr = (udp->isr & udp->imr);
	}
}




/* API usb_dc.h Functions */

/*
 * This is the first function called. It is called by usb_enable() in
 * usb_device.c, which is called by the init function of the class using USB,
 * such as cdc_acm.c. This setups up the Attached State (40.6.3.2) and the
 * powered state. At the end of this function we are in powered state waiting
 * to transition to Default State.
 */
int usb_dc_attach(void)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	unsigned int key;

	key = irq_lock();

	/* Make sure you have a clock going to the USBCK that is 48 MHz. */

	/* Select which PLL is providing the 48 MHz clock to the UDP. */

	__PMC->usb = 1;

	/* Enable the UDP transceiver clock (UDPCL, 48 MHz). */

	__PMC->scer = BIT(7);

	/* Enable the UDP peripheral clock (MCK). */

	__PMC->pcer1 = BIT(2);

	/* Enable fast restart of the PMC on USB alarm
	 * This is for suspended mode and low power modes.
	 */

	__PMC->fsmr |= (1 << 18);

	/* Enable transceiver. */

	udp->txvc &= ~UDP_TXVC_TXVDIS;

	/* Attach device. */

	udp->txvc |= UDP_TXVC_PUON;

	/* Turn on interrupts. */

	udp->ier =  UDP_IER_RXSUSP | UDP_IER_WAKEUP | UDP_IER_RXRSM
		| UDP_IER_EXTRSM | UDP_IER_SOFINT;

	/* Connect and enable the UDP's USB interrupt. */

	IRQ_CONNECT(IRQ_UDP, CONFIG_USB_SAM4S_IRQ_PRI,
		udp_isr, 0, 0);
	irq_enable(IRQ_UDP);

	usb_dc_sam4s.attached = 1;

	irq_unlock(key);

	return 0;
}

int usb_dc_detach(void)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);

	/* Check to see if we are attached. */

	if (!usb_dc_sam4s.attached) {
		return 0;
	}

	/* Disable the transceiver. */

	udp->txvc |= UDP_TXVC_TXVDIS;

	/* Detach device from the bus. */

	udp->txvc &= ~UDP_TXVC_PUON;

	usb_dc_sam4s.attached = 0;

	return 0;
}

int usb_dc_reset(void)
{
	/* Clear state data. */
	memset(&usb_dc_sam4s, 0, sizeof(usb_dc_sam4s));

	return 0;
}

/*
 * When this function gets called we are moving from Default State to
 * Address State (40.6.3.4).
 */
int usb_dc_set_address(const uint8_t addr)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);

	SYS_LOG_DBG("Setting address to 0x%x", addr);

	if (addr) {

		/*
		 * Store the new device address. We can't actully set the
		 * address here because the algorithm in usb_device.c send the
		 * ZLP after calling this function. You can't set the address
		 * until you've sent the ZLP. So we store the address and wait
		 * for the TXCOMP indicating the host got our ZLP response.
		 */

		new_device_address = addr;

		set_address_state = ADDRESS_PENDING_TXCOMP;

	} else {
		/*
		 * Set the address to zero. The FEN bit must still be set
		 * in order to receive or send data packets from or to the
		 * host.
		 */

		udp->faddr = UDP_FADDR_FEN;

		/*
		 * Make sure we are not in either the configured or addressed
		 * state.
		 */

		udp->glb_stat &= ~(UDP_GLB_STAT_FADDEN | UDP_GLB_STAT_CONFIG);

		set_address_state = ADDRESS_UNSET;

		new_device_address = 0;
	}

	return 0;
}

int usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	usb_dc_sam4s.status_cb = cb;
	return 0;
}



/* API Endpoint Functions */

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data * const cfg)
{
	int epn;

	/*
	 * The endpoint the upper layer sends has a 0x80 or a 0x00 indicating
	 * if the endpoint is IN or OUT. Remove that mask.
	 */

	epn = (cfg->ep_addr & 0x7f);

	/* Skip endpoint 0 because it is always configured by default. */

	if (epn == 0) {
		SYS_LOG_DBG("EP%d skipped configuration", epn);
		return 0;
	}

	/* There are 8 endpoints, 0 to 7. */

	if (epn > 7) {
		SYS_LOG_ERR("Invalid endpoint number: %d", epn);
		return -EINVAL;
	}

	uint8_t type;
	bool dir_in = (cfg->ep_addr & USB_EP_DIR_MASK);

	/* Set the endpoint type p1062. */

	switch (cfg->ep_type) {
	case USB_DC_EP_CONTROL:
		type = 0;
		break;
	case USB_DC_EP_ISOCHRONOUS:
		if (dir_in) {
			type = 5;
		} else {
			type = 1;
		}
		break;
	case USB_DC_EP_BULK:
		if (dir_in) {
			type = 6;
		} else {
			type = 2;
		}
		break;
	case USB_DC_EP_INTERRUPT:
		if (dir_in) {
			type = 7;
		} else {
			type = 3;
		}
		break;
	default:
		SYS_LOG_ERR("Invalid endpoint type requested.");
		return -EINVAL;
	}

	/* CSR is on page 1059. */

	udp_csr_set(epn, (type << 8));

	/*
	 * The max. endpoint size for each endpoint in the UDP is defined in
	 * Table 40-1 p1028.
	 */

	switch (epn) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 6:
	case 7:
		if (cfg->ep_mps > 64) {
			SYS_LOG_ERR("Requested too large of a packet size.");
			return -EINVAL;
		}
		break;
	case 4:
	case 5:
		if (cfg->ep_mps > 512) {
			SYS_LOG_ERR("Requested too large of a packet size.");
			return -EINVAL;
		}
		break;
	}

	usb_dc_sam4s_endpoints[epn].ep_addr = cfg->ep_addr;

	SYS_LOG_DBG("EP%d configured", epn);

	return 0;
}

/*
 * Refer to 40.6.2.4 Stall Handshake. This is the start of the steps in that
 * section.
 */
int usb_dc_ep_set_stall(const uint8_t ep)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	uint8_t epn = (ep & 0x7f);

	/* There are 8 endpoints, 0 to 7. */

	if (epn > 7) {
		SYS_LOG_ERR("Invalid endpoint number: %d", ep);
		return -EINVAL;
	}

	/* Send STALL to the host. Page 1061. */

	udp->csr[epn] = UDP_CSR_FORCESTALL;

	SYS_LOG_DBG("EP%d set stall.", epn);

	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	uint8_t epn = (ep & 0x7f);

	/* There are 8 endpoints, 0 to 7. */

	if (epn > 7) {
		SYS_LOG_ERR("Invalid endpoint number: %d", epn);
		return -EINVAL;
	}

	/* Writing a 0 to this bit returns to normal state. Page 1061. */

	udp->csr[epn] &= ~UDP_CSR_FORCESTALL;

	SYS_LOG_DBG("EP%d clear stall.", epn);

	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	uint8_t epn = (ep & 0x7f);

	/* There are 8 endpoints, 0 to 7. */

	if (epn > 7) {
		SYS_LOG_ERR("Invalid endpoint number: %d", epn);
		return -EINVAL;
	}

	/*
	 * Read the FORCESTALL bit to check if the endpoint is stalled.
	 * Page 1061.
	 */

	*stalled = (udp->csr[ep] & UDP_CSR_FORCESTALL);

	SYS_LOG_DBG("EP%d is stalled %d", epn, *stalled);

	return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	SYS_LOG_DBG("EP%d halted.", ep & 0x7f);

	return -ENOSYS;
}

int usb_dc_ep_enable(const uint8_t ep)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	int epn;

	/*
	 * The endpoint the upper layer sends has a 0x80 or a 0x00 indicating
	 * if the endpoint is IN or OUT. Remove that mask.
	 */

	epn = (ep & 0x7f);

	if (epn == 0) {

		/* The endpoint 0 is always configured. */

		SYS_LOG_DBG("EP0 Skipped enable.");

		return 0;
	}

	/* There are 8 endpoints, 0 to 7. */

	if (epn > 7) {
		SYS_LOG_ERR("Invalid endpoint number: %d", epn);
		return -EINVAL;
	}

	/* Enable the endpoint interrupt. */

	udp->ier = (1 << epn);

	/* Enable the endpoint. */

	udp_csr_set(epn, UDP_CSR_EPEDS);

	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	int epn;

	/* The endpoint the upper layer sends has a 0x80 or a 0x00 indicating
	 * if the endpoint is IN or OUT. Remove that mask.
	 */

	epn = (ep & 0x7f);

	/* There are 8 endpoints, 0 to 7. */

	if (epn > 7) {
		SYS_LOG_ERR("Invalid endpoint number: %d", epn);
		return -EINVAL;
	}

	/* Disable the endpoint interrupt. */

	udp->icr = (1 << epn);

	/* Disable the endpoint. */

	udp->csr[epn] = 0;

	SYS_LOG_DBG("EP%d disabled.", epn);

	return 0;
}

/* This function flushes the FIFOs for the selected endpoint. */
int usb_dc_ep_flush(const uint8_t ep)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	uint8_t ept = (ep & 0x7f);
	/* There are 8 endpoints, 0 to 7. */
	if (ept > 7) {
		SYS_LOG_ERR("Invalid endpoint number: %d", ept);
		return -EINVAL;
	}

	/* Page 1058 */
	udp->rst_ep = (1 << ept);
	udp->rst_ep &= ~(1 << ept);

	SYS_LOG_DBG("EP%d flushed.", ept);

	return 0;
}

/* This is a Data IN Transaction. */
int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data,
		const uint32_t data_len, uint32_t * const ret_bytes)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	uint8_t epn = (ep & 0x7f);

	/* 40.6.2.2 Device to Host (so a write). */

	/* 1. Application checks if it is possible to write in the FIFO by
	 *    by polling TXPKTRDY.
	 */

	while ((udp->csr[epn] & UDP_CSR_TXPKTRDY))
		;

	/* 2. Application write the first packet of data to be sent in the
	 *    endpoints FIFO, writing zero or more byte values in the
	 *    endpoint's UDP_FDRx.
	 */

	int i;

	for (i = 0; i < data_len; i++) {
		// SYS_LOG_DBG("Data IN 0x%x", data[i]);
		udp->fdr[epn] = data[i];
	}

	/* 3. Application notifies the USB peripheral it has finished by
	 *    setting the TXPKTRDY in the endpoint's UDP_CSR.
	 */

	udp_csr_set(epn, UDP_CSR_TXPKTRDY);

	/* 4. The application is notified that the endpoint's FIFO has been
	 *    released by the USB device when TXCOMP in the endpoints's
	 *    UDP_CSRx has been set. Then an interrupt for the corresponding
	 *    endpoint is pending while TXCOMP is set.
	 *
	 *    We are done at this point with setting up the Data IN
	 *    transaction. The endpoint interrupt handler will send
	 *    information back to the upper layers via the endpoint callback.
	 */

	SYS_LOG_DBG("EP%d write initated. 0x%x, len 0x%x",
		epn, udp->faddr, data_len);

	return 0;
}

/* This is a Data OUT Transaction or a Setup Transaction. */
int usb_dc_ep_read(const uint8_t ep, uint8_t *const data,
		   const uint32_t max_data_len, uint32_t *const read_bytes)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	uint8_t epn = (ep & 0x7f);
	struct _usb_dc_sam4s_ep_t *endpoint;
	bool bk0;
	bool bk1;
	int i;
	int len;

	/* Get the length of the first buffer. */

	len = ((udp->csr[epn] & 0xffff0000) >> 16);

	/*
	 * Check if the upper layer sent a NULL for the data and a 0 for
	 * the max_data_len. If they did it means they want to know the
	 * amount to read.
	 */

	if (data == NULL && max_data_len == 0) {

		*read_bytes = len;

		return 0;
	}

	/* Get the endpoint structure. */

	endpoint = &usb_dc_sam4s_endpoints[epn];

	if (len > max_data_len && epn != 0) {
		SYS_LOG_ERR("Not enough room in read buffer.");

		/* We copy out as much as the buffer can hold. */

		len = max_data_len;
	}

	/*
	 * Check if this is the control endpoint out or one of the out
	 * endpoints. If it is not return an error.
	 */

	if (epn == 0) {
		for (i = 0; i < endpoint0_packet_count; i++) {
			data[i] = endpoint0_packet[i];
		}

		*read_bytes = endpoint0_packet_count;

		endpoint0_packet_count = 0;

		udp_csr_clr(epn, UDP_CSR_RX_DATA_BK0);

		return 0;
	}

	/*
	 * OUT packet received.
	 *
	 * OUT packets are received in two banks. The hardware does not provide
	 * information about which bank has been filled last. Therefore we need
	 * to keep track about which bank we read last to figure out which
	 * bank(s) we need to read next.
	 *
	 * When we get here either none, one or both banks can be filled with
	 * data. Depending on which bank we read last and which bank(s) contain
	 * data we need to correctly sequence the FIFO reads:
	 *
	 * case  lastbank      bk0    bk1     read sequence
	 *  1.      0           0      0      No data to read
	 *  2.      0           1      0      Only read bank 0
	 *  3.      0           0      1      Only read bank 1
	 *  4.      0           1      1      Read bank 1, then read bank 0
	 *
	 *  5.      1           0      0      No data to read
	 *  6.      1           1      0      Only read bank 0
	 *  7.      1           0      1      Only read bank 1, should not occur
	 *  8.      1           1      1      Read bank 0, then read bank 1
	 *
	 * lastbank will be updated in sam_req_read() after the FIFO has been
	 * read and clear RXDATABKx.
	 */

	/* Check if we have received data. */

	bk0 = (udp->csr[epn] & UDP_CSR_RX_DATA_BK0);
	bk1 = (udp->csr[epn] & UDP_CSR_RX_DATA_BK1);

	/* Case 2 and 6. Only read bank 0. */

	if (bk0 && !bk1) {
		SYS_LOG_DBG("Reading bank 0");

		for (i = 0; i < len; i++) {
			data[i] = udp->fdr[epn];
			SYS_LOG_DBG("0x%x", data[i]);
		}

		*read_bytes = len;

		endpoint->last_bank = 0;

		udp_csr_clr(epn, UDP_CSR_RX_DATA_BK0);
	}

	/* Case 3 and 7. Only read bank 1. */

	else if (!bk0 && bk1) {
		SYS_LOG_DBG("Reading bank 1");

		for (i = 0; i < len; i++) {
			data[i] = udp->fdr[epn];
			SYS_LOG_DBG("0x%x", data[i]);
		}

		*read_bytes = len;

		endpoint->last_bank = 0;

		udp_csr_clr(epn, UDP_CSR_RX_DATA_BK1);
	}

	/* Case 4 and 8. */

	else if (bk0 && bk1) {

		/* Case 4. Read bank 1, then read bank 0. */

		if (endpoint->last_bank == 0) {
			SYS_LOG_DBG("Reading bank 1 then bank 0");

			for (i = 0; i < len; i++) {
				data[i] = udp->fdr[epn];
				SYS_LOG_DBG("0x%x", data[i]);
			}

			*read_bytes = len;

			udp_csr_clr(epn, UDP_CSR_RX_DATA_BK1);

			len = ((udp->csr[epn] & 0xffff0000) >> 16);

			for (i = 0; i < len; i++) {
				data[i] = udp->fdr[epn];
				SYS_LOG_DBG("0x%x", data[i]);
			}

			*read_bytes = *read_bytes + len;

			endpoint->last_bank = 0;

			udp_csr_clr(epn, UDP_CSR_RX_DATA_BK0);
		}

		/* Case 8. Read bank 0. then read bank 1. */

		else {
			SYS_LOG_DBG("Reading bank 0 then bank 1");

			for (i = 0; i < len; i++) {
				data[i] = udp->fdr[epn];
				SYS_LOG_DBG("0x%x", data[i]);
			}

			*read_bytes = len;

			udp_csr_clr(epn, UDP_CSR_RX_DATA_BK0);

			len = ((udp->csr[epn] & 0xffff0000) >> 16);

			for (i = 0; i < len; i++) {
				data[i] = udp->fdr[epn];
				SYS_LOG_DBG("0x%x", data[i]);
			}

			*read_bytes = *read_bytes + len;

			endpoint->last_bank = 1;

			udp_csr_clr(epn, UDP_CSR_RX_DATA_BK1);
		}
	}

	return 0;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	int epn = (ep & 0x7f);

	/* There are 8 endpoints, 0 to 7. */

	if (epn > 7) {
		SYS_LOG_ERR("Invalid endpoint number: %d", epn);
		return -EINVAL;
	}

	/* If there was no callback submitted we are done. */

	if (cb == NULL) {
		return 0;
	}

	/* Set the callback for the endpoint. */

	usb_dc_sam4s_endpoints[epn].ep_cb = cb;

	SYS_LOG_DBG("EP%d callback set.", epn);

	return 0;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len,
			uint32_t *read_bytes)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	uint8_t ept = (ep & 0x7f);

	SYS_LOG_DBG("EP%d read wait initiated.", ept);

	/* RXBYTECNT in CSRx is the number of bytes available in the FIFO. */
	uint32_t available_data_len = (udp->csr[ept] >> 16);

	if (available_data_len > max_data_len) {
		SYS_LOG_ERR("Not enough room in read buffer.");
		/* We copy out as much as the buffer can hold. */
		available_data_len = max_data_len;
	}

	int i;

	for (i = 0; i < available_data_len; i++) {
		data[i] = udp->fdr[ept];
	}

	*read_bytes = available_data_len;

	return 0;
}

int usb_dc_ep_read_continue(uint8_t ep)
{
	volatile struct _udp *udp = (volatile struct _udp *)(udp_base_addr);
	uint8_t ept = (ep & 0x7f);

	SYS_LOG_DBG("EP%d read continue...", ept);

	if (udp->csr[ept] & UDP_CSR_RX_DATA_BK0) {
		udp->csr[ept] &= ~UDP_CSR_RX_DATA_BK0;
		SYS_LOG_DBG("EP%d RX_DATA_BK0 cleared.", ept);
	}

	if (udp->csr[ept] & UDP_CSR_RXSETUP) {
		udp->csr[ept] &= ~UDP_CSR_RXSETUP;
		SYS_LOG_DBG("EP%d RXSETUP cleared.", ept);
	}

	return 0;
}
