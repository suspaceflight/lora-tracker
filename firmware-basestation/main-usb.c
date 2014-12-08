/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2014 Kuldeep Singh Dhaka <kuldeepdhaka9@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/stm32/crs.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f0/nvic.h>
#include <libopencm3/cm3/nvic.h>

#include "radio.h"

char buff[128];

//usbd_device *usbd_dev;

//void rcc_clock_setup_in_hsi48_out_48mhz(void);
void crs_configure_usbsof_autotrim(void);
//void rcc_set_usbclk_source(enum rcc_osc clk);
enum rcc_osc rcc_usb_clock_source(void);
void rcc_clock_setup_in_hsi_out_48mhz_corrected(void);
void _delay_ms(const uint32_t delay);

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	 }
};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors)
}};

static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Black Sphere Technologies",
	"CDC-ACM Demo",
	"DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static int cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch(req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		char local_buf[10];
		struct usb_cdc_notification *notif = (void *)local_buf;

		/* We echo signals back to host as notification. */
		notif->bmRequestType = 0xA1;
		notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
		notif->wValue = 0;
		notif->wIndex = 0;
		notif->wLength = 2;
		local_buf[8] = req->wValue & 3;
		local_buf[9] = 0;
		// usbd_ep_write_packet(0x83, buf, 10);
		return 1;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if(*len < sizeof(struct usb_cdc_line_coding))
			return 0;

		return 1;
	}
	return 0;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

	if (len) {
	//	usbd_ep_write_packet(usbd_dev, 0x82, buf, len);
		buf[len] = 0;
	}

	int i = 0;
	while(buf[i])
		usart_send_blocking(USART1, buf[i++]);

}

static void cdcacm_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	char buf[64];
	buf[0] = 'f';
//	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

//	if (len) {
		usbd_ep_write_packet(usbd_dev, 0x82, buf, 1);
//		buf[len] = 0;
//	}

//	int i = 0;
//	while(buf[i])
//		usart_send_blocking(USART1, buf[i++]);

}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);// cdcacm_data_tx_cb);//NULL);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}

void rcc_clock_setup_in_hsi_out_48mhz_corrected(void)
{
	rcc_osc_on(HSI);
	rcc_wait_for_osc_ready(HSI);
	rcc_set_sysclk_source(HSI);

	// correction (f072 has PREDIV after clock multiplexer (near PLL)
	//Figure 12. Clock tree (STM32F07x devices)  P96 	RM0091
	//applies to rcc_clock_setup_in_hsi_out_*mhz()
	rcc_set_prediv(RCC_CFGR2_PREDIV_DIV2);

	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
	rcc_set_ppre(RCC_CFGR_PPRE_NODIV);

	flash_set_ws(FLASH_ACR_LATENCY_024_048MHZ);

	// 8MHz * 12 / 2 = 48MHz
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL12);

	RCC_CFGR &= ~RCC_CFGR_PLLSRC;

	rcc_osc_on(PLL);
	rcc_wait_for_osc_ready(PLL);
	rcc_set_sysclk_source(PLL);

	rcc_ppre_frequency = 48000000;
	rcc_core_frequency = 48000000;
}




void crs_configure_usbsof_autotrim(void)
{
	rcc_periph_clock_enable(RCC_CRS);

	CRS_CFGR &= ~CRS_CFGR_SYNCSRC;
	CRS_CFGR |= CRS_CFGR_SYNCSRC_USB_SOF;

	CRS_CR |= CRS_CR_AUTOTRIMEN;
	CRS_CR |= CRS_CR_CEN;
}


void init(void)
{

	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9|GPIO10);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO9|GPIO10);
	usart_set_baudrate(USART1, 9600 );
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);

	//setup pin interrupts for radio
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO6);

	// Clock the SYSCFG peripheral
	//rcc_periph_clock_enable(RCC_SYSCFG_COMP);
	RCC_APB2ENR |= (1<<0);
	//select portB as source for EXTI15
	//SYSCFG_EXTICR(4) |= (1<<12);
	//EXTI_IMR |= (GPIO15);
	//EXTI_RTSR |= GPIO15;   //rising edge interrupt
	exti_select_source(EXTI15, GPIOB);
	exti_set_trigger(EXTI15, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI15);

    // Enable EXTI interrupts in NVIC
	nvic_enable_irq(NVIC_EXTI4_15_IRQ);


}

void exti4_15_isr(void)
{
	if ((EXTI_PR & (GPIO15)) != 0)  //radio irq
	{
		int i;
		uint8_t r = radio_check_read_rx_packet(128,(uint8_t*)buff,1);
		if (r > 0)
		{
			for (i = 0; i < r; i++)
				usart_send_blocking(USART1, buff[i]);
			//usbd_ep_write_packet(usbd_dev, 0x82, buff, r);

			int16_t snr = radio_read_single_reg(REG_PKT_SNR_VALUE);
			if (snr & 0x80)
				snr |= 0xFF00;
			int16_t rssi = radio_read_single_reg(REG_PKT_RSSI_VALUE)-164;
			int32_t error = radio_read_single_reg(REG_FEI_MSB_LORA) << 8;
			error = (error | radio_read_single_reg(REG_FEI_MID_LORA)) << 8;
			error |= radio_read_single_reg(REG_FEI_LSB_LORA);

			r = snprintf(buff,60,"snr: %i  rssi: %i offset: %li     ",snr,rssi,error);
			i=0;
			while (buff[i])
				usart_send_blocking(USART1, buff[i++]);
				/*
			usbd_ep_write_packet(usbd_dev, 0x82, buff, r);
			*/
		}

		EXTI_PR |= (GPIO15);
	}

}

int main(void)
{

	usbd_device *usbd_dev;


	rcc_clock_setup_in_hsi48_out_48mhz();
	crs_configure_usbsof_autotrim();
	rcc_set_usbclk_source(HSI48);
	rcc_set_sysclk_source(HSI48);
	init();

	_delay_ms(100);
	radio_init();
	uint8_t v = radio_read_version();
	radio_lora_settings_t s;
	s.spreading_factor = 12;
	s.bandwidth = BANDWIDTH_20_8K;
	s.coding_rate = CODING_4_5;
	s.implicit_mode = 0;
	s.crc_en = 0;
	s.low_datarate = 1;

	radio_write_lora_config(&s);
	//radio_high_power();
	radio_pa_off();
	radio_lna_max();
	radio_set_frequency(FREQ_434_100);
	radio_set_continuous_rx();


	int k,j;
	uint8_t r;
	uint8_t i=0;
	for (k = 0; k <= 7; k++){
		for (j = 0; j < 16; j++){
			r = radio_read_single_reg(k*16 + j);
			snprintf(buff,60,"%02X ",r);
			i=0;
			while (buff[i])
				usart_send_blocking(USART1, buff[i++]);
		}

		usart_send_blocking(USART1, '\r');
		usart_send_blocking(USART1, '\n');
	}



	while(1)
	{}{
		uint8_t stat = radio_read_single_reg(REG_MODEM_STAT);
		uint8_t irq = radio_read_single_reg(REG_IRQ_FLAGS);
		uint8_t nb = radio_read_single_reg(REG_RX_NB_BYTES);
		uint8_t hrx = radio_read_single_reg(REG_RX_HEADER_CNT_VALUE_LSB);

		snprintf(buff,60,"stat: %X  irq: %X headers rx: %X nBytes: %d\r\n",stat,irq,hrx,nb);
		i=0;
		while (buff[i])
			usart_send_blocking(USART1, buff[i++]);
		_delay_ms(1000);
	}


	usbd_dev = usbd_init(&stm32f0x2_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

uint8_t stat_l = 0;
/*
	while(1)
	{

		uint8_t stat = radio_read_single_reg(REG_MODEM_STAT);
		snprintf(buff,60,"%X ",stat);
		i=0;
		if (stat_l != stat){
			while (buff[i])
				usart_send_blocking(USART1, buff[i++]);
		}
		stat_l = stat;

		i=0;
		uint8_t r = radio_check_read_rx_packet(128,(uint8_t*)buff,0);
		if (r > 0)
		{
			for (i = 0; i < r; i++)
				usart_send_blocking(USART1, buff[i]);

			int16_t snr = radio_read_single_reg(REG_PKT_SNR_VALUE);
			int16_t rssi = radio_read_single_reg(REG_PKT_RSSI_VALUE)-164;
			int32_t error = radio_read_single_reg(REG_FEI_MSB_LORA) << 8;
			error = (error | radio_read_single_reg(REG_FEI_MID_LORA)) << 8;
			error |= radio_read_single_reg(REG_FEI_LSB_LORA);

			snprintf(buff,60,"snr: %i  rssi: %i offset: %i     ",snr,rssi,error);
			i=0;
			while (buff[i])
				usart_send_blocking(USART1, buff[i++]);
		}


		_delay_ms(600);
	}
*/
	while (1){
		usbd_poll(usbd_dev);
		//uint32_t intr = EXTI_PR;

 /*
		uint8_t r = radio_check_read_rx_packet(128,(uint8_t*)buff,1);
		if (r > 0)
		{
			for (i = 0; i < r; i++)
				usart_send_blocking(USART1, buff[i]);
		} */
	}
}

void _delay_ms(const uint32_t delay)
{
    uint32_t i, j;

    for(i=0; i< delay; i++)
        for(j=0; j<1000; j++)
            __asm__("nop");
}
