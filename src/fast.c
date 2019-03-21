/*
 *
 * Copyright (C) 2019 Kongou Hikari <kongouhikari@qq.com>
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
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <string.h>
#include <stdio.h>

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
unsigned char configured = 0;
usbd_device *usbd_dev_handler;
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
	 },
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
	.extralen = sizeof(cdcacm_functional_descriptors),
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
	"Kongou Hikari",
	"Oscilloscope Display CDC Data port",
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

	switch (req->bRequest) {
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
		if (*len < sizeof(struct usb_cdc_line_coding))
			return 0;
		return 1;
	}
	return 0;
}


/* USB interrupt handler */
void usb_wakeup_isr(void)
	__attribute__ ((alias ("usb_int_relay")));

void usb_hp_can_tx_isr(void)
	__attribute__ ((alias ("usb_int_relay")));

void usb_lp_can_rx0_isr(void)
	__attribute__ ((alias ("usb_int_relay")));

static void usb_int_relay(void) {
  /* Need to pass a parameter... otherwise just alias it directly. */
  usbd_poll(usbd_dev_handler);
}

static void gpio_config(void)
{
	GPIOB_CTL |= (0x01) << (11 * 2); 
	GPIOB_CTL &= ~(1 << 11);//PB11推挽输出

	GPIOA_CTL |= (0x01) << (8 * 2); 
	GPIOA_CTL &= ~(1 << 8);//PA8推挽输出	
	
	GPIOA_CTL |= (0x03) << (4 * 2); //PA4模拟模式
#if 0	
	GPIOA_CTL |= (0x02) << (6 * 2); //PA6 TIM2_CH1
	GPIOA_AFSEL0 |= (0x01) << (6 * 4); //PA6 AFIO 1
#else
	GPIOA_CTL &= ~((0x03 << (6 * 2)));
#endif
}


uint8_t framebuffer[1024]; //128*64 fb
uint16_t waveform[2][1048]; //4场扫描
volatile uint8_t read_buf = 0;
volatile uint8_t write_buf = 0;
volatile uint8_t req_rend = 0;
volatile uint8_t rend_done = 0;

static void dac_setup(void)
{
	/* Enable the DAC clock */
	rcc_periph_clock_enable(RCC_DAC);
	dac_trigger_enable(CHANNEL_1);
	dac_set_trigger_source(DAC_CR_TSEL1_T3);
	dac_dma_enable(CHANNEL_1);
	dac_enable(CHANNEL_1);
}

void dma1_channel2_3_isr(void)
{
	//if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL3, DMA_TCIF))
	//{
	if (DMA_ISR(DMA1) & (DMA_TCIF << DMA_FLAG_OFFSET(DMA_CHANNEL3)))
	{
		
#if 0			
			dma_disable_channel(DMA1,DMA_CHANNEL3);
	
			dma_set_priority(DMA1,DMA_CHANNEL3,DMA_CCR_PL_VERY_HIGH);
			dma_set_memory_size(DMA1,DMA_CHANNEL3,DMA_CCR_MSIZE_16BIT);
			dma_set_peripheral_size(DMA1,DMA_CHANNEL3,DMA_CCR_PSIZE_16BIT);
			dma_enable_memory_increment_mode(DMA1,DMA_CHANNEL3);
			dma_enable_circular_mode(DMA1,DMA_CHANNEL3);
			dma_set_read_from_memory(DMA1,DMA_CHANNEL3);
	
			dma_set_peripheral_address(DMA1,DMA_CHANNEL3,(uint32_t) &DAC_DHR12R1);
	
			dma_set_memory_address(DMA1,DMA_CHANNEL3,(uint32_t) waveform[read_buf]);
			dma_set_number_of_data(DMA1,DMA_CHANNEL3, 1024);
			dma_enable_channel(DMA1,DMA_CHANNEL3);
#endif
		//if(rend_done)
		//{			
			//read_buf = (read_buf == 1)? 0 : 1;
			//write_buf = (read_buf == 1)? 0 : 1;
			if(read_buf == 1)
				read_buf = 0;
			else
				read_buf = 1;
				
			//dma_disable_channel(DMA1,DMA_CHANNEL3);
			//DMA_CCR(DMA1, DMA_CHANNEL3) &= ~DMA_CCR_EN;
			//dma_set_memory_address(DMA1,DMA_CHANNEL3,(uint32_t) waveform[read_buf]);
			//DMA_CMAR(DMA1, DMA_CHANNEL3) = (uint32_t) waveform[read_buf];
			//dma_enable_channel(DMA1,DMA_CHANNEL3);
			//DMA_CCR(DMA1, DMA_CHANNEL3) |= DMA_CCR_EN;
			DAC_STATUS = 0xffffffff; //清除翻车位
			req_rend = 1;
			rend_done = 0;
		//}
		dma_clear_interrupt_flags(DMA1,DMA_CHANNEL3,DMA_TCIF);
			
	}
}

void sys_tick_handler(void)
{
	//if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL3, DMA_TCIF))
	//{
		
#if 0			
			dma_disable_channel(DMA1,DMA_CHANNEL3);
	
			dma_set_priority(DMA1,DMA_CHANNEL3,DMA_CCR_PL_VERY_HIGH);
			dma_set_memory_size(DMA1,DMA_CHANNEL3,DMA_CCR_MSIZE_16BIT);
			dma_set_peripheral_size(DMA1,DMA_CHANNEL3,DMA_CCR_PSIZE_16BIT);
			dma_enable_memory_increment_mode(DMA1,DMA_CHANNEL3);
			dma_enable_circular_mode(DMA1,DMA_CHANNEL3);
			dma_set_read_from_memory(DMA1,DMA_CHANNEL3);
	
			dma_set_peripheral_address(DMA1,DMA_CHANNEL3,(uint32_t) &DAC_DHR12R1);
	
			dma_set_memory_address(DMA1,DMA_CHANNEL3,(uint32_t) waveform[read_buf]);
			dma_set_number_of_data(DMA1,DMA_CHANNEL3, 1024);
			dma_enable_channel(DMA1,DMA_CHANNEL3);
#endif
		//if(rend_done)
		//{			
			//read_buf = (read_buf == 1)? 0 : 1;
			//write_buf = (read_buf == 1)? 0 : 1;
		//	if(rend_done)
		//		read_buf = write_buf;
				
			//dma_disable_channel(DMA1,DMA_CHANNEL3);
			//dma_set_memory_address(DMA1,DMA_CHANNEL3,(uint32_t) waveform[write_buf]);
			//dma_enable_channel(DMA1,DMA_CHANNEL3);
			DAC_STATUS = 0xffffffff; //清除翻车位
			req_rend = 1;
			rend_done = 0;
		//}
		dma_clear_interrupt_flags(DMA1,DMA_CHANNEL3,DMA_TCIF);
			
	//}
}



static void dma_setup(void)
{
/* DAC channel 1 shares DMA controller 1 Channel 3. */
/* Enable DMA1 clock and IRQ */
	rcc_periph_clock_enable(RCC_DMA1);
	dma_channel_reset(DMA1,DMA_CHANNEL3);
	
	dma_set_priority(DMA1,DMA_CHANNEL3,DMA_CCR_PL_VERY_HIGH);
	dma_set_memory_size(DMA1,DMA_CHANNEL3,DMA_CCR_MSIZE_16BIT);
	dma_set_peripheral_size(DMA1,DMA_CHANNEL3,DMA_CCR_PSIZE_16BIT);
	dma_enable_memory_increment_mode(DMA1,DMA_CHANNEL3);
	dma_enable_circular_mode(DMA1,DMA_CHANNEL3);
	dma_set_read_from_memory(DMA1,DMA_CHANNEL3);
	
	dma_set_peripheral_address(DMA1,DMA_CHANNEL3,(uint32_t) &DAC_DHR12R1);
	
	dma_set_memory_address(DMA1,DMA_CHANNEL3,(uint32_t) waveform[0]);
	dma_set_number_of_data(DMA1,DMA_CHANNEL3, 1024);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);
	dma_enable_channel(DMA1,DMA_CHANNEL3);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_IRQ);
}

static void timer_setup(void)
{
/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM3);
	timer_reset(TIM3);
/* Timer global mode: - No divider, Alignment edge, Direction up */
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM3);
	timer_set_period(TIM3, 7);
	timer_disable_oc_output(TIM3, TIM_OC2 | TIM_OC3 | TIM_OC4);
	timer_enable_oc_output(TIM3, TIM_OC1);
	timer_disable_oc_clear(TIM3, TIM_OC1);
	timer_disable_oc_preload(TIM3, TIM_OC1);
	timer_set_oc_slow_mode(TIM3, TIM_OC1);
	timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_TOGGLE);
	timer_set_oc_value(TIM3, TIM_OC1, 2);
	timer_disable_preload(TIM3);
/* Set the timer trigger output (for the DAC) to the channel 1 output compare */
	timer_set_master_mode(TIM3, TIM_CR2_MMS_COMPARE_OC1REF);
	timer_enable_counter(TIM3);
}

uint8_t rx_status = 0;
uint16_t write_pointer;

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;
	(void)usbd_dev;

	char buf[64];
	int i;
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);


	if (len) {
		for(i = 0; i < len; i++)
		{
			if(rx_status == 1)
			{
				framebuffer[write_pointer++] = buf[i];
				if(write_pointer >= 1024 && rx_status == 1)
				{	
					rx_status = 0;
					if(GPIOB_ODR & (1 << 11))
						GPIOB_BSRR = 1 << (11 + 16);
					else
					GPIOB_BSRR = 1 << 11;					
				}
				continue;
			}
			if(buf[i] == 'U' && rx_status == 0)
			{
				rx_status = 1;
				
				write_pointer = 0;
				continue;
			}
		}
	}
	
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;
	(void)usbd_dev;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
	configured = 1;
}


int main(void)
{
	int i;

	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	SCB_VTOR = 0x08000000;

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	gpio_config();

	usbd_dev_handler = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev_handler, cdcacm_set_config);
	
	GPIOA_BSRR = 1 << (8 + 16);

	for (i = 0; i < 0x80000; i++)
		__asm__("nop");

	for (i = 0; i < 1024; i++)
	{
		framebuffer[i] = 0xff;
	}


	nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ);
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	nvic_enable_irq(NVIC_USB_WAKEUP_IRQ);
#if 0
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8); /* 72MHz / 8 = 9MHz */
	/* 定时器每N次中断一次 */
	systick_set_reload(1100); /* 9000 / 9000 = 1kHz */
	systick_interrupt_enable();
	systick_counter_enable();
	nvic_set_priority(NVIC_SYSTICK_IRQ, 0);//systick最高

#endif
	nvic_set_priority(NVIC_DMA1_CHANNEL2_3_IRQ, 1);//DMA最高优先级
	nvic_set_priority(NVIC_USB_WAKEUP_IRQ, 1 << 4);
	nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1 << 4);
	nvic_set_priority(NVIC_USB_HP_CAN_TX_IRQ, 1 << 4);
	
	GPIOA_BSRR = 1 << (8);
	
	
	timer_setup();
	dma_setup();
	dac_setup();
	DAC_STATUS = 0xffffffff;

	__asm__("cpsie i"); //开中断
#if 0
	uint16_t x = 0, y = 0, scan = 0;
	uint16_t d = 0;
	for(scan = 0; scan < 8; scan++)
	{
		waveform[1][0 + 131 * scan] = 256;
		waveform[1][1 + 131 * scan] = 0;
		waveform[1][2 + 131 * scan] = 256;

		waveform[0][0 + 131 * scan] = 256;
		waveform[0][1 + 131 * scan] = 0;
		waveform[0][2 + 131 * scan] = 256;		
	}
#else
	uint16_t x = 0, y = 0, scan = 0;
	uint16_t d = 0;
	for(scan = 0; scan < 1024; scan++)
	{
		waveform[0][scan] = scan * 64;//% 16 * 255;
		waveform[1][scan] = 4095 - scan * 4;//% 16 * 255;
	}	
#endif
	
	dma_enable_channel(DMA1,DMA_CHANNEL3);
	DAC_STATUS = 0xffffffff;

	while (1)
	{
#if 0
		uint16_t sample;
		d = 0;
		if(req_rend == 1)
		{
			write_buf ++;
			if(write_buf > 1) write_buf = 0;

			for(scan = 0; scan < 1; scan++)
			{
				d += 3;
				for(x = 0; x < 128; x ++)
				{
				#if 1
					if(framebuffer[(y >> 3) * 128 + x] & (1 << (y & 0x07)))
						sample = 1536 - 20 * y;
					else
					{ //如果下行也白
						uint16_t yd = y > 64? 64: y + 1;
						uint16_t yp = y > 1? y - 1: 1;
						if(framebuffer[(yd >> 3) * 128 + x] & (1 << (yd & 0x07)))
							sample = 1536 - 20 * yd;
						else if (framebuffer[(yp >> 3) * 128 + x] & (1 << (yp & 0x07)))
							sample = 1536 - 20 * yp;
						else
							if(y < 32)
								sample = 1536;
							else
								sample = 256;
					}

					waveform[write_buf][d++] = sample;
				#endif
					//waveform[write_buf][d++] = 2048;
				
				}
				y ++;
				if(y > 63)
					y = 0;

			}
			req_rend = 0;
			rend_done = 1;
		}
		/*
		
		if(dma_get_interrupt_flag(DMA1, DMA_CHANNEL3, DMA_TCIF))
		{
			dma_clear_interrupt_flags(DMA1,DMA_CHANNEL3,DMA_TCIF);
			
			dma_disable_channel(DMA1,DMA_CHANNEL3);
			if(write_buf == read_buf)
				dma_set_memory_address(DMA1,DMA_CHANNEL3,(uint32_t) waveform[read_buf]);
			dma_enable_channel(DMA1,DMA_CHANNEL3);
			DAC_STATUS = 0xffffffff;
			
			if(write_buf == read_buf)
			{				
				read_buf++;
				if(read_buf > 1) read_buf = 0;
			}
		}
		*/
		
		
		
#endif

	}
}
