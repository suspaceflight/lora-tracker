#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include "radio.h"
#include <stdio.h>


void init(void);
void _delay_ms(const uint32_t delay);

uint8_t buff[128] = {0};

void init (void)
{
	rcc_clock_setup_in_hsi_out_8mhz();
	rcc_periph_clock_enable(RCC_GPIOB);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);


	//uart
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO2 | GPIO3);

	usart_set_parity(USART1 ,USART_PARITY_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX );
	usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_databits(USART1, 8);
	usart_set_baudrate(USART1, 9600);
	usart_enable_rx_interrupt(USART1);
	usart_enable(USART1);


}

void usart1_isr(void)
{
	if (((USART_ISR(USART1) & USART_ISR_RXNE) != 0))
	{

		uint8_t data = USART1_RDR;
	}


}

int main(void)
{

	init();


	radio_fsk_settings_t s1;
	s1.freq_dev = 3;
 	s1.bitrate = 0xffff;//BITRATE_500;
 	s1.enable_sync = 0;
 	s1.preamble_size = 0;
 	s1.enable_crc = 0;

 	radio_init();
	//radio_write_fsk_config(&s1);
	radio_high_power();
	radio_set_frequency(FREQ_434_100);
	//radio_fsk_set_fifo_threshold(20);

	_delay_ms(100);

	snprintf(buff,60,"xxxxxHELLOsygygghhghghghghgghghfsfd: %d     \r\n",7);
	buff[0] = 0xFF;
	buff[1] = 0xFF;
	buff[2] = 0xFF;
	buff[3] = 0x80;
	buff[4] = 0x80;

	while(1){


		radio_start_tx_rtty((char*)buff,BAUD_50,4);

		while(rtty_in_progress() != 0){
			radio_rtty_poll_buffer_refill();
		}

		usart_send_blocking(USART1,'G');


		_delay_ms(1000);
	}

	uint8_t r = radio_read_single_reg(REG_OP_MODE) & 0xF8;
	radio_write_single_reg(REG_OP_MODE, r | MODE_TX);






	buff[6] = 0xff;
	buff[8] = 0xff;
	buff[10] = 0xff;
	buff[11] = 0xff;
	buff[12] = 0xff;
	buff[13] = 0xff;
	buff[15] = 0xff;
	buff[17] = 0xff;
	buff[18] = 0xff;
	buff[20] = 0xff;


	/*
	buff[11] = 0xff;
	buff[13] = 0xff;
	buff[14] = 0xff;
	buff[15] = 0xff;

	buff[20] = 0xff;
	buff[21] = 0xff;
	buff[22] = 0xff;
*/
	buff[28] = 0xff;
	buff[29] = 0xff;
	buff[30] = 0xff;



   	radio_write_burst_reg(0,buff,40);



   	while(1){
		if(radio_fsk_poll_fifo_level() == 0){
			GPIOB_ODR = 0;
			radio_write_burst_reg(0,buff,40);
			GPIOB_ODR = (1<<1);
		}
   	}



	while(1)
	{
		radio_write_burst_reg(0,buff,40);
		_delay_ms(500);
		radio_write_burst_reg(0,buff,40);
		_delay_ms(500);
	}


	while(1);

	///////////////////

	radio_lora_settings_t s;
	s.spreading_factor = 12;
	s.bandwidth = BANDWIDTH_20_8K;
	s.coding_rate = CODING_4_8;
	s.implicit_mode = 0;
	s.crc_en = 1;
	s.low_datarate = 1;

	_delay_ms(100);

	radio_init();
	radio_write_lora_config(&s);
	radio_high_power();
	radio_set_frequency(FREQ_434_100);

	uint16_t i = 100;
    while(1)
    {
    	GPIOB_ODR = 0;
    	uint8_t v = radio_read_version();

    	int8_t l = snprintf(buff,60,"HELLOsygygghhghghghghgghghfsfd: %d     \r\n",i);
    	radio_tx_packet(buff,l);
    	_delay_ms(12500);
    	GPIOB_ODR = (1<<1);
    	_delay_ms(500);
    	i++;

    }
}


void _delay_ms(const uint32_t delay)
{
    uint32_t i, j;

    for(i=0; i< delay; i++)
        for(j=0; j<1000; j++)
            __asm__("nop");
}
