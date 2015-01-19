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

uint8_t buff[128] = "HELLOOOOOOo";

void init (void)
{
	rcc_clock_setup_in_hsi_out_8mhz();
	rcc_periph_clock_enable(RCC_GPIOB);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);



}

int main(void)
{

	init();

	radio_lora_settings_t s;
	s.spreading_factor = 12;
	s.bandwidth = BANDWIDTH_20_8K;
	s.coding_rate = CODING_4_5;
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

    	uint8_t l = snprintf(buff,30,"HELLOsfsfd: %d     \r\n",i);
    	radio_tx_packet(buff,l);
    	_delay_ms(6500);
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