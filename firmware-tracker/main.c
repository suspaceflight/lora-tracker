#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include "radio.h"


void init(void);
void _delay_ms(const uint32_t delay);

void init (void)
{
	rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOB);

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);

	radio_init();

}

int main(void)
{

	init();

    while(1)
    {
    	GPIOB_ODR = 0;
    	uint8_t v = radio_read_version();
    	_delay_ms(500);
    	GPIOB_ODR = (1<<1);
    	_delay_ms(500);

    }
}


void _delay_ms(const uint32_t delay)
{
    uint32_t i, j;

    for(i=0; i< delay; i++)
        for(j=0; j<1000; j++)
            __asm__("nop");
}
