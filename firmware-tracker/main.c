#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void init(void);
static void my_usart_print_int(uint32_t usart, int32_t value);
void _delay_ms(const uint32_t delay);




int main(void)
{
	init();

	while(1)
	{
		_delay_ms(500);
		GPIOB_BSRR = (1<<1);
		_delay_ms(500);
		GPIOB_BRR = (1<<1);
		
	}
}






void init(void)
{

	rcc_clock_setup_in_hsi_out_48mhz();

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
	//gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, 0xFF);

	



	_delay_ms(100);


/*
	//dma
	rcc_periph_clock_enable(RCC_DMA);
	dma_channel_reset(DMA1,1);
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	dma_set_peripheral_address(DMA1,1,(0x40000000U) + 0x12400 + 0x40);
	dma_set_memory_address(DMA1,1,(uint32_t)adcout);
	dma_set_number_of_data(DMA1,1,16);
	dma_enable_circular_mode(DMA1,1);
	dma_set_read_from_peripheral(DMA1,1);
	dma_enable_memory_increment_mode(DMA1,1);
	dma_set_peripheral_size(DMA1,1,DMA_CCR_PSIZE_16BIT);
	dma_set_memory_size(DMA1,1,DMA_CCR_MSIZE_16BIT);
	dma_set_priority(DMA1,1,DMA_CCR_PL_HIGH);
	dma_enable_half_transfer_interrupt(DMA1,1);
	dma_enable_transfer_complete_interrupt(DMA1,1);



	//start adc and dma
	ADC_CR(ADC1) |= ADC_CR_ADSTART;
	dma_enable_channel(DMA1,1);



	//usart
	rcc_periph_clock_enable(RCC_USART1);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2|GPIO3);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO2|GPIO3);
	usart_set_baudrate(USART1, 9600 );
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);
*/
}



//from libopencm3 example
static void my_usart_print_int(uint32_t usart, int32_t value)
{
	int8_t i;
	int8_t nr_digits = 0;
	char buffer[25];

	if (value < 0) {
		usart_send_blocking(usart, '-');
		value = value * -1;
	}

	if (value == 0) {
		usart_send_blocking(usart, '0');
	}

	while (value > 0) {
		buffer[nr_digits++] = "0123456789"[value % 10];
		value /= 10;
	}

	for (i = nr_digits-1; i >= 0; i--) {
		usart_send_blocking(usart, buffer[i]);
	}

	usart_send_blocking(usart, ',');
	//usart_send_blocking(usart, '\n');
}


/**
 * Approximately delay for the given time period
 * @param delay The number of milliseconds to delay
 */
void _delay_ms(const uint32_t delay)
{
    uint32_t i, j;

    for(i=0; i< delay; i++)
        for(j=0; j<6000; j++)
            __asm__("nop");
}

