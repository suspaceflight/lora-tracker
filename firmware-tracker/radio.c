#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>


#include "radio.h"

void radio_init(void)
{
	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
	            GPIO5 | GPIO6 | GPIO7);
	gpio_set_af(GPIOA, GPIO_AF0, GPIO5 | GPIO6 | GPIO7);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		            GPIO4);

	// Reset and enable the SPI periph
	spi_reset(R_SPI);
	spi_init_master(R_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_64,
			SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_1,
			SPI_CR1_CRCL_8BIT,
			SPI_CR1_MSBFIRST);

	// Trigger an RXNE event when we have 8 bits (one byte) in the buffer
	spi_fifo_reception_threshold_8bit(R_SPI);

	// NSS must be set high for the peripheral to function
	spi_enable_software_slave_management(R_SPI);
	spi_set_nss_high(R_SPI);
	gpio_set(GPIOA, GPIO4);

	// Enable
	spi_enable(R_SPI);


}

uint8_t radio_read_version(void)
{
	return radio_read_single_reg(REG_VERSION);
}

uint8_t radio_read_single_reg(uint8_t reg)
{
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	gpio_clear(GPIOA, GPIO4);
	spi_send8(R_SPI, reg);
	spi_read8(R_SPI);
	// Wait until send FIFO is empty
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	spi_send8(R_SPI, 0x0);
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	uint8_t out = spi_read8(R_SPI);
	gpio_set(GPIOA, GPIO4);
	return out;
}

void radio_write_single_reg(uint8_t reg, uint8_t data)
{
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	gpio_clear(GPIOA, GPIO4);
	spi_send8(R_SPI, reg);
	spi_read8(R_SPI);
	// Wait until send FIFO is empty
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	spi_send8(R_SPI, data);
	spi_read8(R_SPI);
	// Wait until send FIFO is empty
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	gpio_set(GPIOA, GPIO4);
}

