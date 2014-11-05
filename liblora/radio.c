#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include "radio_config.h"


#include "radio.h"

inline uint16_t min(uint16_t in1, uint16_t in2)
{
	if (in1 > in2)
		return in2;
	else
		return in1;
}

//will change mode to standby if not already in standby or sleep
//if not in lora, will change to lora mode
void radio_write_lora_config(radio_lora_settings_t *s)
{
	uint8_t mode = radio_read_single_reg(REG_OP_MODE);
	mode |= (1<<3); //set LF range

	//check in lora mode
	if ((mode & (1<<7)) == 0)
		radio_write_single_reg(REG_OP_MODE,mode & ~(uint8_t)7);		//set to sleep mode so lora bit can be written
	else{
		//check in right mode to change lora registers
		if (!(((mode&MODE_MASK) == MODE_SLEEP) | ((mode&MODE_MASK) == MODE_STNDBY)))
		{
			radio_write_single_reg(REG_OP_MODE,(mode & ~(uint8_t)7) | 1);  //put into standby
		}
	}

	//put into lora mode
	mode = radio_read_single_reg(REG_OP_MODE);
	radio_write_single_reg(REG_OP_MODE,mode | (1<<7));

	//write modem config
	radio_write_single_reg(REG_MODEM_CONFIG1,
			(s->bandwidth&0xF0) | (s->coding_rate&0xE) | (s->implicit_mode&1));
	radio_write_single_reg(REG_MODEM_CONFIG2,
			((s->spreading_factor&0xF)<<4) | ((s->crc_en&0x1)<<2) | (s->implicit_mode&1));

	mode = radio_read_single_reg(REG_MODEM_CONFIG3);
	mode = mode & ~(1<<3);
	radio_write_single_reg(REG_MODEM_CONFIG3, mode | ((s->low_datarate&0x1)<<3) | (1<<2));  //also turn on auto lna

	if (s->spreading_factor == 6)
	{

		//-Set SpreadingFactor = 6 in RegModemConfig2
		//- The header must be set to Implicit mode.
		//- Set the bit field DetectionOptimize of register RegLoRaDetectOptimize to value "0b101".
		//- Write 0x0C in the register RegDetectionThreshold.
	}
	else
	{
		//unset the above
	}

	//LNA?

}

void radio_set_preamble(uint16_t pre)
{

}

void radio_high_power(void)
{
	radio_write_single_reg(REG_PA_CONFIG,POWER_HIGH);
}

uint8_t radio_get_status(void)
{
	return radio_read_single_reg(REG_OP_MODE);
}

void radio_set_frequency(uint32_t f)
{
	radio_write_single_reg(REG_FRF_LSB,f & 0xFF);
	f >>= 8;
	radio_write_single_reg(REG_FRF_MID,f & 0xFF);
	f >>= 8;
	radio_write_single_reg(REG_FRF_MSB,f & 0xFF);
}

void radio_write_fsk_config(radio_fsk_settings_t s)
{


}

void radio_set_slow_packet_time(void)
{


}

void radio_set_explicit_rx_mode(uint8_t len, uint8_t rate, uint8_t crc_en)
{

}

void radio_set_implicit_mode(void)
{

}


void radio_tx_packet(uint8_t *data, uint16_t len)
{

	uint8_t r = radio_read_single_reg(REG_FIFO_TX_BASE_ADDR);
	radio_write_single_reg(REG_FIFO_ADDR_PTR,r);
	radio_write_burst_reg(0,data,len);
	radio_write_single_reg(REG_PAYLOAD_LENGTH_LORA,len);

	//write mode tx
	r = radio_read_single_reg(REG_OP_MODE) & 0xF8;
	radio_write_single_reg(REG_OP_MODE, r | MODE_TX);

}

void radio_carrier_on(void)
{

}

void radio_sleep(void)
{

}


void radio_init()
{
	rcc_periph_clock_enable(R_RCC_SPI);
	rcc_periph_clock_enable(R_RCC_GPIO);
	gpio_mode_setup(R_SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
			R_SPI_PINS);
	gpio_set_af(R_SPI_PORT, R_SPI_AFn, R_SPI_PINS);
	gpio_mode_setup(R_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
			R_CS_PIN);

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
	gpio_set(R_CS_PORT, R_CS_PIN);

	// Enable
	spi_enable(R_SPI);


}

uint8_t radio_read_version(void)
{
	return radio_read_single_reg(REG_VERSION);
}

void radio_set_continuous_rx(void)
{
	//go into rx mode
	uint8_t r = radio_read_single_reg(REG_OP_MODE) & 0xF8;
	radio_write_single_reg(REG_OP_MODE, r | MODE_RX);

}
/*
uint8_t radio_new_data(void)
{
	//check for RxDone
	if (radio_read_single_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE)
		return 1;
	else
		return 0;
	//also payloadcrcerror
}

void radio_clear_irq(void)
{
	radio_write_single_reg(REG_IRQ_FLAGS, 0xFF);
}
*/
int16_t radio_check_read_rx_packet(uint16_t max_len, uint8_t *buff, uint8_t check_crc)
{
	//check rx timeout, validheader
	uint8_t f = radio_read_single_reg(REG_IRQ_FLAGS);
	//clear flags
	radio_write_single_reg(REG_IRQ_FLAGS, 0xFF);

	if ((f & IRQ_RX_DONE) == 0)
		return 0;

	if ((f & IRQ_CRC_ERROR) && check_crc)
		return -2;


	//RxNbBytes - number of bytes received
	uint16_t c = radio_read_single_reg(REG_RX_NB_BYTES);

	//FifoAddrPtr - where the modem has written up to
	//set FifoAddrPtr to FifoRxCurrentAddr
	f = radio_read_single_reg(REG_FIFO_RX_CURRENT_ADDR);
	radio_write_single_reg(REG_FIFO_ADDR_PTR,f);

	//read Fifo RxNbTimes
	radio_read_burst_reg(REG_FIFO, buff, min(max_len,c));

	return min(max_len,c);
}

uint8_t radio_read_single_reg(uint8_t reg)
{
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	gpio_clear(R_CS_PORT, R_CS_PIN);
	spi_send8(R_SPI, reg & 0x7F);
	spi_read8(R_SPI);
	// Wait until send FIFO is empty
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	spi_send8(R_SPI, 0x0);
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	uint8_t out = spi_read8(R_SPI);
	gpio_set(R_CS_PORT, R_CS_PIN);
	return out;
}

void radio_read_burst_reg(uint8_t reg, uint8_t *buff, uint16_t len)
{
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	gpio_clear(R_CS_PORT, R_CS_PIN);
	spi_send8(R_SPI, reg & 0x7F);
	spi_read8(R_SPI);
	// Wait until send FIFO is empty
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	uint16_t i;
	for ( i = 0; i < len; i++)
	{
		spi_send8(R_SPI, 0x0);
		while(SPI_SR(R_SPI) & SPI_SR_BSY);
		buff[i] = spi_read8(R_SPI);
	}
	gpio_set(R_CS_PORT, R_CS_PIN);
}

void radio_write_single_reg(uint8_t reg, uint8_t data)
{
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	gpio_clear(R_CS_PORT, R_CS_PIN);
	spi_send8(R_SPI, reg | (1<<7));
	spi_read8(R_SPI);
	// Wait until send FIFO is empty
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	spi_send8(R_SPI, data);
	spi_read8(R_SPI);
	// Wait until send FIFO is empty
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	gpio_set(R_CS_PORT, R_CS_PIN);
}

void radio_write_burst_reg(uint8_t reg, uint8_t *data, uint16_t len)
{
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	gpio_clear(R_CS_PORT, R_CS_PIN);
	spi_send8(R_SPI, reg | (1<<7));
	spi_read8(R_SPI);
	// Wait until send FIFO is empty
	uint16_t i = 0;
	for (i = 0; i < len; i++)
	{
		while(SPI_SR(R_SPI) & SPI_SR_BSY);
		spi_send8(R_SPI, data[i]);
		spi_read8(R_SPI);
	}
	// Wait until send FIFO is empty
	while(SPI_SR(R_SPI) & SPI_SR_BSY);
	gpio_set(R_CS_PORT, R_CS_PIN);
}

