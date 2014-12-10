#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include "screen.h"


void screen_init(void)
{

	rcc_periph_clock_enable(RCC_S_I2C);
	rcc_periph_clock_enable(RCC_BACKLIGHT_PORT);
	rcc_periph_clock_enable(RCC_RESET_PORT);

	gpio_mode_setup(S_I2C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
			S_I2C_SCL_PIN|S_I2C_SDA_PIN);
	gpio_set_af(S_I2C_PORT, S_I2C_AFn, S_I2C_SCL_PIN|S_I2C_SDA_PIN);
	gpio_mode_setup(BACKLIGHT_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,BACKLIGHT_PIN);
	gpio_mode_setup(S_RESET_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,S_RESET_PIN);

	gpio_set(BACKLIGHT_PORT,BACKLIGHT_PIN);
	gpio_set(S_RESET_PORT,S_RESET_PIN);

	//i2c_reset(S_I2C);
	//i2c_peripheral_disable(S_I2C);
	//i2c_enable_analog_filter(S_I2C);
	//i2c_set_digital_filter(S_I2C, I2C_CR1_DNF_DISABLED);

	//i2c_set_clock_frequency(S_I2C,I2C_CR2_FREQ_8MHZ );
	//i2c_set_ccr(S_I2C,)



	I2C1_CR1 = 0;   //turns a bunch of stuff off, currently disabl3ed

	//prescaler = 1, low period = 0x13, high period = 0xf, hold time = 2, setup = 4
	I2C1_TIMINGR = (1<<28) | (4<<20) | (2<<16) | (0xf<<8) | 0x13;


	I2C1_CR1 |= I2C_CR1_PE;   //enable



}

void screen_test(void)
{
	const uint8_t contrast = 32;
	const uint8_t rab = 5;

	uint8_t buff[20];
	buff[0] =  (S_CMDSEND | S_MULTIPLE);
	buff[1] = (S_SET_CGRAM | 2);
	buff[2] = (S_DATASEND | S_MULTIPLE);
	buff[3] = (0x1F);
	buff[4] = (S_DATASEND | S_MULTIPLE);
	buff[5] = (0x1F);
	buff[6] = (S_DATASEND | S_MULTIPLE);
	buff[7] = (0x1F);
	buff[8] = (S_DATASEND | S_MULTIPLE);
	buff[9] = (0x1F);
	buff[10] = (S_CMDSEND);

	buff[11] = (S_INSTR_TABLE | S_IS1);
	buff[12] = (S_INT_OSC | S_F_183HZ | S_BIAS_5);
	buff[13] = (S_CONTRAST | (contrast & 0x0F) );
	buff[14] = (S_POWER | S_ICON_ON | S_BOOST_ON | ((contrast & 0x30) >> 4));
	buff[15] = (S_FOLLOWER | S_FON | (rab & 0x07));
	buff[16] = (S_INSTR_TABLE | S_IS0);
	buff[17] = (S_LCD_MODE | S_LCD_ON);

	screen_write_i2c(I2C1, 0x7c, 18,buff);

}

uint8_t i2c_busy(uint32_t i2c)
{
	if ((I2C_ISR(i2c) & I2C_ISR_BUSY) != 0) {
		return 1;
	}

	return 0;
}
void i2c_set_bytes_to_transfer(uint32_t i2c, uint32_t n_bytes)
{
	I2C_CR2(i2c) = (I2C_CR2(i2c) & ~(0xFF<<16)) |
		       (n_bytes << I2C_CR2_NBYTES_SHIFT);
}
void i2c_set_7bit_address(uint32_t i2c, uint8_t addr)
{
	I2C_CR2(i2c) = (I2C_CR2(i2c) & ~(0x3FF)) |
		       ((addr & 0x7F));// << 1);
}
uint8_t i2c_transmit_int_status(uint32_t i2c)
{
	if ((I2C_ISR(i2c) & I2C_ISR_TXIS) != 0) {
		return 1;
	}

	return 0;
}
uint8_t i2c_nack(uint32_t i2c)
{
	if ((I2C_ISR(i2c) & I2C_ISR_NACKF) != 0) {
		return 1;
	}

	return 0;
}

//from libopencm3
void screen_write_i2c(uint32_t i2c, uint8_t i2c_addr, uint8_t size,
	       uint8_t *data)
{
	int wait;
	int i;
	while (i2c_busy(i2c) == 1);
	while ((I2C_CR2(i2c) & I2C_CR2_START)); //(i2c_is_start(i2c) == 1);
	/*Setting transfer properties*/
	i2c_set_bytes_to_transfer(i2c, size);
	i2c_set_7bit_address(i2c, (i2c_addr & 0x7F));
	I2C_CR2(i2c) &= ~I2C_CR2_RD_WRN; //i2c_set_write_transfer_dir(i2c);
	I2C_CR2(i2c) |= I2C_CR2_AUTOEND; //i2c_enable_autoend(i2c);
	/*start transfer*/
	I2C_CR2(i2c) |= I2C_CR2_START; //i2c_send_start(i2c);


	for (i = 0; i < size; i++) {
		wait = true;
		while (wait) {
			if (i2c_transmit_int_status(i2c)) {
				wait = false;
			}
			while (i2c_nack(i2c));
		}
		I2C_TXDR(i2c) = data[i]; //i2c_send_data(i2c, data[i]);
	}
}

void backlight_on(void)
{
	gpio_clear(BACKLIGHT_PORT,BACKLIGHT_PIN);
}

void backlight_off(void)
{
	gpio_set(BACKLIGHT_PORT,BACKLIGHT_PIN);
}

