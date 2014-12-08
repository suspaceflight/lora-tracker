#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/nvic.h>
#include "radio.h"
#include <stdio.h>

//#define ENABLE_GPS
#define LORA_RX
//#define UPLINK

void init(void);
void _delay_ms(const uint32_t delay);
void uart_send_blocking_len(uint8_t *buff, uint16_t len);
uint16_t calculate_crc16 (char *input);


char buff[128] = {0};

char gnss_buff[255] = {0};

//len = 44
const uint8_t flight_mode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
		0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
		0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4D, 0xDB};

//len = 16
const uint8_t disable_nmea_gpgga[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};
const uint8_t disable_nmea_gpgll[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
const uint8_t disable_nmea_gpgsa[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
const uint8_t disable_nmea_gpgsv[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
const uint8_t disable_nmea_gprmc[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F};
const uint8_t disable_nmea_gpvtg[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};

//len = 16
const uint8_t enable_navpvt[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00,
		0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1};


volatile uint16_t gnss_string_count = 0; //0 - not in string, >=1 - in string
volatile uint16_t gnss_string_len = 0;
volatile uint16_t gnss_message_id = 0;
volatile char* gnss_buff_ptr = &gnss_buff[0];

volatile uint8_t time_updated = 0;
volatile uint8_t pos_updated = 0;
volatile uint8_t gnss_status_updated = 0;

volatile uint8_t fixtype = 0;
volatile int32_t latitude = 0;
volatile int32_t longitude = 0;
volatile int32_t altitude = 0;
volatile uint8_t hour = 0;
volatile uint8_t minute = 0;
volatile uint8_t second = 0;
volatile uint8_t sats = 0;
volatile uint8_t pos_valid = 0;
volatile uint8_t time_valid = 0;

uint16_t payload_counter = 0;
uint16_t uplink_counter = 0;


void init_wdt(void)
{
	RCC_CSR |= 1;   //LSI on
	while(RCC_CSR&(1<<1));  //wait for LSI ready

	while(IWDG_SR&1);
	IWDG_KR = 0x5555;
	IWDG_PR = 0b110; // 40kHz/256

	while(IWDG_SR&2);
	IWDG_KR = 0x5555;
	IWDG_RLR = 2600;

	IWDG_KR = 0xAAAA;
	IWDG_KR = 0xCCCC;

	/*
	///// Configuring the IWDG when the window option is disabled
	//1. Enable register access by writing 0x0000 5555 in the IWDG_KR register.
	IWDG_KR = 0x5555;
	//2. Write the IWDG prescaler by programming IWDG_PR from 0 to 7.
	IWDG_PR = 0b110; // 40kHz/256
	//3. Write the reload register (IWDG_RLR).
	IWDG_RLR = 1600;
	//4. Wait for the registers to be updated (IWDG_SR = 0x0000 0000).
	while(IWDG_SR&1)
	{
		int t = IWDG_SR;
		t++;
	}
	//5. Refresh the counter value with IWDG_RLR (IWDG_KR = 0x0000 AAAA).
	IWDG_KR = 0xAAAA;
	//6. Enable the IWDG by writing 0x0000 CCCC in the IWDG_KR.
	IWDG_KR = 0xCCCC; */
}

void init (void)
{
	rcc_clock_setup_in_hsi_out_8mhz();
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);

/*
	//adc
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
	uint8_t channel_array[] = { ADC_CHANNEL1};
	adc_power_off(ADC1);
	adc_calibrate_start(ADC1);
	adc_calibrate_wait_finish(ADC1);
	//adc_set_operation_mode(ADC1, ADC_MODE_SCAN); //adc_set_operation_mode(ADC1, ADC_MODE_SCAN_INFINITE);
	adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
//	adc_set_single_conversion_mode(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_013DOT5);
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
	//adc_set_single_conversion_mode(ADC1);
	adc_disable_analog_watchdog(ADC1);
	adc_power_on(ADC1); */


	//uart
	//nvic_enable_irq(NVIC_USART1_IRQ);
	rcc_periph_clock_enable(RCC_USART1);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9|GPIO10);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO9|GPIO10);
	usart_set_baudrate(USART1, 9600 );
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);
/*
	adc_start_conversion_regular(ADC1);

	_delay_ms(200);
	uart_send_blocking_len((uint8_t*)flight_mode,44);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgga,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgll,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgsa,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgsv,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gprmc,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpvtg,16);
	uart_send_blocking_len((uint8_t*)enable_navpvt,16); */

	//used to hiz the uart so the pc can query flight mode
	//gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
	//			GPIO2 | GPIO3);
}

void uart_send_blocking_len(uint8_t *buff, uint16_t len)
{
	uint16_t i = 0;
	for (i = 0; i < len; i++)
		usart_send_blocking(USART1,*buff++);

}


int main(void)
{

	init();
//	init_wdt();

 	radio_lora_settings_t s_lora;
 	s_lora.spreading_factor = 12;
	s_lora.bandwidth = BANDWIDTH_20_8K;
	s_lora.coding_rate = CODING_4_5;
	s_lora.implicit_mode = 0;
	s_lora.crc_en = 1;
	s_lora.low_datarate = 1;

	_delay_ms(100);

 	radio_init();


#ifdef LORA_RX
 	radio_write_lora_config(&s_lora);
 	radio_pa_off();
	radio_lna_max();
	radio_set_frequency(FREQ_434_100);
	radio_set_continuous_rx();
	int i;
	int j=0;
	while(1)
	{
		uint8_t stat = radio_read_single_reg(REG_MODEM_STAT);
		uint8_t irq = radio_read_single_reg(REG_IRQ_FLAGS);
		uint8_t nb = radio_read_single_reg(REG_RX_NB_BYTES);
		uint8_t hrx = radio_read_single_reg(REG_RX_HEADER_CNT_VALUE_LSB);

		snprintf(buff,60,"stat: %X  irq: %X headers rx: %X nBytes: %d\r\n",stat,irq,hrx,nb);
		i=0;
		//while (buff[i])
		//	usart_send_blocking(USART1, buff[i++]);

		if (irq & (1<<6))
		{
			int16_t r = radio_check_read_rx_packet(128,(uint8_t*)buff,1);

			if (r > 0)
			{
				for (i = 0; i < r; i++)
					usart_send_blocking(USART1, buff[i]);
				int16_t snr = radio_read_single_reg(REG_PKT_SNR_VALUE);
				if (snr & 0x80)
					snr |= 0xFF00;
				int16_t rssi = radio_read_single_reg(REG_PKT_RSSI_VALUE)-164;
				int32_t error = radio_read_single_reg(REG_FEI_MSB_LORA) << 8;
				error = (error | radio_read_single_reg(REG_FEI_MID_LORA)) << 8;
				error |= radio_read_single_reg(REG_FEI_LSB_LORA);
				r = snprintf(buff,60,"snr: %i  rssi: %i offset: %li     \r\n",snr,rssi,error);
				i=0;
				while (buff[i])
					usart_send_blocking(USART1, buff[i++]);
			}
			else
			{
				snprintf(buff,60,"CRC ERROR\r\n");
				i=0;
				while (buff[i])
					usart_send_blocking(USART1, buff[i++]);
			}
			radio_write_single_reg(REG_IRQ_FLAGS,0xFF);
			if (0)//1)//(j++ &0x3) == 0x3)
			{

				radio_sleep();
				_delay_ms(10);
				radio_set_frequency(FREQ_434_100);
				radio_write_lora_config(&s_lora);
				radio_standby();
				radio_high_power();
				i=snprintf(buff,60,"PINGPINGPING");

				radio_tx_packet(buff,i);

				_delay_ms(200);
				while(lora_in_progress())
					_delay_ms(50);

				radio_standby();
				radio_pa_off();
				radio_lna_max();
				radio_set_frequency(FREQ_434_100);
				radio_set_continuous_rx();


			}

		}

		_delay_ms(250);
	}


#endif

	radio_high_power();
		radio_set_frequency(FREQ_434_100);

	while(1){
		int k=snprintf(buff,105,"xxxxx$$PAYLOAD,TESTTESTTESTESTSTST,");



		if (1)//(payload_counter & 0x3) == 0x3)  //rtty
		{
			radio_sleep();
			_delay_ms(10);
			radio_high_power();
			radio_start_tx_rtty((char*)buff,BAUD_50,4);
			while(rtty_in_progress() != 0){
				radio_rtty_poll_buffer_refill();
				_delay_ms(20);
			}
			_delay_ms(100);
			radio_sleep();
		}
		else   //lora
		{
			radio_sleep();
			_delay_ms(10);
			radio_write_lora_config(&s_lora);

			radio_standby();
			radio_high_power();
			radio_set_frequency(FREQ_434_100);



			radio_tx_packet(&buff[5],k-5);
			//_delay_ms(12500);


			_delay_ms(200);


			while(lora_in_progress())
				_delay_ms(50);


		}
	}


 	
}

uint16_t crc_xmodem_update (uint16_t crc, uint8_t data)
{
	int i;

	crc = crc ^ ((uint16_t)data << 8);
	for (i=0; i<8; i++)
	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	}

	return crc;
}

uint16_t calculate_crc16 (char *input)
{
	uint16_t crc;
	crc = 0xFFFF;

	while (*input)
	{
		crc = crc_xmodem_update(crc, *input);
		input++;
	}

	return crc;
}

void _delay_ms(const uint32_t delay)
{
    uint32_t i, j;

    for(i=0; i< delay; i++)
        for(j=0; j<1000; j++)
            __asm__("nop");
}
