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
	adc_power_on(ADC1);


	//uart
	nvic_enable_irq(NVIC_USART1_IRQ);
	rcc_periph_clock_enable(RCC_USART1);
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

	adc_start_conversion_regular(ADC1);

	_delay_ms(200);
	uart_send_blocking_len((uint8_t*)flight_mode,44);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgga,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgll,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgsa,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgsv,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gprmc,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpvtg,16);
	uart_send_blocking_len((uint8_t*)enable_navpvt,16);

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

void usart1_isr(void)
{
	if (((USART_ISR(USART1) & USART_ISR_RXNE) != 0))
	{

		uint8_t d = (uint8_t)USART1_RDR;


		if (gnss_string_count == 0){ //look for '0xB5'
			if (d == 0xB5)
				gnss_string_count++;
		}
		else if (gnss_string_count == 1){ //look for '0x62'
			if (d == 0x62)
				gnss_string_count++;
			else
				gnss_string_count = 0;
		}
		else if (gnss_string_count == 2){  //message id
			gnss_message_id = d << 8;
			gnss_string_count++;
		}
		else if (gnss_string_count == 3){  //message id
			gnss_message_id |= d;
			gnss_string_count++;
		}
		else if (gnss_string_count == 4){  //length top byte
			gnss_string_len = d;
			gnss_string_count++;
		}
		else if (gnss_string_count == 5){  //length bottom byte
			gnss_string_len |= d << 8;
			gnss_string_count++;
			gnss_buff_ptr = &gnss_buff[0];
		}
		else if (gnss_string_count > 0){  //process payload + checksum

			if (gnss_string_len < 255)
				*gnss_buff_ptr++ = d;

			if (gnss_string_count >= 256)
				gnss_string_count = 0;   //something probably broke

			gnss_string_count++;
			if (gnss_string_count-6+2 == gnss_string_len) //got all bytes, check checksum
			{
				//lets assume checksum == :)

				if (gnss_message_id == 0x0107 && gnss_string_len == 92)  //navpvt
				{
					fixtype = gnss_buff[20];
					if (fixtype == 2 || fixtype == 3){
						latitude = (gnss_buff[31] << 24)
								 | (gnss_buff[30] << 16)
								 | (gnss_buff[29] << 8)
								 | (gnss_buff[28]);
						longitude = (gnss_buff[27] << 24)
								 | (gnss_buff[26] << 16)
								 | (gnss_buff[25] << 8)
								 | (gnss_buff[24]);
						altitude = (gnss_buff[39] << 24)
								 | (gnss_buff[38] << 16)
								 | (gnss_buff[37] << 8)
								 | (gnss_buff[36]);
						pos_valid |= 1;
					}


					sats = gnss_buff[23];

					uint8_t valid = gnss_buff[11];  //valid time flags
					if (valid & (1<<1))
					{
						hour = gnss_buff[8];
						minute = gnss_buff[9];
						second = gnss_buff[10];
						time_valid |= 1;
					}

					gnss_status_updated = 1;
					time_updated = 1;
					pos_updated = 1;
				}
				gnss_string_count = 0;  //wait for the next string
			}
		}
	}
	else if (((USART_ISR(USART1) & USART_ISR_ORE) != 0))  //overrun, clear flag
	{
		USART1_ICR = USART_ICR_ORECF;
	}
}

int main(void)
{

	init();
	init_wdt();

 	radio_lora_settings_t s_lora;
 	s_lora.spreading_factor = 12;
	s_lora.bandwidth = BANDWIDTH_20_8K;
	s_lora.coding_rate = CODING_4_8;
	s_lora.implicit_mode = 0;
	s_lora.crc_en = 1;
	s_lora.low_datarate = 1;


 	radio_init();

  	//radio_reset();
 	radio_high_power();
	radio_set_frequency(FREQ_434_100);


	uint16_t k;

	while(1)
	{


		while(time_updated == 0 ||  pos_updated == 0 || gnss_status_updated == 0);

		nvic_disable_irq(NVIC_USART1_IRQ);
		int32_t _latitude = latitude;
		int32_t _longitude = longitude;
		int32_t _altitude = altitude/1000;
		uint8_t _hour = hour;
		uint8_t _minute = minute;
		uint8_t _second = second;
		uint8_t _sats = sats;
		gnss_status_updated = 0;
		pos_updated = 0;
		gnss_status_updated = 0;
		nvic_enable_irq(NVIC_USART1_IRQ);

		uint32_t bv = ADC1_DR;
		bv = bv * 8;
		bv = bv/100;
		adc_start_conversion_regular(ADC1);

		k=snprintf(buff,105,"xxxxx$$PAYLOAD,%u,",payload_counter++);
		if (time_valid)
			k+=snprintf(&buff[k],105-k,"%02u:%02u:%02u,",
					_hour,_minute,_second);
		else
			k+=snprintf(&buff[k],105-k,",");

		if (pos_valid)
			k+=snprintf(&buff[k],105-k,"%ld,%ld,%d,%u",
					_latitude,_longitude,_altitude,_sats);
		else
			k+=snprintf(&buff[k],105-k,",,,%u",
					_sats);

		k+=snprintf(&buff[k],105-k,",%u",bv);

		uint16_t crc = calculate_crc16(&buff[7]);

		k+=snprintf(&buff[k],15,"*%04X\n",crc);

		buff[0] = 0x55;
		buff[1] = 0xAA;
		buff[2] = 0x55;
		buff[3] = 0x80;
		buff[4] = 0x80;

		if (payload_counter == 4){
			while(1);}

		//WDT reset
		IWDG_KR = 0xAAAA;

		if (payload_counter & 1)  //rtty
		{
			radio_sleep();
			_delay_ms(10);
			radio_high_power();
			radio_start_tx_rtty((char*)buff,BAUD_50,4);
			while(rtty_in_progress() != 0){
				radio_rtty_poll_buffer_refill(20);
				_delay_ms(20);
			}
		}
		else   //lora
		{
			radio_sleep();
			_delay_ms(10);
			radio_write_lora_config(&s_lora);
			radio_high_power();

			GPIOB_ODR = 0;

			radio_tx_packet(&buff[5],k-5);
			//_delay_ms(12500);
			GPIOB_ODR = (1<<1);

			while(lora_in_progress())
				_delay_ms(50);


		}

		_delay_ms(1000);
	}

/*



	//radio_write_fsk_config(&s1);

	//radio_fsk_set_fifo_threshold(20);

	_delay_ms(100);

	snprintf(buff,60,"xxxxxHELLOsygygghhghghghghgghghfsfd: %d     \r\n",7);
	buff[0] = 0xFF;
	buff[1] = 0xFF;
	buff[2] = 0xFF;
	buff[3] = 0x80;
	buff[4] = 0x80;

	while(1)
	{}{
		//uint16_t d = usart_recv_blocking;
		//d++;
	//}{
		uint32_t r = USART1_ISR;
		uint32_t busy = r & (1<<16);
	//	uint32_t cha = USART1_RDR;

		if (r & USART_ISR_RXNE)
		{
			uint16_t data = USART1_RDR;

			//usart_send_blocking(USART1,'G');

			//USART1_RQR = (1<<3);
			data++;
		}
	}

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
    */
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
