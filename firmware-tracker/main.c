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

#include <stdio.h>
#include <string.h>

#include "radio.h"
#include "cmp.h"

#define ENABLE_GPS
//#define LORA_RX
#define UPLINK

#define TESTING

void init(void);
void _delay_ms(const uint32_t delay);
void uart_send_blocking_len(uint8_t *buff, uint16_t len);
uint16_t calculate_crc16 (char *input);
uint16_t process_packet(char* buffer, uint16_t len, uint8_t format);

#define TOTAL_SENTENCES 8
#define RTTY_SENTENCE 0xFF
static const uint8_t sentences_coding[] =    {CODING_4_8,      CODING_4_6,      CODING_4_8,      CODING_4_5,      CODING_4_6,      CODING_4_8,     CODING_4_5,     0};
static const uint8_t sentences_spreading[] = {11,              8 ,              11,              8,               8,               11,             7,              0};
static const uint8_t sentences_bandwidth[] = {BANDWIDTH_20_8K, BANDWIDTH_20_8K, BANDWIDTH_41_7K, BANDWIDTH_41_7K, BANDWIDTH_20_8K, BANDWIDTH_125K, BANDWIDTH_125K, RTTY_SENTENCE};
static sentence_counter = 0;

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

///////// msgpack stuff
//#define HB_BUF_LEN 100
//uint8_t hb_buf[HB_BUF_LEN] = {0};
uint8_t hb_buf_ptr = 0;

static bool read_bytes(void *data, size_t sz, FILE *fh) {
    return fread(data, sizeof(uint8_t), sz, fh) == (sz * sizeof(uint8_t));
}

static bool file_reader(cmp_ctx_t *ctx, void *data, size_t limit) {
    return read_bytes(data, limit, (FILE *)ctx->buf);
}

static size_t file_writer(cmp_ctx_t *ctx, const void *data, size_t count) {

	uint16_t i;
	//if (hb_buf_ptr+count > HB_BUF_LEN)
	//	return -1;

	for (i = 0; i < count; i++)
	{
		((char*)ctx->buf)[hb_buf_ptr] = *((uint8_t*)data);
		data++;
		hb_buf_ptr++;
	}
	return count;
    //return fwrite(data, sizeof(uint8_t), count, (FILE *)ctx->buf);
}

/////////////////

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
#ifndef TESTING
	init_wdt();
#endif

 	radio_lora_settings_t s_lora;
 	s_lora.spreading_factor = 11;
	s_lora.bandwidth = BANDWIDTH_20_8K;
	s_lora.coding_rate = CODING_4_8;
	s_lora.implicit_mode = 0;
	s_lora.crc_en = 1;
	s_lora.low_datarate = 1;

	_delay_ms(100);

 	radio_init();
 	uint8_t uplink_en = 1;

#ifdef LORA_RX
 	radio_write_lora_config(&s_lora);
 	radio_pa_off();
	radio_lna_max();
	radio_set_frequency_frq(434098000);
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
		while (buff[i])
			usart_send_blocking(USART1, buff[i++]);

		if (irq & (1<<6))
		{
			int16_t r = radio_check_read_rx_packet(128,(uint8_t*)buff,1);

			//if (r > 0)
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
				if (error & (1<<19))
					error |= 0xFFF00000;
				snprintf(buff,60,"snr: %i  rssi: %i offset: %li     \r\n",snr,rssi,error);
				i=0;
				while (buff[i])
					usart_send_blocking(USART1, buff[i++]);
			}
			//else
			if (r < 0)
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
				radio_set_frequency_frreg(FREQ_434_100);
				radio_write_lora_config(&s_lora);
				radio_standby();
				radio_high_power();
				i=snprintf(buff,60,"PINGPINGPING");
				_delay_ms(700);
				radio_tx_packet(buff,i);

				_delay_ms(200);
				while(lora_in_progress())
					_delay_ms(50);

				radio_standby();
				radio_pa_off();
				radio_lna_max();
				radio_set_frequency_frreg(FREQ_434_100);
				radio_set_continuous_rx();


			}

		}

		_delay_ms(250);
	}


#endif


 	radio_high_power();
	radio_set_frequency_frreg(FREQ_434_100);

	uint16_t k;

	while(1)
	{

#ifndef ENABLE_GPS
		time_updated = 1;
#endif

		while(time_updated == 0 &&  pos_updated == 0 && gnss_status_updated == 0);


		//WDT reset
		IWDG_KR = 0xAAAA;

		sentence_counter++;
		if (sentence_counter >= TOTAL_SENTENCES)
			sentence_counter = 0;

		uplink_en = 0;
		if (sentences_bandwidth[sentence_counter] == RTTY_SENTENCE)//(payload_counter & 0x3) == 0x3)  //rtty
		{
			process_packet(buff,100,2);

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
			s_lora.spreading_factor = sentences_spreading[sentence_counter];
			s_lora.bandwidth = sentences_bandwidth[sentence_counter];
			s_lora.coding_rate = sentences_coding[sentence_counter];
			s_lora.implicit_mode = 0;
			s_lora.crc_en = 1;
			s_lora.low_datarate = 1;    //todo: this

			if (s_lora.bandwidth != BANDWIDTH_125K)
				uplink_en = 1;

			k=process_packet(buff,100,1);

			radio_sleep();
			_delay_ms(10);
			radio_write_lora_config(&s_lora);

			radio_standby();
			radio_high_power();
			radio_set_frequency_frreg(FREQ_434_100);

			radio_tx_packet((uint8_t*)(&buff[0]),k);

			_delay_ms(200);

			while(lora_in_progress())
				_delay_ms(50);
		}

#ifdef UPLINK

		if (uplink_en)// && !((payload_counter & 0x3) == 0x3))
		{
			radio_sleep();
			s_lora.bandwidth = BANDWIDTH_62_5K;
			radio_write_lora_config(&s_lora);
			radio_standby();
			radio_pa_off();
			radio_lna_max();
			radio_set_frequency_frreg(FREQ_434_100);
			radio_write_single_reg(REG_IRQ_FLAGS,0xFF);
			radio_set_continuous_rx();
			GPIOB_ODR = 0;
			//see if we get a header
			_delay_ms(300);
			uint8_t stat = radio_read_single_reg(REG_MODEM_STAT);
			if (stat & (1<<0))
			{
				//wait for packet
				uint8_t count = 100;
				_delay_ms(300);
				while(count){
					_delay_ms(40);
					uint8_t irq = radio_read_single_reg(REG_IRQ_FLAGS);
/*					//uint8_t nb = radio_read_single_reg(REG_RX_NB_BYTES);
					//uint8_t hrx = radio_read_single_reg(REG_RX_HEADER_CNT_VALUE_LSB);

					int32_t ui_offset = radio_read_single_reg(REG_FEI_MSB_LORA) << 8;
					ui_offset = (ui_offset | radio_read_single_reg(REG_FEI_MID_LORA)) << 8;
					ui_offset |= radio_read_single_reg(REG_FEI_LSB_LORA);

					if (ui_offset & 0x080000)
						ui_offset |= 0xFFF00000;

					snprintf(buff,60,"stat: %X  irq: %X headers rx: %X nBytes: %d offset: %li\r\n",stat,irq,hrx,nb,ui_offset);
					i=0;
					while (buff[i])
						usart_send_blocking(USART1, buff[i++]);
*/
					if (irq & (1<<6))
					{
						count = 0;
						uplink_counter++;
					}
					else
						count--;
				}
			}

			GPIOB_ODR = (1<<1);
			radio_sleep();
			_delay_ms(10);
			//s_lora.bandwidth = BANDWIDTH_20_8K;
			//radio_write_lora_config(&s_lora);
			//radio_set_frequency_frreg(FREQ_434_100);

			/*
			radio_standby();
			radio_high_power();
			*/
		}
		else
			_delay_ms(50);
#else
		_delay_ms(50);
#endif


	}
}

//returns length written
//format - 0 = habpack
//         1 = ascii
//         2 = ascii & rtty
uint16_t process_packet(char* buffer, uint16_t len, uint8_t format)
{
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
	uint16_t k;

	if ((format == 1) || (format == 2)){
		k=0;
		if  (format == 2)
			k=5;//snprintf(&buff[k],len,"xxxxx");
#ifdef TESTING
		k+=snprintf(&buff[k],len-k,"$$PAYLOAD,%u,",payload_counter++);
#else
		k+=snprintf(&buff[k],len-k,"$$SUSF,%u,",payload_counter++);
#endif
		if (time_valid)
			k+=snprintf(&buff[k],len-k,"%02u:%02u:%02u,",
					_hour,_minute,_second);
		else
			k+=snprintf(&buff[k],len-k,",");

		if (pos_valid)
			k+=snprintf(&buff[k],len-k,"%ld,%ld,%d,%u",
					_latitude,_longitude,_altitude,_sats);
		else
			k+=snprintf(&buff[k],len-k,",,,%u",
					_sats);

		k+=snprintf(&buff[k],len-k,",%u,%u",bv,uplink_counter);

		uint16_t crc;

		if (format == 2){
			buff[0] = 0x55;
			buff[1] = 0xAA;
			buff[2] = 0x55;
			buff[3] = 0x80;
			buff[4] = 0x80;
			crc = calculate_crc16(&buff[7]);
		}
		else
			crc = calculate_crc16(&buff[2]);

		k+=snprintf(&buff[k],15,"*%04X\n",crc);
	}
	else
	{
		memset((void*)buff,0,len);
		hb_buf_ptr = 0;

		cmp_ctx_t cmp;
		hb_buf_ptr = 0;
		cmp_init(&cmp, (void*)buff, file_reader, file_writer);


		cmp_write_map(&cmp, 7);


		cmp_write_uint(&cmp, 0);
#ifdef TESTING
		cmp_write_str(&cmp, "PAYLOAD", 7);
#else
		cmp_write_str(&cmp, "SUSF", 4);
#endif

		cmp_write_uint(&cmp, 1);
		cmp_write_uint(&cmp, payload_counter++);

		cmp_write_uint(&cmp, 2);
		cmp_write_uint(&cmp, (uint32_t)_hour*(3600) + (uint32_t)_minute*60 + (uint32_t)_second);

		cmp_write_uint(&cmp, 3);
		cmp_write_array(&cmp, 3);
		cmp_write_sint(&cmp, _latitude);
		cmp_write_sint(&cmp, _longitude);
		cmp_write_sint(&cmp, _altitude);

		cmp_write_uint(&cmp, 4);
		cmp_write_uint(&cmp, _sats);

		cmp_write_uint(&cmp, 40);
		cmp_write_uint(&cmp, bv);

		cmp_write_uint(&cmp, 50);
		cmp_write_uint(&cmp, uplink_counter);

		return hb_buf_ptr;
	}

	return k;
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
