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
#include <libopencm3/cm3/systick.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "radio.h"
#include "cmp.h"
#include "util.h"

extern void initialise_monitor_handles(void);

#define RADIO_FREQ  FREQ_434_300

#define RADIATION
#define ENABLE_GPS		//comment out if a GPS is not yet fitted
//#define LORA_RX		//old
//#define UPLINK			//enables/disables uplink after each lora packet
#define MULTI_POS		//enables the sending of multiple GPS positions in a packet. Only works with msgpack/lora
//#define TESTING		//disables the WDT and sets a fake payload name (to prevent being accidently left enabled)



#ifdef MULTI_POS
#define GPS_UPDATE_PERIOD 200				// in ms. Should be a factor of 1000

//Number of GPS positions to collect before starting to send another packet
// MAX_POSITIONS_PER_SENTENCE/GPS_UPDATE_RATE   should ideally be an integer
#define MAX_POSITIONS_PER_SENTENCE 20 //22    //TODO: ensure output buff is long enough
//memory usage: 3 bytes + 4 (scaling) + 2 (object 62) + 3*2 (describing arrays)

//Number of msgpack bytes. See item below
#define MAX_MSGPACK_LEN 130   //TODO: THIS

//Useful for testing. Ensure 'MAX_MSGPACK_LEN' number
//  of bytes can be sent before all the GPS positions have been collected
//	for the next packet
#define PAD_MSGPACK_TO_MAX   //TODO: this

static const uint8_t sentences_coding[] =    {CODING_4_5     };
static const uint8_t sentences_spreading[] = {8             };
static const uint8_t sentences_bandwidth[] = {BANDWIDTH_20_8K};


#else
#define GPS_UPDATE_PERIOD 1000

static const uint8_t sentences_coding[] =    {CODING_4_8,      CODING_4_6,      CODING_4_8,      CODING_4_5,      CODING_4_6,      CODING_4_6,     CODING_4_5,     0};
static const uint8_t sentences_spreading[] = {11,              8 ,              11,              8,               8,               11,             7,              0};
static const uint8_t sentences_bandwidth[] = {BANDWIDTH_20_8K, BANDWIDTH_20_8K, BANDWIDTH_41_7K, BANDWIDTH_41_7K, BANDWIDTH_20_8K, BANDWIDTH_20_8K, BANDWIDTH_125K, RTTY_SENTENCE};

//static const uint8_t sentences_coding[] =    {CODING_4_5,       0, 0};
//static const uint8_t sentences_spreading[] = {10,               0, 0};
//static const uint8_t sentences_bandwidth[] = {BANDWIDTH_41_7K,  RTTY_SENTENCE, RTTY_SENTENCE};
#endif

#define TOTAL_SENTENCES (sizeof(sentences_coding)/sizeof(uint8_t))
#define MAX_VAL_DIFFS (127-1)
#define MIN_VAL_DIFFS (-32+1)
//These need another -1/+1 to avoid overflow due to accumulated rounding issues




/* TIM14 option register (TIM14_OR) */
#define TIM_OR(tim_base)		MMIO32(tim_base + 0x50)
#define TIM14_OR			TIM_OR(TIM14)

void init(void);
void calibrate_hsi(void);
void _delay_ms(const uint32_t delay);
void uart_send_blocking_len(uint8_t *buff, uint16_t len);
uint16_t process_packet(char* buffer, uint16_t len, uint8_t format);
uint8_t find_diff_scaling(void);
uint8_t find_diff_scaling_alt(void);
void process_diffs(uint8_t scaling_factor,uint8_t scaling_factor_alt);
void process_diff(uint8_t scaling_factor, int32_t start_val, int32_t* input_diff, int8_t* output_diff);
void get_radiation(uint32_t* rad1, uint32_t* rad2, uint32_t* rad3, uint32_t* current);

#define CLK_PORT GPIOA
#define CLK_PIN GPIO9
#define DAT_PORT GPIOA
#define DAT_PIN GPIO10

static uint8_t sentence_counter = 0;

char buff[150] = {0};

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
//len = 14
const uint8_t set_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xF4, 0x01, 0x01, 0x00,
		0x00, 0x00, 0x0A, 0x75};


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

volatile uint16_t ms_countdown = 0;

uint16_t payload_counter = 0;
uint16_t uplink_counter = 0;

#ifdef MULTI_POS
volatile int32_t diff_lat [MAX_POSITIONS_PER_SENTENCE-1] = {0};  //TODO: check these are fine as int16
volatile int32_t diff_long[MAX_POSITIONS_PER_SENTENCE-1] = {0};
volatile int32_t diff_alt [MAX_POSITIONS_PER_SENTENCE-1] = {0};
int8_t diff_lat_out [MAX_POSITIONS_PER_SENTENCE-1] = {0};
int8_t diff_long_out [MAX_POSITIONS_PER_SENTENCE-1] = {0};
int8_t diff_alt_out [MAX_POSITIONS_PER_SENTENCE-1] = {0};
volatile uint8_t diff_count = 0;
volatile uint8_t diff_valid = 0;
volatile int32_t prev_latitude = 0;
volatile int32_t prev_longitude = 0;
volatile int32_t prev_altitude = 0;
volatile uint8_t second_prev = 99;
uint16_t diff_scaling_factor = 1;
uint8_t diff_scaling_factor_alt = 1;
#endif

#ifdef RADIATION
	uint32_t rad1;
	uint32_t rad2;
	uint32_t rad3;
	uint32_t current;
#endif


///////// msgpack stuff
//#define HB_BUF_LEN 100
//uint8_t hb_buf[HB_BUF_LEN] = {0};
uint8_t hb_buf_ptr = 0;

/*
static bool read_bytes(void *data, size_t sz, FILE *fh) {
    return fread(data, sizeof(uint8_t), sz, fh) == (sz * sizeof(uint8_t));
}

static bool file_reader(cmp_ctx_t *ctx, void *data, size_t limit) {
    return read_bytes(data, limit, (FILE *)ctx->buf);
}
*/
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


uint16_t cal_get_period(void)
{
	//measure ext clock
	uint16_t c1,c2;
	ms_countdown = 2*6*3;
	while(((TIM14_SR & (1<<1))==0) && (ms_countdown>0));  //CC1IF
	TIM14_SR |= (1<<1);
	while(((TIM14_SR & (1<<1))==0) && (ms_countdown>0));  //CC1IF
	TIM14_SR |= (1<<1);
	c1 = TIM14_CCR1;
	while(((TIM14_SR & (1<<1))==0) && (ms_countdown>0));  //CC1IF
	TIM14_SR |= (1<<1);
	c2 = TIM14_CCR1;

	while(((TIM14_SR & (1<<1))==0) && (ms_countdown>0));  //CC1IF
	TIM14_SR |= (1<<1);
	uint16_t c3 = TIM14_CCR1;

	if (ms_countdown == 0) //error condition
		return 0;

	int32_t diff = c2-c1;
	if(diff < 0)
		diff += 65536;	//correct for overflows
	return (uint16_t)diff;
}

void calibrate_hsi(void)
{
	return;
	/*
	uint8_t wasinsleep = 0;
	if ((radio_read_single_reg(REG_OP_MODE)&0x7) == 0){
		wasinsleep = 1;
		radio_standby();
	}
	//turn on clkout for DIO5
	uint8_t diomapping = radio_read_single_reg(REG_DIO_MAPPING2) & ~(3<<4);
	if ((radio_read_single_reg(REG_OP_MODE)&(1<<7)) == 0) //FSK
		radio_write_single_reg(REG_DIO_MAPPING2,diomapping);
	else
		radio_write_single_reg(REG_DIO_MAPPING2,diomapping | (1<<4));

	diomapping = radio_read_single_reg(REG_DIO_MAPPING2);
	*/
	//radio_sleep();
	//radio_write_single_reg(REG_OP_MODE,1);

	radio_write_single_reg(REG_OSC,5);    //1MHz
	rcc_clock_setup_in_hsi_out_48mhz();
	RCC_CR |= (1<<18);   //HSE Bypass
	RCC_CR |= (1<<16);   //HSE ON
	_delay_ms(50);

	//configure
	timer_reset(TIM14);
	TIM14_OR &= ~(0x3);
	TIM14_OR |= 0x2;	//set TIM14 ch1 input to HSE/32
	TIM14_CCMR1 = (0x9 << 4) | (3 << 2) | 1;    //bits1:0 = CC1 is an input, bits7:4 input filter, bits3:2 /4 prescaler.
	TIM14_CCER |= 1; //rising edges, enable capture
	timer_enable_counter(TIM14);
	const uint16_t normal_len = 32*48*4*2;
	const uint16_t error_len = 600*2;
	const uint16_t expected_min_error = 100*2;
	const uint16_t near_zero_error = 15*2;

	// 5% of 6144 (ideal period) is 307


	int16_t error = 100;

	//binary search for trim
	uint8_t trim_min = 0;
	uint8_t trim_max = 31;
	uint8_t tries = 5;
	uint8_t trim_val;
	uint32_t prev = RCC_CR;

	while((tries>0) && (abs(error) > near_zero_error))
	{
		tries--;
		trim_val = trim_min+((trim_max-trim_min)>>1);
		RCC_CR &= ~(0x1F<<3);
		RCC_CR |= trim_val<<3;
		error = ((uint16_t)cal_get_period())-normal_len;

		if (abs(error) > error_len)  //something went wrong, set trim to default
		{
			RCC_CR = prev;
			tries = 0;
		}

		if (error < 0){ //HSI too slow
			trim_min = trim_val;
		}else{//HSI too fast
			trim_max = trim_val;
		}


	}

/*
	uint8_t trim = 0;
	uint16_t tf[32];
	while(trim <= 0x1F){
		RCC_CR &= ~(0x1F<<3);
		RCC_CR |= trim<<3;
		diff = cal_get_period();
		tf[trim] = diff;
		trim++;

	}
	*/

	//final check
	error = ((uint16_t)cal_get_period())-normal_len;
	if (abs(error) > expected_min_error)  //something went wrong
	{
		RCC_CR &= ~(0x1F<<3);
		RCC_CR |= 16<<3;
	}



	timer_reset(TIM14);
	RCC_CR &= ~(1<<16); //HSE off
	radio_write_single_reg(REG_OSC,7);    //sx1278 osc out off
	rcc_clock_setup_in_hsi_out_8mhz();

	usart_send_blocking(USART1,0);
	usart_send_blocking(USART1,trim_val);
	////if (wasinsleep)
	//	radio_sleep();

}

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
	rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_TIM14);

	//systick
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(48000-1);		//1kHz at 8MHz clock
	systick_interrupt_enable();
	systick_counter_enable();

	RCC_CR &= ~(0x1F<<3); //trim the crystal down a little
	RCC_CR |= 15<<3;


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

#ifdef RADIATION
	gpio_mode_setup(CLK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CLK_PIN);
	gpio_mode_setup(DAT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, DAT_PIN);
#else
#ifdef TESTING
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10 | GPIO9);
#endif
#endif

#ifdef TESTING
	rcc_periph_clock_enable(RCC_GPIOF);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
#endif

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
	radio_init();

//	_delay_ms(200);
//	calibrate_hsi();
	uart_send_blocking_len((uint8_t*)flight_mode,44);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgga,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgll,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgsa,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpgsv,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gprmc,16);
	uart_send_blocking_len((uint8_t*)disable_nmea_gpvtg,16);

	memcpy(buff, set_rate,12);
	buff[6] = GPS_UPDATE_PERIOD & 0xFF;
	buff[7] = (GPS_UPDATE_PERIOD >> 8) & 0xFF;
	uint16_t ubloxcrc = calculate_ublox_crc((uint8_t*)&buff[2],10);

	uart_send_blocking_len((uint8_t*)buff,12);
	usart_send_blocking(USART1,ubloxcrc>>8);
	usart_send_blocking(USART1,ubloxcrc&0xFF);

	ubloxcrc = calculate_ublox_crc(&enable_navpvt[2],16-4);
	uart_send_blocking_len((uint8_t*)enable_navpvt,14);
	usart_send_blocking(USART1,ubloxcrc>>8);
	usart_send_blocking(USART1,ubloxcrc&0xFF);

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

void sys_tick_handler(void)
{
	if (ms_countdown)
		ms_countdown--;
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
			else if (d == 0xB5)
				gnss_string_count = 1;
			else
				gnss_string_count = 0;
		}
		else if (gnss_string_count == 2){  //message id
			gnss_message_id = d << 8;
			gnss_string_count++;
		}
		else if (gnss_string_count == 3){  //message id
			gnss_message_id |= d;
			if (gnss_message_id != 0x0107)
				gnss_string_count = 0;  //not looking for this string, reset
			else
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
			gnss_string_count++;

			if (gnss_string_count < sizeof(gnss_buff))
				*gnss_buff_ptr++ = d;
			else
				gnss_string_count = 0;   //something probably broke

			//if (gnss_string_count >= 256){
			//	gnss_string_count = 0;   //something probably broke
			//}

			if ((gnss_string_count-6-2) == gnss_string_len) //got all bytes, check checksum
			{
				//lets assume checksum == :)

				if ((gnss_message_id == 0x0107) && (gnss_string_len == 92))  //navpvt
				{
					fixtype = gnss_buff[20];
					uint8_t valid_time = gnss_buff[11];  //valid time flags

#ifdef MULTI_POS
					if ((valid_time & (1<<1)) && (diff_count==0))
					{
						hour = gnss_buff[8];
						minute = gnss_buff[9];
						//second_prev = second;
						second = gnss_buff[10];
						time_valid |= 1;
					}

					if (fixtype == 2 || fixtype == 3){
						if((diff_count>0) && (diff_count < MAX_POSITIONS_PER_SENTENCE)){
							int32_t temp;
							temp = (gnss_buff[31] << 24)
									 | (gnss_buff[30] << 16)
									 | (gnss_buff[29] << 8)
									 | (gnss_buff[28]);
							diff_lat[diff_count-1] = temp-prev_latitude;
							prev_latitude = temp;

							temp = (gnss_buff[27] << 24)
									 | (gnss_buff[26] << 16)
									 | (gnss_buff[25] << 8)
									 | (gnss_buff[24]);
							diff_long[diff_count-1] = temp-prev_longitude;
							prev_longitude = temp;

							temp = (gnss_buff[39] << 24)
									 | (gnss_buff[38] << 16)
									 | (gnss_buff[37] << 8)
									 | (gnss_buff[36]);
							diff_alt[diff_count-1] = temp-prev_altitude;
							prev_altitude = temp;

							diff_count++;
						}
						else if (diff_count==0){
							//check if time is aligned to .000 sec
							if ((valid_time & (1<<1)) && (second_prev != second)){
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
								prev_latitude = latitude;
								prev_longitude = longitude;
								prev_altitude = altitude;
								pos_valid |= 1;
								diff_count++;
								diff_valid = 1;
							}
						}
					}
					else if ((diff_count>0) && (diff_count < MAX_POSITIONS_PER_SENTENCE)){
						//if gps loses lock we still want to fill in the diff array
						//no changes to prev_position
						diff_lat[diff_count-1] = 0;
						diff_long[diff_count-1] = 0;
						diff_alt[diff_count-1] = 0;
						diff_count++;
					}
					else if (diff_count==0){
						//if no lock at start of sequence, then disable sending diffs
						diff_valid = 0;

						//diff_count=0; //keep this at 0.
					}

					if (valid_time & (1<<1))
						second_prev = gnss_buff[10];


#else
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
					if (valid_time & (1<<1))
					{
						hour = gnss_buff[8];
						minute = gnss_buff[9];
						second = gnss_buff[10];
						time_valid |= 1;
					}

#endif

					sats = gnss_buff[23];
					gnss_status_updated = 1;
					pos_updated = 1;
				}

				gnss_string_count = 0;  //wait for the next string
			}
		}
		//gpio_clear(GPIOA,GPIO0);
		//gpio_clear(GPIOF,GPIO1);
	}
	else// if (((USART_ISR(USART1) & USART_ISR_ORE) != 0))  //overrun, clear flag
	{
		USART1_ICR = USART_ICR_ORECF;
	}
	//else //clear all the things
	//{
	//	USART1_ICR = 0x20a1f;
	//}
}

int main(void)
{

	init();
#ifndef TESTING
	init_wdt();
#endif

 	radio_lora_settings_t s_lora;

	_delay_ms(100);


 	uint8_t uplink_en = 1;


 	radio_high_power();
	radio_set_frequency_frreg(RADIO_FREQ);

	uint16_t k;


	//initialise_monitor_handles(); /* initialize handles */
	//while(1)
	//printf("hello world!\r\n");



#ifdef MULTI_POS
	diff_count = 0;
#endif
	while(1)
	{

#ifndef ENABLE_GPS
		time_updated = 1;
#endif

		//calibrate_hsi();
		uart_send_blocking_len((uint8_t*)flight_mode,44);

#ifdef RADIATION
		get_radiation(&rad1, &rad2, &rad3, &current);
#endif

		while((pos_updated == 0) && (gnss_status_updated == 0))
			;//USART1_ICR = USART_ICR_ORECF | USART_ICR_FECF;
#ifdef MULTI_POS
		while((diff_count < MAX_POSITIONS_PER_SENTENCE) && (diff_valid > 0));
#endif

		//WDT reset
		IWDG_KR = 0xAAAA;

		gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
		if (pos_valid)
			GPIOB_ODR = 0;
		else
			GPIOB_ODR = (1<<1);

		sentence_counter++;
		if (sentence_counter >= TOTAL_SENTENCES)
			sentence_counter = 0;

		radio_sleep();
		_delay_ms(10);

		uplink_en = 0;
		if (sentences_bandwidth[sentence_counter] == RTTY_SENTENCE)//(payload_counter & 0x3) == 0x3)  //rtty
		{
			process_packet(buff,100,2);

			radio_high_power();
			radio_start_tx_rtty((char*)buff,BAUD_50,4);
			gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);  //turn off led
			//calibrate_hsi();
			while(rtty_in_progress() != 0){
				radio_rtty_poll_buffer_refill();
				_delay_ms(20);
			}
			_delay_ms(300);
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

#ifdef MULTI_POS
			if ((diff_count < MAX_POSITIONS_PER_SENTENCE))
				k=process_packet(buff,100,0);
			else{
				uint8_t d = find_diff_scaling();
				uint8_t da = find_diff_scaling_alt();
				diff_scaling_factor = d;
				diff_scaling_factor_alt = da;
				//process_diffs(d,0);
				process_diff(d, latitude, (int32_t *)diff_lat, diff_lat_out);
				process_diff(d, longitude, (int32_t *)diff_long, diff_long_out);
				process_diff(da, altitude, (int32_t *)diff_alt, diff_alt_out);
				k=process_packet(buff,100,3);
			}
			diff_count = 0; //make sure diff_count is reset to 0
#else
			k=process_packet(buff,100,1);
#endif



			radio_write_lora_config(&s_lora);

			radio_standby();
			radio_high_power();
			radio_set_frequency_frreg(RADIO_FREQ);

			gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1); //turn off led

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
			s_lora.spreading_factor = 12;
			radio_write_lora_config(&s_lora);
			radio_standby();
			radio_pa_off();
			radio_lna_max();
			radio_set_frequency_frreg(RADIO_FREQ);
			radio_write_single_reg(REG_IRQ_FLAGS,0xFF);
			radio_set_continuous_rx();

			//see if we get a header
			_delay_ms(400);
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


			radio_sleep();
			_delay_ms(10);
			//s_lora.bandwidth = BANDWIDTH_20_8K;
			//radio_write_lora_config(&s_lora);
			//radio_set_frequency_frreg(RADIO_FREQ);

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

#ifdef RADIATION
void get_radiation(uint32_t *rad1, uint32_t *rad2, uint32_t *rad3, uint32_t *current){

	uint8_t i,j;
	uint32_t* din;
	_delay_ms(1);

	for (i = 0; i < 4; i++){

		switch(i){
		case 0:	 din = rad1; break;
		case 1:	 din = rad2; break;
		case 2:	 din = rad3; break;
		default: din = current; break;
		}

		for (j = 0; j < 32; j++){
			*din <<= 1;
			gpio_set(CLK_PORT, CLK_PIN);
			_delay_ms(1);
			if (gpio_get(DAT_PORT, DAT_PIN))
				*din |= 1;
			gpio_clear(CLK_PORT, CLK_PIN);

			_delay_ms(1);
		}
	}
}
#endif

void process_diff(uint8_t scaling_factor, int32_t start_val, int32_t* input_diff, int8_t* output_diff)
{
	int32_t acc, current, diff;
	uint16_t i;

	current = start_val + input_diff[0];
	output_diff[0] = (int8_t)(input_diff[0] >> scaling_factor);   //TODO: CHECK THIS WILL BEHAVE AS EXPECTED!
	acc = start_val + ((int32_t)output_diff[0] << scaling_factor);
	for(i = 1; i < MAX_POSITIONS_PER_SENTENCE-1; i++){
		current = current + input_diff[i];
		diff = current - acc;
		output_diff[i] = (int8_t)(diff >> scaling_factor);
		acc = acc + ((int32_t)output_diff[i] << scaling_factor);
	}
}

void process_diffs(uint8_t scaling_factor, uint8_t scaling_factor_alt)
{
	int32_t lat_acc, lat_current, diff, long_acc, long_current, alt_acc, alt_current;
	uint16_t i;

	lat_current = latitude + diff_lat[0];
	diff_lat_out[0] = (int8_t)(diff_lat[0] >> scaling_factor);   //TODO: CHECK THIS WILL BEHAVE AS EXPECTED!
	lat_acc = latitude + ((int32_t)diff_lat_out[0] << scaling_factor);
	for(i = 1; i < MAX_POSITIONS_PER_SENTENCE-1; i++){
		lat_current = lat_current + diff_lat[i];
		diff = lat_current - lat_acc;
		diff_lat_out[i] = (int8_t)(diff >> scaling_factor);
		lat_acc = lat_acc + ((int32_t)diff_lat_out[i] << scaling_factor);
	}

//TODO: check diffs are not too big
	long_current = longitude + diff_long[0];
	diff_long_out[0] = (int8_t)(diff_long[0] >> scaling_factor);
	long_acc = longitude + ((int32_t)diff_long_out[0] << scaling_factor);
	for(i = 1; i < MAX_POSITIONS_PER_SENTENCE-1; i++){
		long_current = long_current + diff_long[i];
		diff = long_current - long_acc;
		diff_long_out[i] = (int8_t)(diff >> scaling_factor);
		long_acc = long_acc + ((int32_t)diff_long_out[i] << scaling_factor);
	}

	alt_current = altitude + diff_alt[0];
	diff_alt_out[0] = (int8_t)(diff_alt[0] >> scaling_factor_alt);
	alt_acc = altitude + ((int32_t)diff_alt_out[0] << scaling_factor_alt);
	for(i = 1; i < MAX_POSITIONS_PER_SENTENCE-1; i++){
		alt_current = alt_current + diff_alt[i];
		diff = alt_current - alt_acc;
		diff_alt_out[i] = (int8_t)(diff >> scaling_factor);
		alt_acc = alt_acc + ((int32_t)diff_alt_out[i] << scaling_factor_alt);
	}
}

uint8_t find_diff_scaling(void)
{
	int16_t diff_max = diff_lat[0];
	int16_t diff_min = diff_lat[0];
	uint16_t i;

	//find max/min
	for (i = 0; i < MAX_POSITIONS_PER_SENTENCE-1; i++){
		if (diff_lat[i] > diff_max)
			diff_max = diff_lat[i];
		if (diff_long[i] > diff_max)
			diff_max = diff_long[i];

		if (diff_lat[i] < diff_min)
			diff_min = diff_lat[i];
		if (diff_long[i] < diff_min)
			diff_min = diff_long[i];
	}


	uint16_t out = 0;
	while((diff_max > MAX_VAL_DIFFS) || (diff_min < MIN_VAL_DIFFS)){
		out++;
		diff_max = diff_max /2;
		diff_min = diff_min /2;
	}
	return out;
}

uint8_t find_diff_scaling_alt(void)
{
	int16_t diff_max = diff_alt[0];
	int16_t diff_min = diff_alt[0];
	uint16_t i;

	//find max/min
	for (i = 0; i < MAX_POSITIONS_PER_SENTENCE-1; i++){
		if (diff_alt[i] > diff_max)
			diff_max = diff_alt[i];
		if (diff_alt[i] < diff_min)
			diff_min = diff_alt[i];
	}


	uint16_t out = 0;
	while((diff_max > MAX_VAL_DIFFS) || (diff_min < MIN_VAL_DIFFS)){
		out++;
		diff_max = diff_max /2;
		diff_min = diff_min /2;
	}
	return out;
}


//returns length written
//format - 0 = habpack
//         1 = ascii
//         2 = ascii & rtty
//         3 = habpack diffs
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
	bv = bv * 6;
	bv = bv/100;
	adc_start_conversion_regular(ADC1);
	uint16_t k;

	if ((format == 1) || (format == 2)){
		k=0;
#ifndef MULTI_POS
		if  (format == 2)
			k=7;//snprintf(&buff[k],len,"xxxxx");
#ifdef TESTING
		k+=snprintf(&buff[k],len-k,"$$PAYIOAD,%u,",payload_counter++);
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
			buff[5] = 0x80;
			buff[6] = 0x80;
			crc = calculate_crc16(&buff[9]);
		}
		else
			crc = calculate_crc16(&buff[2]);

		k+=snprintf(&buff[k],15,"*%04X\n",crc);
		if  (format == 2){
			k+=snprintf(&buff[k],3,"XX");
			buff[k-1] = 0x80;
			buff[k-2] = 0x80;
		}
#endif
	}
	else if ((format == 0) || (format == 3))
	{
		memset((void*)buff,0,len);
		hb_buf_ptr = 0;

		cmp_ctx_t cmp;
		hb_buf_ptr = 0;
		cmp_init(&cmp, (void*)buff, 0, file_writer);

		uint8_t total_send = 6;
#ifdef RADIATION
		total_send += 2;
#endif
#ifdef UPLINK
		total_send += 1;
#endif

		if (format == 3)
			total_send += 3;

		cmp_write_map(&cmp,total_send);

		cmp_write_uint(&cmp, 0);
#ifdef TESTING
		cmp_write_str(&cmp, "PAYIOAD", 7);
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

#ifdef RADIATION
		cmp_write_uint(&cmp, 41);
		cmp_write_uint(&cmp, current & 0xFFFF);

		cmp_write_uint(&cmp, 39);
		cmp_write_array(&cmp, 2);
		cmp_write_uint(&cmp, rad1);
		cmp_write_uint(&cmp, rad2);
#endif
#ifdef UPLOAD
		cmp_write_uint(&cmp, 50);
		cmp_write_uint(&cmp, uplink_counter);
#endif
		if (format == 3){
			uint16_t i;
			cmp_write_uint(&cmp, 60);
			cmp_write_uint(&cmp, diff_scaling_factor);

			cmp_write_uint(&cmp, 61);	//altitude diff
			cmp_write_uint(&cmp, diff_scaling_factor_alt);

			cmp_write_uint(&cmp, 62);
			cmp_write_array(&cmp, 3);
			cmp_write_array(&cmp, MAX_POSITIONS_PER_SENTENCE-1);
			for (i = 0; i < MAX_POSITIONS_PER_SENTENCE-1; i++)
				cmp_write_int(&cmp, diff_lat_out[i]);
			cmp_write_array(&cmp, MAX_POSITIONS_PER_SENTENCE-1);
			for (i = 0; i < MAX_POSITIONS_PER_SENTENCE-1; i++)
				cmp_write_int(&cmp, diff_long_out[i]);
			cmp_write_array(&cmp, MAX_POSITIONS_PER_SENTENCE-1);
			for (i = 0; i < MAX_POSITIONS_PER_SENTENCE-1; i++)
				cmp_write_int(&cmp, diff_alt_out[i]);

		}

		return hb_buf_ptr;
	}
	else
		return 0;

	return k;
}



void _delay_ms(const uint32_t delay)
{
	ms_countdown = delay;
	while(ms_countdown);

}
