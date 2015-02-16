#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/systick.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "radio.h"
#include "screen.h"
#include "bluetooth.h"
#include "telem_parser.h"
#include "main.h"


//#define ENABLE_GPS
#define LORA_RX
//#define UPLINK

char buff[128] = {0};
char buff_tx[32] = {0};


#define BTBUFFLEN 256
char bt_buff[BTBUFFLEN];
volatile uint16_t bt_buff_ptr_w = 0;
uint16_t bt_buff_ptr_r = 0;
uint16_t bt_buff_ptr_last = 0;
uint8_t in_new_string = 0;

/// The previous state of the rotary encoder
static volatile int rotary_prev = 0;

/// For debounce periods
static volatile int debounce_count = 0;

/// For rotary acceleration
static volatile int32_t rotary_acceleration = 0;

/// For counting time since x
static volatile uint16_t seconds_prescaler = 1000;
static volatile uint8_t bt_prescaler = 10;

/// Prevents double steps. When == 2 inc, when == -2 dec
static volatile int rotary_count = 0;


/// UI display variables (only being read in an ISR so doesnt need volatile?
static char ui_payload[14]="Waiting...   ";  //14 long
static uint32_t time_since_last = 0;
static uint32_t time_since_last_rstat = 0;
static uint8_t last_rstat = 0;  //0 - none, 1 - crc err, 2 - packet
static char ui_lati[10]="--.----  ";
static char ui_longi[10]="---.---- ";
static char ui_lati_prev[10]="--.----  ";
static char ui_longi_prev[10]="---.---- ";
static int32_t ui_alt = 0;
static int16_t ui_rssi = 0;
static int32_t ui_offset = 0;
static uint32_t ui_time = 0;
static uint32_t ui_time_prev = 0;
static volatile uint8_t ui_status_led = 0;   //bit0: led status (0 - bad, 1 - good); bit1: currently being flashed?
static int32_t ui_sequence = -1;
static volatile uint8_t redraw_timers_flag = 0;
static volatile uint8_t redraw_screen_flag = 0;
static uint8_t ui_position_valid_flags = 0;
static volatile uint8_t update_settings_flag = 0;
static volatile uint8_t write_flash_flag = 0;
static volatile uint8_t uplink_force_flag = 0;

static volatile int preset_btn_hold_count = 0;
static volatile int last_btn_preset = 0;
static volatile int last_btn_menu = 0;
static volatile int last_btn_rotm = 0;
static volatile int menu_preset_sel = 1;

static volatile uint16_t adc_input = 0;
static volatile uint16_t adc_batt = 0;
static volatile uint8_t adc_chan = 0;
static volatile uint8_t adc_low_input_count = 0;

/// radio parameters
volatile radio_lora_settings_t s_lora;
volatile uint16_t radio_bandwidth = 208;
volatile radio_lora_settings_t s_currently_unsaved;

volatile uint8_t auto_ping = 1;

#define PRESET_MAX 5
volatile radio_lora_settings_t presets[PRESET_MAX];
volatile uint8_t preset_lock[PRESET_MAX];




// UI state
typedef enum {PAYLOAD, RSSI, POSITION, RADIO_STATUS, RADIO_FREQ, BATT_VOLTAGE} ui_main_t;
ui_main_t ui_main = PAYLOAD;
typedef enum {MENU_FREQ, MENU_SF, MENU_BW, MENU_TRACK, MENU_UPLINK, MENU_AUTOPING} ui_menu_option_t;
ui_menu_option_t ui_menu = MENU_FREQ;
typedef enum {MENU_NONE, MENU_OPTIONS, MENU_PRESET, MENU_PRESET_WRITE, MENU_OPTIONS_WRITE, MENU_OPTIONS_CONFIRM} ui_menu_state_t;
ui_menu_state_t ui_state = MENU_NONE;
typedef enum {UP_PING, UP_CAL, UP_CUTDOWN} ui_uplink_msg_t;
ui_uplink_msg_t ui_selected_uplink = UP_PING;

static const uint8_t ui_icon_warning_arr[] = {0x1F,0x1B,0x1B,0x1B,0x1B,0x1F,0x1B,0x1F}; //1
static const uint8_t ui_icon_tick_arr[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; //2
static const uint8_t ui_icon_dot_arr[] = {0x00,0x0E,0x11,0x15,0x11,0x0E,0x00,0x00}; //3
static const char ui_icon_warning = 1;
static const char ui_icon_tick = 2;
static const char ui_icon_dot = 3;


void init(void);
void _delay_ms(const uint32_t delay);
void uart_send_blocking_len(uint8_t *buff, uint16_t len);
uint16_t calculate_crc16 (char *input);
static int rotary_state_next(const int current_state);
static int rotary_state_prev(const int current_state);
static int rotary_state(int in);
static ui_main_t inc_ui_main(ui_main_t in);
static ui_main_t dec_ui_main(ui_main_t in);
static void redraw_screen(void);
static void redraw_timers(void);
static void process_packet(char *buff, uint16_t max_len);
static void process_bt_buffer(void);
static uint8_t check_bt_buffer(void);
static ui_menu_option_t inc_ui_menu(ui_menu_option_t in);
static ui_menu_option_t dec_ui_menu(ui_menu_option_t in);
static char* get_bandwidth(uint8_t bandwidth);
static void change_setting(int8_t direction);
static uint8_t print_frequency(char* buff, uint32_t regfreq, uint8_t digits);
static void update_offset(void);
static void load_settings(radio_lora_settings_t);
static uint16_t get_radio_bandwidth(radio_lora_settings_t s);
static radio_lora_settings_t read_settings_flash(uint8_t pos);
static uint8_t write_settings_flash(radio_lora_settings_t in, uint8_t pos);
static uint8_t erase_settings_page(void);
static void correct_offset(void);
static ui_uplink_msg_t inc_uplink_msg(ui_uplink_msg_t in);
static ui_uplink_msg_t dec_uplink_msg(ui_uplink_msg_t in);
static void switch_to_rx(void);
static void switch_to_tx(void);
static void send_uplink(void);
static void low_batt(void);

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

}

void init (void)
{
	rcc_clock_setup_in_hsi_out_8mhz();
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);


	//UI stuff
	//rotary encoder A,B: B12,A15; middle: B14
	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO12 | GPIO14);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO15);
	//enable interrupts (just on one input)
	RCC_APB2ENR |= (1<<0);
	exti_select_source(EXTI14, GPIOB);
	exti_set_trigger(EXTI14, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI14);
	exti_select_source(EXTI12, GPIOB);
	exti_set_trigger(EXTI12, EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI12);
	nvic_enable_irq(NVIC_EXTI4_15_IRQ);
	exti_select_source(EXTI15, GPIOA);
	exti_set_trigger(EXTI15, EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI15);

	//menu button B6; preset button A5
	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO6);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO5);
	exti_select_source(EXTI6, GPIOB);
	exti_set_trigger(EXTI6, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI6);
	exti_select_source(EXTI5, GPIOA);
	exti_set_trigger(EXTI5, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI5);

	//leds
	gpio_mode_setup(LED_PST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, LED_PST_PIN);
	gpio_mode_setup(LED_MENU_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, LED_MENU_PIN);
	gpio_clear(LED_PST_PORT,LED_PST_PIN);
	gpio_clear(LED_MENU_PORT,LED_MENU_PIN);

	gpio_mode_setup(LED_RED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, LED_RED_PIN);
	gpio_mode_setup(LED_GRN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, LED_GRN_PIN);
	gpio_set(LED_RED_PORT,LED_RED_PIN);
	gpio_clear(LED_GRN_PORT,LED_GRN_PIN);

	//radio interrupt
	//poll :(
	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO15);


	//systick
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(7999);
	systick_interrupt_enable();
	systick_counter_enable();


	//adc
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	uint8_t channel_array[] = {ADC_CHANNEL0,ADC_CHANNEL1 };
	adc_power_off(ADC1);
	adc_calibrate_start(ADC1);
	adc_calibrate_wait_finish(ADC1);
	//adc_set_operation_mode(ADC1, ADC_MODE_SCAN); //adc_set_operation_mode(ADC1, ADC_MODE_SCAN_INFINITE);
	adc_set_operation_mode(ADC1, ADC_MODE_SEQUENTIAL);
//	adc_set_single_conversion_mode(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_239DOT5);
	adc_set_regular_sequence(ADC1, 2, channel_array);
	adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
	adc_set_single_conversion_mode(ADC1);
	adc_enable_eoc_interrupt(ADC1);
	nvic_enable_irq(NVIC_ADC_COMP_IRQ);
	adc_disable_analog_watchdog(ADC1);
	adc_power_on(ADC1);


	//uart
	//nvic_enable_irq(NVIC_USART1_IRQ);
	rcc_periph_clock_enable(RCC_USART1);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9|GPIO10);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO9|GPIO10);
	usart_set_baudrate(USART1, 115200);//9600 );
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);

/*
	//uart
	//nvic_enable_irq(NVIC_USART1_IRQ);
	rcc_periph_clock_enable(RCC_USART2);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2|GPIO3);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO2|GPIO3);
	usart_set_baudrate(USART2, 115200);//9600 );
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_enable(USART2);

*/

	adc_start_conversion_regular(ADC1);

}

void exti4_15_isr(void)
{
	/*if ((EXTI_PR & (GPIO15)) != 0)  //radio irq
	{

		int i;
		uint8_t r = radio_check_read_rx_packet(128,(uint8_t*)buff,1);
		if (r > 0)
		{
			for (i = 0; i < r; i++)
				usart_send_blocking(USART1, buff[i]);
			//usbd_ep_write_packet(usbd_dev, 0x82, buff, r);

			int16_t snr = radio_read_single_reg(REG_PKT_SNR_VALUE);
			if (snr & 0x80)
				snr |= 0xFF00;
			int16_t rssi = radio_read_single_reg(REG_PKT_RSSI_VALUE)-164;
			int32_t error = radio_read_single_reg(REG_FEI_MSB_LORA) << 8;
			error = (error | radio_read_single_reg(REG_FEI_MID_LORA)) << 8;
			error |= radio_read_single_reg(REG_FEI_LSB_LORA);

			r = snprintf(buff,60,"snr: %i  rssi: %i offset: %li     ",snr,rssi,error);
			i=0;
			while (buff[i])
				usart_send_blocking(USART1, buff[i++]);

		}


		EXTI_PR |= (GPIO15);
	}
*/
	if ((EXTI_PR & (GPIO12)) != 0 || ((EXTI_PR & (GPIO15)) != 0))  //rotary encoder irq
	{
		int current_state;
		// Don't do anything unless the debounce interval has expired
		if (debounce_count == 0)
		{
			int rotary_incdec = 0;
			debounce_count = DEBOUNCE_INTERVAL_MS;

			current_state = rotary_state((GPIO_IDR(BTN_ROTA_PORT) & BTN_ROTA_PIN) |
					(GPIO_IDR(BTN_ROTB_PORT) & BTN_ROTB_PIN));

			if(current_state == rotary_state_next(rotary_prev))  //next screen
				rotary_count++;
			if(current_state == rotary_state_prev(rotary_prev))  //prev screen
				rotary_count--;

			if (rotary_count >= 2){
				rotary_incdec = -1;
				rotary_count = 0;
				rotary_acceleration += 150;
			}
			if (rotary_count <= -2){
				rotary_incdec = 1;
				rotary_count = 0;
				rotary_acceleration += 150;
			}

			//now actually change stuff
			if (ui_state == MENU_NONE){
				if (rotary_incdec == 1)
				{
					ui_main = inc_ui_main(ui_main);
					redraw_screen_flag = 1;
				}
				if (rotary_incdec == -1)
				{
					ui_main = dec_ui_main(ui_main);
					redraw_screen_flag = 1;
				}
			}
			else if (ui_state == MENU_OPTIONS)
			{
				if (rotary_incdec == 1)
				{
					ui_menu = inc_ui_menu(ui_menu);
					redraw_screen_flag = 1;
				}
				if (rotary_incdec == -1)
				{
					ui_menu = dec_ui_menu(ui_menu);
					redraw_screen_flag = 1;
				}
			}
			else if (ui_state == MENU_OPTIONS_WRITE)
				change_setting(rotary_incdec);
			else if ((ui_state == MENU_PRESET) || (ui_state == MENU_PRESET_WRITE))
			{
				menu_preset_sel += rotary_incdec;
				if (menu_preset_sel < 1)
					menu_preset_sel = 1;
				if (menu_preset_sel > PRESET_MAX)
					menu_preset_sel = PRESET_MAX;
				redraw_screen_flag = 1;
			}
		}

		if ((EXTI_PR & (GPIO15)) != 0)
			EXTI_PR |= (GPIO15);
		if ((EXTI_PR & (GPIO12)) != 0)
			EXTI_PR |= (GPIO12);
	}
	else if ((EXTI_PR & (BTN_PST_PIN)) != 0 )  //preset
	{

		if (debounce_count == 0 && last_btn_preset)
		{
			debounce_count = DEBOUNCE_INTERVAL_MS;
			if ((ui_state != MENU_PRESET) && (ui_state != MENU_PRESET_WRITE))
			{
				ui_state = MENU_PRESET;

			}
			else
				ui_state = MENU_NONE;

			redraw_screen_flag = 1;
		}
		EXTI_PR |= (BTN_PST_PIN);
	}
	else if ((EXTI_PR & (BTN_MENU_PIN)) != 0)   //menu
	{
		if (debounce_count == 0  && last_btn_menu)
		{
			debounce_count = DEBOUNCE_INTERVAL_MS;
			if ((ui_state != MENU_OPTIONS) && (ui_state != MENU_OPTIONS_WRITE))
			{
				ui_state = MENU_OPTIONS;
				ui_menu = MENU_FREQ;
				s_currently_unsaved = s_lora;
			}
			else
				ui_state = MENU_NONE;
			redraw_screen_flag = 1;
		}

		EXTI_PR |= (BTN_MENU_PIN);
	}
	else if ((EXTI_PR & (BTN_ROTM_PIN)) != 0)   //rotary middle
	{
		if (debounce_count == 0  && last_btn_rotm)
		{
			debounce_count = DEBOUNCE_INTERVAL_MS*3;
			if (ui_state == MENU_OPTIONS){
				ui_state = MENU_OPTIONS_WRITE;
				s_currently_unsaved = s_lora;
				redraw_screen_flag = 1;
			}
			else if (ui_state == MENU_OPTIONS_WRITE){
				if (ui_menu == MENU_UPLINK){
					ui_state = MENU_OPTIONS_CONFIRM;
					redraw_screen_flag = 1;
				}else{
					ui_state = MENU_OPTIONS;
					//s_lora = s_currently_unsaved;
					update_settings_flag = 1; //load_settings(s_currently_unsaved);
					screen_write_text("~",ROW_BOT + 15);
				}
			}
			else if (ui_state == MENU_PRESET){
				ui_state = MENU_NONE;
				s_currently_unsaved = presets[menu_preset_sel-1];
				update_settings_flag = 1; //load_settings(s_lora);
				radio_bandwidth = get_radio_bandwidth(s_lora);
				redraw_screen_flag = 1;
				//write settings
				//update radio_bandwidth
			}
			else if (ui_state == MENU_PRESET_WRITE){
				if (preset_lock[menu_preset_sel-1]==0){
					presets[menu_preset_sel-1] = s_lora;
					ui_state = MENU_NONE;
					redraw_screen_flag = 1;
					write_flash_flag = 1;
				}
			}
			else if (ui_state == MENU_OPTIONS_CONFIRM)
			{
				uplink_force_flag = 1;
				ui_state = MENU_NONE;
			}

		}
		//redraw_screen_flag = 1;
		EXTI_PR |= (GPIO14);
	}

}

void adc_comp_isr(void)
{
	if (adc_chan)
	{
		adc_input = adc_read_regular(ADC1);
	}
	else
	{
		adc_chan = 1;
		adc_batt = adc_read_regular(ADC1);
		adc_start_conversion_regular(ADC1);
		if (adc_batt < 2250 && adc_input < 2250)  //~3.6V
		{
			adc_low_input_count++;
			if (adc_low_input_count > 3)
				low_batt();
		}
		else
			adc_low_input_count = 0;
	}

}

void usart2_isr(void)
{
	if (((USART_ISR(USART2) & USART_ISR_RXNE) != 0))
	{
		uint8_t d = (uint8_t)USART2_RDR;
		bt_buff_ptr_w++;
		if (bt_buff_ptr_w >= BTBUFFLEN)
			bt_buff_ptr_w = 0;
		bt_buff[bt_buff_ptr_w] = d;
	}
	else if (((USART_ISR(USART2) & USART_ISR_ORE) != 0))  //overrun, clear flag
	{
		USART2_ICR = USART_ICR_ORECF;
	}
}

void sys_tick_handler(void)
{

	if (debounce_count)
	{
		debounce_count--;
		rotary_prev = rotary_state((GPIO_IDR(BTN_ROTA_PORT) & BTN_ROTA_PIN) |
				(GPIO_IDR(BTN_ROTB_PORT) & BTN_ROTB_PIN));

	}

	if (rotary_acceleration)
	{
		rotary_acceleration -= 1;
	}

	if (bt_prescaler == 0){
		bt_timer_10ms_tick();
		bt_prescaler = 10;
	}
	bt_prescaler--;

	if (seconds_prescaler == 0)
	{
		seconds_prescaler = 1000;
		time_since_last++;
		time_since_last_rstat++;
		redraw_timers_flag = 1;
		if (ADC1_ISR & (ADC_ISR_EOSMP)){
			adc_start_conversion_regular(ADC1);
			adc_chan = 0;
		}
	}
	seconds_prescaler--;
	if (seconds_prescaler == 500){
		if (ui_status_led & 2){
			ui_status_led = (~ui_status_led)&1;
			if ((ui_status_led&1) == 0){
				gpio_clear(LED_RED_PORT,LED_RED_PIN);
				gpio_set(LED_GRN_PORT,LED_GRN_PIN);
			}else{
				gpio_set(LED_RED_PORT,LED_RED_PIN);
				gpio_clear(LED_GRN_PORT,LED_GRN_PIN);
			}
		}
	}

	if ((GPIO_IDR(BTN_PST_PORT) & BTN_PST_PIN) == 0)
	{
		preset_btn_hold_count++;
		if (preset_btn_hold_count > PRESET_HOLD_THRES)
		{
			if (ui_state != MENU_PRESET_WRITE)
			{
				ui_state = MENU_PRESET_WRITE;
				redraw_screen_flag = 1;
			}
		}
	}
	else
		preset_btn_hold_count = 0;

	last_btn_preset = GPIO_IDR(BTN_PST_PORT) & BTN_PST_PIN;
	last_btn_menu = GPIO_IDR(BTN_MENU_PORT) & BTN_MENU_PIN;
	last_btn_rotm = GPIO_IDR(BTN_ROTM_PORT) & BTN_ROTM_PIN;

}


void uart_send_blocking_len(uint8_t *buff, uint16_t len)
{
	uint16_t i = 0;
	for (i = 0; i < len; i++)
		usart_send_blocking(USART1,*buff++);

}


static void process_bt_buffer(void)
{
	if (check_bt_buffer())
	{
		bt_process_line_rx(bt_buff, bt_buff_ptr_last, bt_buff_ptr_r, BTBUFFLEN);
		bt_buff_ptr_last = bt_buff_ptr_r;
	}
}

//looks for strings in the bt buffer
//returns 1 if it has found a new string
static uint8_t check_bt_buffer(void)
{
	if (bt_buff_ptr_r != bt_buff_ptr_w)
	{
		if (bt_buff_ptr_r == bt_buff_ptr_last && in_new_string == 0) //increment both ptrs until a non \r\n char is found
		{
			bt_buff_ptr_r++;
			if (bt_buff_ptr_r >= BTBUFFLEN)
				bt_buff_ptr_r = 0;

			if (!((bt_buff[bt_buff_ptr_r] == (char)10) || (bt_buff[bt_buff_ptr_r] == (char)13)))
				in_new_string = 1;

			bt_buff_ptr_last = bt_buff_ptr_r;
		}
		else if (in_new_string == 1)
		{
			uint16_t temp = bt_buff_ptr_r + 1;

			if (temp >= BTBUFFLEN)
				temp = 0;

			if (((bt_buff[temp] == (char)10) || (bt_buff[temp] == (char)13))){
				in_new_string = 0;
				return 1;
			}
			else
				bt_buff_ptr_r = temp;
		}
	}
	return 0;
}


int main(void)
{

	init();
	screen_init();
	bluetooth_init();
	usart_enable_rx_interrupt(USART2);
	nvic_enable_irq(NVIC_USART2_IRQ);  //enable BT interrupts


	backlight_on();
	bluetooth_wakeup();


	screen_add_cc(ui_icon_warning_arr, 1);
	screen_add_cc(ui_icon_tick_arr, 2);
	screen_add_cc(ui_icon_dot_arr, 3);

	uint8_t r;
/*
	while(1)
		{
			if (USART1_ISR & (1<<5)){  //RXNE
				r = USART1_RDR;
				usart_send_blocking(USART2, r);
			}
			if (USART2_ISR & (1<<5)){  //RXNE
				r = USART2_RDR;
				usart_send_blocking(USART1, r);
			}
			USART1_ICR = (1<<3);
			USART2_ICR = (1<<3);
		}
*/




	buff[0] = 0b01000000;
	buff[1] = 'A';
	buff[2] = 'b';


	screen_on();

	radio_init();

	s_lora.spreading_factor = 12;
	s_lora.bandwidth = BANDWIDTH_20_8K;
	s_lora.coding_rate = CODING_4_5;
	s_lora.implicit_mode = 0;
	s_lora.crc_en = 1;
	s_lora.low_datarate = 1;
	s_lora.enable_frequency_tracking = 1;
	s_lora.frequency = 434100000;



	_delay_ms(100);

	radio_lora_settings_t s_test;
	s_test = read_settings_flash(0);
	if ((s_test.bandwidth == 0xFF) && (s_test.coding_rate == 0xFF) && (s_test.spreading_factor == 0xFF)){
		//load presets
		for (r = 0; r < PRESET_MAX; r++)
			presets[r] = s_lora;

		flash_unlock();
		erase_settings_page();
		for (r = 0; r < PRESET_MAX; r++)
			write_settings_flash(s_lora,r);
	}
	else{
		for (r = 0; r < PRESET_MAX; r++)
			presets[r] = read_settings_flash(r);

		if ((GPIO_IDR(BTN_MENU_PORT) & BTN_MENU_PIN) || (GPIO_IDR(BTN_PST_PORT) & BTN_PST_PIN)){
			preset_lock[0] = 1;
			preset_lock[1] = 1;
		}
	}

	radio_write_lora_config((radio_lora_settings_t *)&s_lora);
	radio_pa_off();
	radio_lna_max();
	radio_set_frequency_frq(s_lora.frequency);
	radio_set_continuous_rx();




	bluetooth_configure();
	_delay_ms(1000);

	redraw_screen();

	//clear buffer
	bt_buff_ptr_r = bt_buff_ptr_w;
	bt_buff_ptr_last = bt_buff_ptr_w;

	check_characteristic_handle();
	while(bt_waiting_for_response()) { process_bt_buffer(); }
	start_advertise();
	while(bt_waiting_for_response()) { process_bt_buffer(); }




	//while(bt_get_status() != CONNECTED) process_bt_buffer();

	//_delay_ms(2000);


/*
 *
    uint32_t seq;
	uint32_t time;
	int32_t alt;
	char str[] = "$$icarus,418,10:00:43,54.125980,-0.656900,14353,51.86,85.9,-8.0,-35.4*9A36";


	uint8_t sending = start_bt_telem_send((uint8_t *)str,74);
	while(bt_waiting_for_response()) { process_bt_buffer(); }
	while(sending)
	{
		sending = continue_bt_telem_send();
		while(bt_waiting_for_response()) { process_bt_buffer(); }
	}

	while(1)
	{
		send_uplink();
		_delay_ms(100);
	}
*/
	while(1)
	{
		if (redraw_timers_flag) {
			redraw_timers();
			if (ui_state == MENU_NONE && ui_main == RADIO_STATUS){
				uint8_t stat = radio_read_single_reg(REG_MODEM_STAT);
				snprintf(buff,17,"Radio Status: %02X",stat);
				screen_write_text(buff,0);
			}
		}

		if (update_settings_flag) { s_lora = s_currently_unsaved; load_settings(s_lora); }
		if (redraw_screen_flag) { redraw_screen(); }
		if (write_flash_flag){
			write_flash_flag = 0;
			flash_unlock();
			erase_settings_page();
			for (r = 0; r < PRESET_MAX; r++)
				write_settings_flash(presets[r],r);
		}

		if (uplink_force_flag){
			uplink_force_flag = 0;
			ui_state = MENU_NONE;
			screen_write_text("Transmitting... ",0);
			screen_clear_row(ROW_BOT);
			send_uplink();
			send_uplink();
			send_uplink();
			send_uplink();
			redraw_screen();
		}

		process_bt_buffer();
		if (bt_get_status() == DISCONNECTED)
			start_advertise();

		if (GPIO_IDR(RADIO_INT_PORT) & RADIO_INT_PIN)
		{
		//uint8_t irq = radio_read_single_reg(REG_IRQ_FLAGS);
		//if (irq & (1<<6)){
			int16_t r = radio_check_read_rx_packet(128,(uint8_t*)buff,1);
			if (r > 0)
			{
				last_rstat = 2;
				time_since_last_rstat = 0;
				//parse_ascii(buff, 127, ui_payload, &seq, &time, ui_lati, ui_longi, &ui_alt, 10);
				//redraw_screen();
				//time_since_last = 0;
				process_packet(buff,127);

				if (bt_get_status() == CONNECTED){
					uint8_t sending = start_bt_telem_send((uint8_t *)buff,r);
					while(bt_waiting_for_response()) { process_bt_buffer(); }
					while(sending)
					{
						sending = continue_bt_telem_send();
						while(bt_waiting_for_response()) { process_bt_buffer(); }
					}
				}
			}
			else if (r == -2){
				last_rstat = 1;
				time_since_last_rstat = 0;
				correct_offset();
				redraw_screen();
			}
		}
		_delay_ms(10); // do something else for a while
		//if screen = status poll status

	}
/*
	parse_ascii(telem, 127, call, &seq, &time, lati, longi, &alt, 10);
	screen_write_text(call, 0);
	screen_write_text(lati, 6);
	screen_write_text(longi, ROW_BOT | 0);


	while(1)
	{

		//if (GPIO_IDR(RADIO_INT_PORT) & RADIO_INT_PIN)
		//{
		uint8_t irq = radio_read_single_reg(REG_IRQ_FLAGS);
		if (irq & (1<<6)){
			int16_t r = radio_check_read_rx_packet(128,(uint8_t*)buff,1);
			if (r > 0)
			{
				parse_ascii(buff, 127, ui_payload, &seq, &time, ui_lati, ui_longi, &ui_alt, 10);
				redraw_screen();
			}
		}
		_delay_ms(100); // do something else for a while
		//if screen = status poll status
	}

*/
	while(1)
	{
		if (USART1_ISR & (1<<5)){  //RXNE
			r = USART1_RDR;
			usart_send_blocking(USART2, r);
		}
		if (USART2_ISR & (1<<5)){  //RXNE
			r = USART2_RDR;
			usart_send_blocking(USART1, r);
		}
		USART1_ICR = (1<<3);
		USART2_ICR = (1<<3);
	}


//	init_wdt();




#ifdef LORA_RX
 	radio_write_lora_config((radio_lora_settings_t *)&s_lora);
 	radio_pa_off();
	radio_lna_max();
	radio_set_frequency_frreg(FREQ_434_100);
	radio_set_continuous_rx();
	int i;
	int j=0;
	while(1)
	{
		if (redraw_timers_flag)
			redraw_timers();

		uint8_t stat = radio_read_single_reg(REG_MODEM_STAT);
		uint8_t irq = radio_read_single_reg(REG_IRQ_FLAGS);
		uint8_t nb = radio_read_single_reg(REG_RX_NB_BYTES);
		uint8_t hrx = radio_read_single_reg(REG_RX_HEADER_CNT_VALUE_LSB);

		snprintf(buff,60,"stat: %X  irq: %X headers rx: %X nBytes: %d\r\n",stat,irq,hrx,nb);
		i=0;
		while (buff[i])
			usart_send_blocking(USART1, buff[i++]);

		//if (irq & (1<<6))
		if (GPIO_IDR(RADIO_INT_PORT) & RADIO_INT_PIN)
		{
			int16_t r = radio_check_read_rx_packet(128,(uint8_t*)buff,1);

			if (r > 0)
			{
				process_packet(buff,127);

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
				radio_set_frequency_frreg(FREQ_434_100);
				radio_write_lora_config((radio_lora_settings_t *)&s_lora);
				radio_standby();
				radio_high_power();
				i=snprintf(buff,60,"PINGPINGPING");

				radio_tx_packet((uint8_t*)buff,i);

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
			radio_write_lora_config((radio_lora_settings_t *)&s_lora);

			radio_standby();
			radio_high_power();
			radio_set_frequency_frreg(FREQ_434_100);



			radio_tx_packet((uint8_t*)&buff[5],k-5);
			//_delay_ms(12500);


			_delay_ms(200);


			while(lora_in_progress())
				_delay_ms(50);


		}
	}


 	
}

static void update_offset(void)
{
	ui_offset = radio_read_single_reg(REG_FEI_MSB_LORA) << 8;
	ui_offset = (ui_offset | radio_read_single_reg(REG_FEI_MID_LORA)) << 8;
	ui_offset |= radio_read_single_reg(REG_FEI_LSB_LORA);

	if (ui_offset & 0x080000)
		ui_offset |= 0xFFF00000;

	int64_t t = (int64_t)ui_offset << (24-11);
	t = t * (int64_t)radio_bandwidth;
	t = t / (int64_t)(500 * ((int64_t)32000000>>11) * 10);

	//ui_offset *= 21;
	//ui_offset *= radio_bandwidth;
	//ui_offset /= 10000;
	ui_offset = (uint32_t) t;
}

static void correct_offset(void)
{
	update_offset();
	if (s_lora.enable_frequency_tracking && update_settings_flag == 0){
		if (abs(ui_offset) > 250){
			//uint32_t f = radio_read_single_reg(REG_FRF_MSB) << 8;
			//f = (f | radio_read_single_reg(REG_FRF_MID)) << 8;
			//f |= radio_read_single_reg(REG_FRF_LSB);
			s_lora.frequency -= ui_offset;
			load_settings(s_lora);
			ui_offset = 0;
		}
	}
}


void process_packet(char *buff, uint16_t max_len)
{
	correct_offset();


	if ((buff[0] & 0xF0) == 0x80){ //check for msgpack
		int32_t la,lo;
		ui_position_valid_flags = parse_habpack(buff, max_len, ui_payload,
				(uint32_t *)(&ui_sequence), &ui_time, &la, &lo, &ui_alt, 10);
		if (la != 0) //TODO: change for sats field
			snprintf(ui_lati,10,"%ld",la);
		else
			ui_position_valid_flags &= ~(1<<2);

		if (lo != 0)
			snprintf(ui_longi,10,"%ld",lo);
		else
			ui_position_valid_flags &= ~(1<<3);

	}
	else
		ui_position_valid_flags = parse_ascii(buff, max_len, ui_payload, (uint32_t *)(&ui_sequence), &ui_time, ui_lati, ui_longi, &ui_alt, 10);
	time_since_last = 0;
	seconds_prescaler = 1000;

	//check if telemetry is good
	if ((ui_time == ui_time_prev) ||
			(strcmp(ui_lati,ui_lati_prev) == 0) ||
			(strcmp(ui_longi,ui_longi_prev)==0) ||
			(ui_position_valid_flags != 0b11111))
	{
		ui_status_led = 0;
	}
	else
		ui_status_led = 1;
	ui_time_prev = ui_time;
	strncpy(ui_lati_prev,ui_lati,10);
	strncpy(ui_longi_prev,ui_longi,10);

	//flash status LED
	ui_status_led = ((~ui_status_led)&1) | 2;
	if ((ui_status_led&1) == 0){
		gpio_clear(LED_RED_PORT,LED_RED_PIN);
		gpio_set(LED_GRN_PORT,LED_GRN_PIN);
	}else{
		gpio_set(LED_RED_PORT,LED_RED_PIN);
		gpio_clear(LED_GRN_PORT,LED_GRN_PIN);
	}

	//uplink stuff
	if (ui_state == MENU_OPTIONS_CONFIRM)
	{
		//see if uplink window
		//just assume all packets have uplink windows
		screen_write_text("Transmitting... ",0);
		send_uplink();
		redraw_screen();
	}
	else if (auto_ping)
	{
		//see if uplink window
		//just assume all packets have uplink windows
		screen_write_text("Transmitting... ",0);
		send_uplink();
		redraw_screen();
	}

	//get rssi
	ui_rssi = radio_read_single_reg(REG_PKT_RSSI_VALUE)-164;

	redraw_screen();
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


/**
 * Given a current rotary encoder state, return what would be the 'next' state
 * of the encoder (i.e. turned clockwise).
 * @param current_state The current state of the encoder
 * @returns The next state of the encoder
 */
static int rotary_state_next(const int current_state)
{
    switch(current_state)
    {
        case 0:
            return 2;
        case 2:
            return 3;
        case 3:
            return 1;
        default:
            return current_state - 1;
    }
}

/**
 * Given a current rotary encoder state, return what would be the 'previous'
 * state of the encoder (i.e. turned anticlockwise).
 * @param current_state The current state of the encoder
 * @returns The previous state of the encoder
 */
static int rotary_state_prev(const int current_state)
{
    switch(current_state)
    {
        case 1:
            return 3;
        case 2:
            return 0;
        case 3:
            return 2;
        default:
            return current_state + 1;
    }
}

/**
 * Get the current state of the rotary encoder input as a value between 0 and 3
 * inclusive
 * @param in The current state
 * @returns The new state
 */
int rotary_state(int in)
{
	int out = 0;
    if (in & BTN_ROTA_PIN)
    	out = 2;
    if (in & BTN_ROTB_PIN)
    	out |= 1;
    return out;
}


static ui_main_t inc_ui_main(ui_main_t in)
{
	switch(in){
		case PAYLOAD:
			return RSSI;
		case RSSI:
			return POSITION;
		case POSITION:
			return RADIO_STATUS;
		case RADIO_STATUS:
			return RADIO_FREQ;
		case RADIO_FREQ:
			return BATT_VOLTAGE;
		default: //BATT_VOLTAGE
			return PAYLOAD;
	}
}
static ui_main_t dec_ui_main(ui_main_t in)
{
	switch(in){
		case PAYLOAD:
			return BATT_VOLTAGE;
		case RSSI:
			return PAYLOAD;
		case POSITION:
			return RSSI;
		case RADIO_STATUS:
			return POSITION;
		case RADIO_FREQ:
			return RADIO_STATUS;
		default: //BATT_VOLTAGE
			return RADIO_FREQ;
	}
}
static ui_menu_option_t inc_ui_menu(ui_menu_option_t in)
{
	switch(in){
		case MENU_FREQ:
			return MENU_SF;
		case MENU_SF:
			return MENU_BW;
		case MENU_BW:
			return MENU_TRACK;
		case MENU_TRACK:
			return MENU_UPLINK;
		case MENU_UPLINK:
			return MENU_AUTOPING;
		default: //MENU_AUTOPING
			return MENU_FREQ;
	}
}
static ui_menu_option_t dec_ui_menu(ui_menu_option_t in)
{
	switch(in){
		case MENU_FREQ:
			return MENU_AUTOPING;
		case MENU_SF:
			return MENU_FREQ;
		case MENU_BW:
			return MENU_SF;
		case MENU_TRACK:
			return MENU_BW;
		case MENU_UPLINK:
			return MENU_TRACK;
		default: //MENU_AUTOPING:
			return MENU_UPLINK;
	}
}

static ui_uplink_msg_t inc_uplink_msg(ui_uplink_msg_t in)
{
	switch(in){
		case UP_PING:
			return UP_CAL;
		case UP_CAL:
			return UP_CUTDOWN;
		default: //UP_CUTDOWN:
			return UP_PING;
	}
}
static ui_uplink_msg_t dec_uplink_msg(ui_uplink_msg_t in)
{
	switch(in){
		case UP_PING:
			return UP_CUTDOWN;
		case UP_CAL:
			return UP_PING;
		default: //UP_CUTDOWN:
			return UP_CAL;
	}
}


static void redraw_timers(void)
{
	redraw_timers_flag = 0;

	char buff[17];

	if (ui_state == MENU_NONE){
		switch(ui_main){
			case PAYLOAD:

				break;
			case RSSI:
				snprintf(buff,7,"%lus   ",(unsigned long)time_since_last);
				screen_write_text(buff,ROW_BOT | 9);
				break;
			case POSITION:

				break;
			case RADIO_STATUS:
				if (last_rstat > 0){
					snprintf(buff,7,"%lus   ",(unsigned long)time_since_last_rstat);
					screen_write_text(buff,ROW_BOT | 13);
				}
				break;
			case RADIO_FREQ:

				break;
			case BATT_VOLTAGE:
				snprintf(buff,8,"%lu mV   ",(((unsigned long)adc_batt)*161)/100);
				screen_write_text(buff,9);
				//snprintf(buff,7,"%lu mV   ",((unsigned long)adc_input)*161/100);
				//screen_write_text(buff,ROW_TOP | 8);
				break;
			default:
				break;
		}
	}
}

static void redraw_screen(void)
{
	redraw_screen_flag = 0;
	screen_clear_row(ROW_TOP);
	screen_clear_row(ROW_BOT);

	//the on-off-on flicker wont be noticeable
	gpio_clear(LED_PST_PORT,LED_PST_PIN);
	gpio_clear(LED_MENU_PORT,LED_MENU_PIN);

	if ((ui_status_led&1) == 0){
		gpio_clear(LED_RED_PORT,LED_RED_PIN);
		gpio_set(LED_GRN_PORT,LED_GRN_PIN);
	}
	else{
		gpio_set(LED_RED_PORT,LED_RED_PIN);
		gpio_clear(LED_GRN_PORT,LED_GRN_PIN);
	}



	char buff[17];

	if (ui_state == MENU_NONE){
		switch(ui_main){
			case PAYLOAD:
				screen_write_text(ui_payload,0);
				if (ui_time > 0)
				{
					uint8_t hours = ui_time/(60*60);
					uint16_t mins = ui_time%(60*60);
					uint8_t secs = mins%60;
					mins = mins / 60;
					snprintf(buff,10,"%02u:%02u:%02u",hours,mins,secs);
					screen_write_text(buff,ROW_BOT+8);
				}
				else
					screen_write_text("--:--:--",ROW_BOT+8);
				if (ui_sequence >= 0){
					snprintf(buff,7,"%lu",(unsigned long)ui_sequence);
					screen_write_text(buff,ROW_BOT);
				}
				if ((ui_position_valid_flags & (1<<1)) == 0)
					screen_write_char(ui_icon_warning,ROW_BOT+7);
				break;
			case RSSI:
				screen_write_text("RSSI: ",0);
				if (ui_rssi < 0){
					uint8_t r = ((ui_rssi+130)/10);
					snprintf(buff,r+2,"###########");
				}
				screen_write_text(buff,6);
				snprintf(buff,17,"Last Rx: %lus",(unsigned long)time_since_last);
				screen_write_text(buff,ROW_BOT);
				break;
			case POSITION:
				//if (ui_position_valid_flags & (1<<2))
					screen_write_text(ui_lati,0);
				//if (ui_position_valid_flags & (1<<3))
					screen_write_text(ui_longi,ROW_BOT);
				//if (ui_position_valid_flags & (1<<4))
					snprintf(buff,6,"%lim",(long)ui_alt);
				//else
				//	snprintf(buff,6,"%-m",(long)ui_alt);
				screen_write_text(buff,ROW_BOT | 10);
				if ((ui_position_valid_flags & ((1<<2)|(1<<3)|(1<<4))) != ((1<<2)|(1<<3)|(1<<4)))
					screen_write_char(ui_icon_warning,ROW_TOP+15);
				break;
			case RADIO_STATUS:
				screen_write_text("Radio Status:",0);
				if (last_rstat == 1)
					snprintf(buff,16,"Last no CRC: ");
				else if (last_rstat == 2)
					snprintf(buff,16,"Last packet: ");
				if (last_rstat > 0)
					screen_write_text(buff,ROW_BOT);
				break;
			case RADIO_FREQ:
				snprintf(buff,17,"Freq: ");
				screen_write_text(buff,0);
				print_frequency(buff,s_lora.frequency,5);
				screen_write_text(buff,6);
				snprintf(buff,17,"Offset: %liHz",(long)ui_offset);
				screen_write_text(buff,ROW_BOT);
				break;
			default: //BATT_VOLTAGE
				screen_write_text("Battery: ",0);
				break;
		}
	}
	else if ((ui_state == MENU_OPTIONS) || (ui_state == MENU_OPTIONS_WRITE)){
		gpio_set(LED_MENU_PORT,LED_MENU_PIN);
		switch(ui_menu){
			case MENU_FREQ:
				screen_write_text("M1 Frequency",0);
				print_frequency(buff,s_currently_unsaved.frequency,6);
				screen_write_text(buff,ROW_BOT);
				break;
			case MENU_SF:
				screen_write_text("M2 Spreading Ftr",0);
				snprintf(buff,17,"SF%i",s_currently_unsaved.spreading_factor);
				screen_write_text(buff,ROW_BOT);
				break;
			case MENU_BW:
				screen_write_text("M3 Bandwidth",0);
				screen_write_text(get_bandwidth(s_currently_unsaved.bandwidth),ROW_BOT);
				screen_write_text(" kHz",ROW_BOT + 4);
				break;
			case MENU_TRACK:
				screen_write_text("M4 Track Offset",0);
				if (s_currently_unsaved.enable_frequency_tracking)
					screen_write_text("Enabled",ROW_BOT);
				else
					screen_write_text("Disabled",ROW_BOT);
				break;
			case MENU_UPLINK:
				screen_write_text("M5 Send Uplink",0);
				if (ui_selected_uplink == UP_CAL)
					screen_write_text("CALIBRATE",ROW_BOT);
				if (ui_selected_uplink == UP_PING)
					screen_write_text("PING",ROW_BOT);
				if (ui_selected_uplink == UP_CUTDOWN)
					screen_write_text("CUTDOWN",ROW_BOT);
				break;
			default: //MENU_AUTOPING
				screen_write_text("M6 Autoping",0);
				if (auto_ping)
					screen_write_text("Enabled",ROW_BOT);
				else
					screen_write_text("Disabled",ROW_BOT);
				break;
		}
		if (ui_state == MENU_OPTIONS_WRITE)
			screen_write_char(ui_icon_dot,ROW_BOT+15);
	}
	else if (ui_state == MENU_PRESET){
		gpio_set(LED_PST_PORT,LED_PST_PIN);
		snprintf(buff,17,"LOAD P%i: SF%i,",menu_preset_sel,presets[menu_preset_sel-1].spreading_factor);
		screen_write_text(buff,0);
		screen_write_text("BW ",ROW_BOT);
		screen_write_text(get_bandwidth(presets[menu_preset_sel-1].bandwidth),ROW_BOT+3);
		print_frequency(buff,presets[menu_preset_sel-1].frequency,3);
		screen_write_text(", ",ROW_BOT+7);
		screen_write_text(buff,ROW_BOT+9);
	}
	else if (ui_state == MENU_PRESET_WRITE){
		gpio_set(LED_PST_PORT,LED_PST_PIN); //ideally flash
		snprintf(buff,17,"Write to P%i?",menu_preset_sel);
		screen_write_text(buff,0);
		if (preset_lock[menu_preset_sel-1])
			screen_write_text("- Locked",ROW_BOT);
		else
			screen_write_text("@ Unlocked",ROW_BOT);

	}
	else if (ui_state == MENU_OPTIONS_CONFIRM)
	{
		screen_write_text("WAITING TO ",ROW_TOP);
		if (ui_selected_uplink == UP_CAL)
			screen_write_text("CAL",11);
		if (ui_selected_uplink == UP_PING)
			screen_write_text("PING",11);
		if (ui_selected_uplink == UP_CUTDOWN)
			screen_write_text("CUT",11);
		screen_write_text("OK = Immediate",ROW_BOT);
	}

}

static uint8_t print_frequency(char* buff, uint32_t f, uint8_t digits)
{
	//uint64_t f;
	//f = (uint64_t) regfreq*32000000L;
	//f = f >> 19;

	uint16_t mhz = f / 1000000L;
	uint32_t khz = f % 1000000L;

	return snprintf(buff,digits+5,"%i.%06lu",mhz,(unsigned long)khz);
}

static void change_setting(int8_t direction)
{
	int8_t i;
	if (update_settings_flag)
		return;
	if ((ui_state == MENU_OPTIONS_WRITE))
	{
		switch(ui_menu){
			case MENU_FREQ:
				if (rotary_acceleration > 200)
					s_currently_unsaved.frequency += (5000*direction);
				else
					s_currently_unsaved.frequency += (250*direction);
				redraw_screen_flag = 1;
				break;
			case MENU_SF:
				i = s_currently_unsaved.spreading_factor + direction;
				if (i < 6)
					i = 6;
				else if (i > 12)
					i = 12;
				s_currently_unsaved.spreading_factor = i;
				redraw_screen_flag = 1;
				break;
			case MENU_BW:
				i = (s_currently_unsaved.bandwidth >> 4) + direction;
				if (i < 0)
					i = 0;
				else if (i > 9)
					i = 9;
				s_currently_unsaved.bandwidth = i<<4;
				redraw_screen_flag = 1;
				break;
			case MENU_TRACK:
				if (direction != 0)
					s_currently_unsaved.enable_frequency_tracking = (~s_currently_unsaved.enable_frequency_tracking) & 1;
				redraw_screen_flag = 1;
				break;
			case MENU_UPLINK:
				if (direction > 0)
					ui_selected_uplink = inc_uplink_msg(ui_selected_uplink);
				else if (direction < 0)
					ui_selected_uplink = dec_uplink_msg(ui_selected_uplink);
				redraw_screen_flag = 1;
				break;
			default: // MENU_AUTOPING:
				if (direction != 0)
					auto_ping = (~auto_ping) & 1;
				redraw_screen_flag = 1;
				break;
		}
	}
}

static void load_settings(radio_lora_settings_t s)
{
	//NEED TO SET/UNSET SHORT PACKET BIT?
	update_settings_flag = 0;
	redraw_screen_flag = 1;
	radio_standby();
	radio_write_lora_config(&s);
	radio_pa_off();
	radio_lna_max();
	radio_set_frequency_frq(s.frequency);
	radio_set_continuous_rx();
}

static uint16_t get_radio_bandwidth(radio_lora_settings_t s)
{
	switch(s.bandwidth){
		case(0<<4):
			return 78;
		case(1<<4):
			return 104;
		case(2<<4):
			return 156;
		case(3<<4):
			return 208;
		case(4<<4):
			return 312;
		case(5<<4):
			return 417;
		case(6<<4):
			return 625;
		case(7<<4):
			return 1250;
		case(8<<4):
			return 2500;
		case(9<<4):
			return 5000;
		default:
			return 0;
	}
}

static char* get_bandwidth(uint8_t bandwidth)
{
	switch(bandwidth){
		case(0<<4):
			return " 7.8";
		case(1<<4):
			return "10.4";
		case(2<<4):
			return "15.6";
		case(3<<4):
			return "20.8";
		case(4<<4):
			return "31.2";
		case(5<<4):
			return "41.7";
		case(6<<4):
			return "62.5";
		case(7<<4):
			return " 125";
		case(8<<4):
			return " 250";
		case(9<<4):
			return " 500";
		default:
			return ":(";
	}
}

static radio_lora_settings_t read_settings_flash(uint8_t pos)
{
	uint32_t size_each = sizeof(radio_lora_settings_t);
	radio_lora_settings_t out;

	uint32_t out_ptr = (uint32_t)(&out);
	uint32_t flash_ptr = (uint32_t)(FLASH_STORAGE_ADDR + pos*size_each);

	//word align
	if (size_each & 0x3)
		size_each += (4-(size_each & 0x3));


	int i;
	for (i = 0; i < (size_each>>2); i++)
	{
		*(uint32_t*)out_ptr = *(uint32_t*)flash_ptr;
		flash_ptr += 4;
		out_ptr += 4;
	}

	return out;

}

//returns 0 for error
static uint8_t write_settings_flash(radio_lora_settings_t in, uint8_t pos)
{
	uint32_t size_each = sizeof(radio_lora_settings_t);

	uint32_t in_ptr = (uint32_t)(&in);
	uint32_t flash_ptr = (FLASH_STORAGE_ADDR + pos*size_each);

	//word align
	if (size_each & 0x3)
		size_each += (4-(size_each & 0x3));


	int i;
	for (i = 0; i < (size_each>>2); i++)
	{
		/*programming word data*/
		flash_program_word(flash_ptr, *(uint32_t*)in_ptr);
		if(flash_get_status_flags() != FLASH_SR_EOP)
			return 1;

		///*verify if correct data is programmed*/
		//if(*((uint32_t*)(current_address+iter)) != *((uint32_t*)(input_data + iter)))
		//	return FLASH_WRONG_DATA_WRITTEN;

		flash_ptr += 4;
		in_ptr += 4;
	}

	return 0;

}

//returns 0 for error
static uint8_t erase_settings_page(void)
{
	flash_erase_page(FLASH_STORAGE_ADDR);
	if(flash_get_status_flags() != FLASH_SR_EOP)
		return 0;
	else
		return 1;
}

static void switch_to_rx(void)
{
	//NEED TO SET/UNSET SHORT PACKET BIT?
	radio_standby();
	radio_pa_off();
	radio_lna_max();
	radio_set_frequency_frq(s_lora.frequency);
	radio_write_lora_config((radio_lora_settings_t *)&s_lora);
	radio_write_single_reg(REG_IRQ_FLAGS,0xFF);
	radio_set_continuous_rx();
}

static void switch_to_tx(void)
{
	//NEED TO SET/UNSET SHORT PACKET BIT?
	radio_lora_settings_t temp = s_lora;
	temp.bandwidth = BANDWIDTH_62_5K;
	radio_sleep();
	_delay_ms(10);
	radio_set_frequency_frq(s_lora.frequency);
	radio_write_lora_config((radio_lora_settings_t *)&temp);
	radio_standby();
	radio_high_power();


}

static void send_uplink(void)
{
	switch_to_tx();
	uint8_t i=snprintf(buff_tx,60,"PINGPINGPING");
	radio_tx_packet((uint8_t*)buff_tx,i);
	_delay_ms(200);
	while(lora_in_progress())
		_delay_ms(50);
	switch_to_rx();
}

static void low_batt(void)
{
	//TURN OFF ALL THE THINGS!
	gpio_clear(LED_PST_PORT,LED_PST_PIN);
	gpio_clear(LED_MENU_PORT,LED_MENU_PIN);
	gpio_set(LED_RED_PORT,LED_RED_PIN);
	gpio_clear(LED_GRN_PORT,LED_GRN_PIN);
	screen_clear_row(ROW_TOP);
	screen_clear_row(ROW_BOT);
	screen_write_text("CONNECT CHARGER",0);
	backlight_off();
	radio_sleep();
	bluetooth_sleep();
	systick_counter_disable();
	rcc_periph_clock_disable(RCC_ADC);
	rcc_periph_clock_disable(RCC_USART1);
	rcc_periph_clock_disable(RCC_USART2);
	pwr_set_stop_mode();
	while(1);
}

