#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <string.h>

#include "bluetooth.h"

bt_status_t bt_status = IDLE;

bt_query_t bt_last_query;
volatile uint8_t waiting_for_response = 0;

static volatile uint8_t bt_timeout = 0;

#define BT_BUFF_LEN 50
//static volatile char bt_uart_buffer[BT_BUFF_LEN];
//static volatile uint8_t bt_buffer_ptr = 0;

static volatile uint8_t bt_query_result = 0;

static volatile uint8_t *telem_ptr;
static volatile uint16_t telem_len_remain;
static volatile uint8_t telem_packet_counter;  //first nibble = packet id, second nibble = total packets-1

const char private_service[] = "0000200AFFFF1000800052004D7A1D0B";
const char private_characteristic[]        = "00001501FFFF1000800052004D7A1D0B";
const char private_characteristic_freq[]   = "000015F1FFFF1000800052004D7A1D0B";
const char private_characteristic_offset[] = "000015F2FFFF1000800052004D7A1D0B";
const char bt_end[] = "END";
const char bt_ack[] = "AOK";
const char bt_err[] = "ERR";
const char bt_conn[] = "Connected";
const char bt_disconn[] = "Connection End";

uint8_t auto_re_advertise = 1;


static void bt_send_string(uint32_t usart, const char *buff);
static uint8_t strncmp_circ(char *in1, const char *in2, uint16_t start1, uint16_t wrap1, uint16_t len2);

void bluetooth_init(void)
{
	rcc_periph_clock_enable(RCC_B_WAKE_SW_PORT);
	rcc_periph_clock_enable(RCC_B_WAKE_HW_PORT);
	rcc_periph_clock_enable(RCC_B_CMD_PORT);


	gpio_mode_setup(B_WAKE_SW_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, B_WAKE_SW_PIN);
	gpio_mode_setup(B_WAKE_HW_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, B_WAKE_HW_PIN);
	gpio_mode_setup(B_CMD_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, B_CMD_PIN);

	//uart

	rcc_periph_clock_enable(RCC_B_USART);
	gpio_mode_setup(B_USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, B_USART_TX_PIN|B_USART_RX_PIN);
	gpio_set_af(B_USART_PORT, GPIO_AF1, B_USART_TX_PIN|B_USART_RX_PIN);
	usart_set_baudrate(B_USART, 115200);//9600 );
	usart_set_databits(B_USART, 8);
	usart_set_stopbits(B_USART, USART_CR2_STOP_1_0BIT);
	usart_set_mode(B_USART, USART_MODE_TX_RX);
	usart_set_parity(B_USART, USART_PARITY_NONE);
	usart_set_flow_control(B_USART, USART_FLOWCONTROL_NONE);
	usart_enable(B_USART);

}

void bluetooth_sleep(void)
{
	gpio_clear(B_WAKE_HW_PORT,B_WAKE_HW_PIN);
	gpio_clear(B_WAKE_SW_PORT,B_WAKE_SW_PIN);
}

void bluetooth_wakeup(void)
{
	gpio_clear(B_CMD_PORT,B_CMD_PIN);
	gpio_set(B_WAKE_HW_PORT,B_WAKE_HW_PIN);
	gpio_set(B_WAKE_SW_PORT,B_WAKE_SW_PIN);

	int j;
	for(j=0; j<3000; j++)
		__asm__("nop");

	gpio_set(B_CMD_PORT,B_CMD_PIN);

}

void bluetooth_configure(void)
{
	//sets the private service/characteristics
	bt_send_string(B_USART,"SS,C0000001\r\n");
	_delay_ms(100);
	bt_send_string(B_USART,"PZ\r\n");
	_delay_ms(100);
	bt_send_string(B_USART,"PS,");
	bt_send_string(B_USART,private_service);
	bt_send_string(B_USART,"\r\n");
	_delay_ms(100);
	bt_send_string(B_USART,"PC,");
	bt_send_string(B_USART,private_characteristic);
	bt_send_string(B_USART,",22,14\r\n");
	_delay_ms(100);
	bt_send_string(B_USART,"U\r\n");
	_delay_ms(100);
	bt_send_string(B_USART,"R,1\r\n");

}

static void bt_send_string(uint32_t usart, const char *buff)
{
	uint32_t i=0;
	while (buff[i])
		usart_send_blocking(usart, buff[i++]);
}

//return: 0 - everything done
//return: 1 - still needs more send commands to send the telem
uint8_t start_bt_telem_send(uint8_t *in, uint16_t len)
{

	//work out total number of packets needed
	if (len > 19*16)
		return 0;
	telem_packet_counter = len/19;
	uint8_t i,l;
	if (len%19 > 0)
		telem_packet_counter++;
	telem_packet_counter--;

	uint8_t buff[20];
	if (len < 19)
		l = len;
	else
		l = 19;

	//fill out packet to send
	for (i = 0; i < l; i++)
	{
		buff[i+1] = in[i];
	}
	buff[0] = telem_packet_counter;

	//send first 19 bytes. The first byte is the packet id
	bt_update_characteristic(buff,l+1,private_characteristic);
	bt_last_query = UPDATE_NOTIFY;  //overwrite this as we want to know when the phone has read it


	telem_len_remain = len - l;

	if (telem_len_remain == 0)
		return 0;

	telem_ptr = &in[l];
	return 1;
}

bt_status_t bt_get_status(void)
{
	return bt_status;
}

uint8_t continue_bt_telem_send(void)
{
	//work out total number of packets needed

	telem_packet_counter += 0x10;
	uint8_t i,l;


	uint8_t buff[20];
	if (telem_len_remain < 19)
		l = telem_len_remain;
	else
		l = 19;

	//fill out packet to send
	for (i = 0; i < l; i++)
		buff[i+1] = *telem_ptr++;
	buff[0] = telem_packet_counter;


	//send first 19 bytes. The first byte is the packet id
	bt_update_characteristic(buff,l+1,private_characteristic);
	bt_last_query = UPDATE_NOTIFY;  //overwrite this as we want to know when the phone has read it


	telem_len_remain = telem_len_remain - l;

	if (telem_len_remain == 0)
		return 0;
	return 1;

}

void bt_update_characteristic(uint8_t *data, uint8_t len, const char *uuid)
{
	bt_last_query = UPDATE;
	uint8_t i,d;
	bt_send_string(B_USART,"SUW,");
	bt_send_string(B_USART,uuid);
	bt_send_string(B_USART,",");
	for (i=0; i < len; i++)
	{
		d = (data[i] & 0xF0) >> 4;
		if (d <= 9)
			d += 48;
		else
			d+= 55;
		usart_send_blocking(B_USART, d);
		d = data[i] & 0xF;
		if (d <= 9)
			d += 48;
		else
			d+= 55;
		usart_send_blocking(B_USART, d);
	}
	bt_send_string(B_USART,"\r\n");
	bt_query_result = 0;
	waiting_for_response = 1;
}

uint8_t start_advertise(void)
{
	bt_send_string(B_USART,"A\r\n");
	bt_last_query = START_ADVERT;
	bt_status = ADVERTISING;
	bt_query_result = 0;
	waiting_for_response = 1;
	bt_timeout = BT_TIMEOUT_PERIOD;
	//while(waiting_for_response);
	//return bt_query_result;
	return 0;
}

uint8_t check_characteristic_handle(void)
{
	bt_send_string(B_USART,"LS\r\n");
	bt_last_query = LIST;
	bt_query_result = 0;
	waiting_for_response = 1;
	bt_timeout = BT_TIMEOUT_PERIOD;
	//while(waiting_for_response);
	//return bt_query_result;
	return 0;
}



void bt_timer_10ms_tick(void)
{

	if (bt_timeout)
	{
		bt_timeout--;
		if (bt_timeout == 0)
			waiting_for_response = 0;
	}
}


void bt_process_line_rx(char *buffin, uint16_t line_start, uint16_t line_end, uint16_t buff_len)
{
	uint16_t ptr = line_start;
	uint16_t count = 0;			//number of non whitespace characters received
	uint16_t text_start = 0;	//first offset address of non whitespace
	char in;
	line_end++;
	if (line_end == buff_len)
		line_end = 0;
	while(ptr != line_end)
	{
		in = buffin[ptr];
		if ((in != ' ') || count > 0)
			count++;
		else
			text_start++;

		if (waiting_for_response)
		{
			switch(bt_last_query)
			{
				case LIST:
					if (count == 3) //see if we have 'END'
					{
						//if ((bt_uart_buffer[0] == 'E') &&
						//		(bt_uart_buffer[1] == 'N') &&
						//		(bt_uart_buffer[2] == 'D'))
						if (strncmp_circ(buffin, bt_end, line_start+text_start, buff_len, 4))
						{
							waiting_for_response = 0;
							bt_last_query = NONE;
						}
					}

					if (count == 32)
					{
						int r1 = strncmp_circ(buffin, private_service, line_start+text_start, buff_len, 32); //strncmp(bt_uart_buffer, private_service, 32);
						int r2 = strncmp_circ(buffin, private_characteristic, line_start+text_start, buff_len, 32); //strncmp(bt_uart_buffer, private_characteristic, 32);
						if (r1 == 0)
							bt_query_result |= 1;
						if (r2 == 0)
							bt_query_result |= 2;
					}


					break;

				case START_ADVERT:
					if (count == 3) //see if we have 'AOK'
					{
						if (strncmp_circ(buffin, bt_ack, line_start+text_start, buff_len, 4))
						{
							waiting_for_response = 0;
							bt_query_result = 1;
							bt_status = ADVERTISING;
						}
						if (strncmp_circ(buffin, bt_err, line_start+text_start, buff_len, 4))
						{
							waiting_for_response = 0;
							bt_query_result = 0;
						}
					}
					break;

				case UPDATE_NOTIFY:
					if (count == 3) //see if we have 'AOK'
					{
						if (strncmp_circ(buffin, bt_ack, line_start+text_start, buff_len, 4))
						{
							bt_query_result++;
							if (bt_query_result == 2)    //we need one AOK for command rx, and one AOK for phone got info
								waiting_for_response = 0;
						}
					}
					break;
				default:

					break;
			}
		}

		//look for things like Connected, Connection End
		if (count == 9){
			if (strncmp_circ(buffin, bt_conn, line_start+text_start, buff_len, 9))
				bt_status = CONNECTED;
		}
		if (count == 14){
			if (strncmp_circ(buffin, bt_disconn, line_start+text_start, buff_len, 14)){
				bt_status = DISCONNECTED;
				if (auto_re_advertise == 1)
				{

				}
			}
		}


		ptr++;
		if (ptr == buff_len)
			ptr = 0;
	}
}


uint8_t bt_waiting_for_response(void)
{
	return waiting_for_response;
}

//in1 is a circular buffer
//in2 is a \0 terminated char[]
//start1 is the offset of in1 for which the string to compare starts
//end1 is the length of in1 when the ptr needs to wrap to 0
//len2 is the maximum length of in2
//returns 1 if the same, else 0
static uint8_t strncmp_circ(char *in1, const char *in2, uint16_t start1, uint16_t wrap1, uint16_t len2)
{
	uint16_t ptr1 = start1;
	uint16_t ptr2 = 0;


	while(ptr2 < len2 && in2[ptr2] && in1[start1])
	{
		if (in1[ptr1] != in2[ptr2])
			return 0;

		ptr2++;
		ptr1++;
		if (ptr1 == wrap1)
			ptr1 = 0;
	}

	return 1;
}
/*
void _delay_ms(const uint32_t delay)
{
    uint32_t i, j;

    for(i=0; i< delay; i++)
        for(j=0; j<1000; j++)
            __asm__("nop");
}
*/
