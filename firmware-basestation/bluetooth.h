#define B_WAKE_HW_PORT GPIOA
#define B_WAKE_HW_PIN (1<<4)
#define RCC_B_WAKE_HW_PORT RCC_GPIOA

#define B_WAKE_SW_PORT GPIOB
#define B_WAKE_SW_PIN (1<<11)
#define RCC_B_WAKE_SW_PORT RCC_GPIOB

#define B_CMD_PORT GPIOB
#define B_CMD_PIN (1<<1)
#define RCC_B_CMD_PORT RCC_GPIOB

#define B_USART USART2
#define RCC_B_USART RCC_USART2
#define B_USART_PORT GPIOA
#define B_USART_TX_PIN (1<<2)
#define B_USART_RX_PIN (1<<3)

#define B_USART_AFn GPIO_AF1

#define BT_TIMEOUT_PERIOD 10


typedef enum {START_ADVERT, UPDATE, UPDATE_NOTIFY, CONFIGURE, DISCONNECT, LIST, NONE} bt_query_t;
typedef enum {IDLE, ADVERTISING, CONNECTED, DISCONNECTED} bt_status_t;




void bluetooth_init(void);
void bluetooth_wakeup(void);
void bluetooth_configure(void);
uint8_t check_characteristic_handle(void);
void bt_timer_10ms_tick(void);
uint8_t bt_waiting_for_response(void);
void bt_process_line_rx(char *buffin, uint16_t line_start, uint16_t line_end, uint16_t buff_len);
uint8_t start_bt_telem_send(uint8_t *in, uint16_t len);
uint8_t continue_bt_telem_send(void);
void bt_update_characteristic(uint8_t *data, uint8_t len, const char *uuid);
bt_status_t bt_get_status(void);
uint8_t start_advertise(void);

