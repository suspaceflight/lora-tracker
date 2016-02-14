#include <avr/io.h>
#include <util/delay.h>

#include "radio.h"

uint8_t buff[64] = {0};

int main(void){

	DDRD |= (1<<2);

	radio_init();


	while (radio_read_version() == 0xFF)
		_delay_ms(500);
	radio_lora_settings_t s_lora;
	radio_high_power();
	radio_set_frequency_frreg(FREQ_434_400);
	s_lora.spreading_factor = 11;
        s_lora.bandwidth = BANDWIDTH_20_8K;
        s_lora.coding_rate = CODING_4_5;
        s_lora.implicit_mode = 0;
        s_lora.crc_en = 1;
        s_lora.low_datarate = 1;    //todo: this


	while(1){
		PORTD |= (1<<2);
		_delay_ms(1000);
		PORTD &= ~(1<<2);
		_delay_ms(1000);
		radio_sleep();
		_delay_ms(10);
		radio_write_lora_config(&s_lora);
		radio_standby();
		radio_high_power();
		radio_tx_packet(buff,64);
		_delay_ms(200);
		while(lora_in_progress())
			_delay_ms(50);
		radio_read_version();
	}


	return 0;
}
