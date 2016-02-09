#include <avr/io.h>
#include <util/delay.h>

#include "radio.h"

int main(void){

	DDRD |= (1<<2);

	
	
	while(1){
		PORTD |= (1<<2);
		_delay_ms(1000);
		PORTD &= ~(1<<2);
		_delay_ms(1000);
		radio_init();
	}


	return 0;
}
