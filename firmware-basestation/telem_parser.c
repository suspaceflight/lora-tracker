#include <stdio.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include "telem_parser.h"


//returns:
//bit0 - parsed complete
//bit1 - valid time
//bit2 - valid lat
//bit3 - valid long
//bit4 - altitude
uint8_t parse_ascii(char *buff, uint16_t max_in_len, char *call, uint32_t *seq,
		uint32_t *time, char *lati, char *longi, int32_t *alt, uint8_t max_out_len)
{

	char b[8];
	uint8_t out = 0;

	//$PAYLOAD,43,14:48:12,52.32523,-0.053243,24000,.....,*ABCD
	//checksum assumed correct


	uint16_t i = 0;
	uint16_t j = 0;
	//1. look for a '$' proceeded by  not a '$'
	uint8_t found_start = 0;
	for (; i < max_in_len; i++)
	{
		if (found_start && (buff[i] != '$'))
			break;
		if ((found_start == 0) && (buff[i] == '$'))
			found_start = 1;
	}

	//2. extract callsign
	//iterate until ','
	j=0;
	while((i < max_in_len) && (j < max_out_len) && (buff[i] != ','))
	{
		*call++ = buff[i];
		i++;
		j++;
	}
	*call = '\0';
	i++;

	//3. extract seq
	//iterate until ','
	j=0;
	while((i < max_in_len) && (j < 8-1) && (buff[i] != ','))
	{
		b[j] = buff[i];
		i++;
		j++;
	}
	b[j] = '\0';
	i++;
	*seq = atoi(b);

	//4. extract time
	//check for xx:xx:xx
	j=0;
	if (buff[i] != ',')  //check whether no time in packet
	{
		if (i > max_in_len + 8)
			return 0;
		if (buff[i+2] != ':')
			return 0;
		if (buff[i+5] != ':')
			return 0;
		b[0] = buff[i];
		b[1] = buff[i+1];
		b[2] = '\0';
		*time = atoi(b)*60*60;
		b[0] = buff[i+3];
		b[1] = buff[i+4];
		*time += atoi(b)*60;
		b[0] = buff[i+6];
		b[1] = buff[i+7];
		*time += atoi(b);
		i += 8;
		out |= (1<<1);
	}
	i++;

	//5. extract lat
	//iterate until ','
	j=0;
	if (buff[i] != ','){
		while((i < max_in_len) && (j < max_out_len) && (buff[i] != ','))
		{
			*lati++ = buff[i];
			i++;
			j++;
		}
		*lati = '\0';
		out |= (1<<2);
	}
	i++;

	//6. extract long
	//iterate until ','
	j=0;
	if (buff[i] != ','){
		while((i < max_in_len) && (j < max_out_len) && (buff[i] != ','))
		{
			*longi++ = buff[i];
			i++;
			j++;
		}
		*longi = '\0';
		out |= (1<<3);
	}
	i++;

	//7. extract altitude
	//iterate until ','
	j=0;
	if (buff[i] != ','){
		while((i < max_in_len) && (j < 8-1) && (buff[i] != ','))
		{
			b[j] = buff[i];
			i++;
			j++;
		}
		b[j] = '\0';
		out |= (1<<4);
		*alt = atoi(b);
	}
	i++;

	out |= (1<<0);

	return out;
}

