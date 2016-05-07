#include <stdio.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include "telem_parser.h"

#include "cmp.h"

uint8_t hb_buf_ptr = 0;
uint16_t hb_ptr_max = 20;
//static bool read_bytes(void *data, size_t sz, FILE *fh) {
//    return fread(data, sizeof(uint8_t), sz, fh) == (sz * sizeof(uint8_t));
//}

static bool file_reader(cmp_ctx_t *ctx, void *data, size_t limit) {
	if (hb_buf_ptr + limit >= hb_ptr_max)
    	return 0;
	uint32_t i = 0;
	for(i = 0; i < limit; i++){
		*(uint8_t *)data = ((uint8_t *)(ctx->buf))[hb_buf_ptr];
		data++;
		hb_buf_ptr++;
	}

	//data = ctx->buf+hb_buf_ptr;
	//hb_buf_ptr += limit;
	return 1;
	//return read_bytes(data, limit, (FILE *)ctx->buf);
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


uint8_t parse_habpack(char *buff, uint16_t max_in_len, char *call, uint32_t *seq,
		uint32_t *time, int32_t *lati, int32_t *longi, int32_t *alt, uint8_t max_out_len)
{
	cmp_ctx_t cmp;
	uint32_t map_size;
	uint32_t array_size;
	uint64_t map_id;
	uint32_t i;
	uint8_t out = 0;
	cmp_object_t obj;
	uint64_t tu64;
	int64_t ts64;

	cmp_init(&cmp, (void*)buff, file_reader, file_writer);
	hb_buf_ptr = 0;
	hb_ptr_max = max_in_len;

	//check we have a map
	if (!cmp_read_map(&cmp, &map_size))
	        return out;

	for (i=0; i < map_size; i++)
	{
		if (!(cmp_read_uinteger(&cmp, &map_id)))
			return out;

		switch(map_id){
			case 0: //callsign
				//tu64 = max_out_len;
				if (!cmp_read_str(&cmp, call, (uint32_t *)(&max_out_len)))
				        return out;
				break;
			case 1: //count
				if (!(cmp_read_uinteger(&cmp, &tu64)))
					return out;
				*seq = (uint32_t)tu64;
				break;
			case 2: //time
				if (!(cmp_read_uinteger(&cmp, &tu64)))
					return out;
				out |= (1<<1);
				*time = (uint32_t)tu64;
				break;
			case 3: //position
				if (!cmp_read_array(&cmp, &array_size))
					return out;
				if (array_size != 3)
					return out;
				if (!(cmp_read_sinteger(&cmp, &ts64)))
					return out;
				*lati = (int32_t)ts64;
				out |= (1<<2);
				if (!(cmp_read_sinteger(&cmp, &ts64)))
					return out;
				*longi = (int32_t)ts64;
				out |= (1<<3);
				if (!(cmp_read_sinteger(&cmp, &ts64)))
					return out;
				*alt = (int32_t)ts64;
				out |= (1<<4);
				break;
			//case 4: //satellites
			//	break;
			//case 5: //lock type
			//	break;
			default:
				if (!cmp_read_object(&cmp, &obj))
					return out;
				//TODO: if string type, step through string
				break;
		}

		//if everything useful is parsed, exit early
		//this saves having to deal with habpack fully
		if (out == 0x1E)
			return out | 1;
	}

	return out | 1;
}

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

