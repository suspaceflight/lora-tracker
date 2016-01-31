#include <inttypes.h>


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

uint16_t calculate_ublox_crc(uint8_t *input, uint16_t len)
{
	uint8_t a = 0;
	uint8_t b = 0;
	uint16_t i;
	for (i=0;i<len;i++)
	{
		a = a + *input++;
		b = b + a;
	}
	return (a << 8) | b;
}
