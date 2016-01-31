#define RTTY_SENTENCE 0xFF

uint16_t crc_xmodem_update (uint16_t crc, uint8_t data);
uint16_t calculate_crc16 (char *input);
uint16_t calculate_ublox_crc(uint8_t *input, uint16_t len);
