#define BACKLIGHT_PORT GPIOC
#define BACKLIGHT_PIN (1<<15)
#define RCC_BACKLIGHT_PORT RCC_GPIOC
#define RCC_S_I2C RCC_I2C1

#define S_I2C_PORT GPIOB
#define S_I2C_SCL_PIN (1<<8)
#define S_I2C_SDA_PIN (1<<9)

#define S_I2C_AFn GPIO_AF1

#define RCC_RESET_PORT RCC_GPIOC
#define S_RESET_PORT GPIOC
#define S_RESET_PIN (1<<13)


#define S_RCC_I2C RCC_I2C1
#define S_I2C I2C1


void screen_init(void);
void screen_on(void);
void backlight_on(void);
void backlight_off(void);
void screen_write_i2c(uint32_t i2c, uint8_t i2c_addr, uint8_t size, uint8_t *data);
void screen_test(void);
void screen_write_text(char *text, uint8_t position);
void screen_clear_row(uint8_t row);

#define ROW_TOP 0
#define ROW_BOT 0x40

#define S_WRITE           0x00
#define S_READ            0x01
#define S_CMDSEND         0x00
#define S_DATASEND        0x40
#define S_MULTIPLE        0x80
#define S_LCD_CLEAR       0x01
#define S_LCD_HOME        0x02
#define S_TOP_ROW         0x00
#define S_BOT_ROW         0x40
#define S_DDRAM_SIZE      0x28
#define S_SET_DDRAM       0x80
#define S_SET_CGRAM       0x40
#define S_LCD_MODE        0x08
#define S_LCD_ON          0x04
#define S_CURSOR          0x02
#define S_BLINK           0x01
#define S_ENTRY_MODE      0x04
#define S_INC             0x02
#define S_DEC             0x00
#define S_SHIFT           0x01
#define S_INSTR_TABLE     0x38
#define S_IS0             0x00
#define S_IS1             0x01
#define S_LCD_SHIFT       0x18
#define S_INT_OSC         0x10
#define S_BS              0x08
#define S_F_183HZ         0x04
#define S_BIAS_5          0x00
#define S_POWER           0x50
#define S_ICON_ON         0x08
#define S_BOOST_ON        0x04
#define S_CONTRAST        0x70
#define S_FOLLOWER        0x60
#define S_FON             0x08
