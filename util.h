/*
 * Hockey table
 * util.h
 */

 #ifndef UTIL_H
 #define UTIL_H
 
 #define SYSCLK 32000000L

// LCD Display
#define LCD_E_PORT GPIOA
#define LCD_E_PIN 15
#define LCD_RS_PORT GPIOB
#define LCD_RS_PIN 3
#define LCD_D4_PORT GPIOA
#define LCD_D4_PIN 14 
#define LCD_D5_PORT GPIOA
#define LCD_D5_PIN 13
#define LCD_D6_PORT GPIOA
#define LCD_D6_PIN 12
#define LCD_D7_PORT GPIOA
#define LCD_D7_PIN 11
 
 #define LCD_RS_0 (LCD_RS_PORT->ODR &= ~(1 << LCD_RS_PIN))
 #define LCD_RS_1 (LCD_RS_PORT->ODR |= (1 << LCD_RS_PIN))
 #define LCD_E_0 (LCD_E_PORT->ODR &= ~(1 << LCD_E_PIN))
 #define LCD_E_1 (LCD_E_PORT->ODR |= (1 << LCD_E_PIN))
 #define LCD_D4_0 (LCD_D4_PORT->ODR &= ~(1 << LCD_D4_PIN))
 #define LCD_D4_1 (LCD_D4_PORT->ODR |= (1 << LCD_D4_PIN))
 #define LCD_D5_0 (LCD_D5_PORT->ODR &= ~(1 << LCD_D5_PIN))
 #define LCD_D5_1 (LCD_D5_PORT->ODR |= (1 << LCD_D5_PIN))
 #define LCD_D6_0 (LCD_D6_PORT->ODR &= ~(1 << LCD_D6_PIN))
 #define LCD_D6_1 (LCD_D6_PORT->ODR |= (1 << LCD_D6_PIN))
 #define LCD_D7_0 (LCD_D7_PORT->ODR &= ~(1 << LCD_D7_PIN))
 #define LCD_D7_1 (LCD_D7_PORT->ODR |= (1 << LCD_D7_PIN))
 #define CHARS_PER_LINE 16
 #define MAXBUFFER 80
 
 void sleep(unsigned int ms);
 void usleep(unsigned char us);

 void lcd_init(void);
 void lcd_print(char *s, unsigned char line, unsigned char clear);
 void lcd_write_command(unsigned char x);
 void lcd_write_data(unsigned char x);
 void lcd_byte(unsigned char x);
 void lcd_pulse(void);
 
 
 #endif /* UTIL_H */
