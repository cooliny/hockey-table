/*
 * Coin Picking Robot (Remote)
 * util.h
 */

 #ifndef UTIL_H
 #define UTIL_H
 
 #define SYSCLK 32000000L
 
 #define LCD_RS_0 (GPIOA->ODR &= ~BIT15)
 #define LCD_RS_1 (GPIOA->ODR |= BIT15)
 #define LCD_E_0 (GPIOB->ODR &= ~BIT3)
 #define LCD_E_1 (GPIOB->ODR |= BIT3)
 #define LCD_D4_0 (GPIOA->ODR &= ~BIT14)
 #define LCD_D4_1 (GPIOA->ODR |= BIT14)
 #define LCD_D5_0 (GPIOA->ODR &= ~BIT13)
 #define LCD_D5_1 (GPIOA->ODR |= BIT13)
 #define LCD_D6_0 (GPIOA->ODR &= ~BIT12)
 #define LCD_D6_1 (GPIOA->ODR |= BIT12)
 #define LCD_D7_0 (GPIOA->ODR &= ~BIT11)
 #define LCD_D7_1 (GPIOA->ODR |= BIT11)
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