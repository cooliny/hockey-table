/*
 * Coin Picking Robot (Remote)
 * util.c
 */

 #include "include/stm32l051xx.h"
 #include "util.h"
 
 void sleep(unsigned int ms) {
	 unsigned int i;
	 for (i = 0; i < 4*ms; ++i)
		 usleep(250);
	 return;
 }
 
 void usleep(unsigned char us) {
	 SysTick->LOAD = (SYSCLK / 1000000L * us) - 1;
	 SysTick->VAL = 0;
	 SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
	 while ((SysTick->CTRL & BIT16) == 0);
	 SysTick->CTRL = 0x00;
	 return;
 }


 void lcd_init(void) {
	LCD_E_0;
	sleep(20);

	lcd_write_command(0x33);
	lcd_write_command(0x33);
	lcd_write_command(0x32);

	lcd_write_command(0x28);
	lcd_write_command(0x0C);
	lcd_write_command(0x01);
	sleep(20);

	return;
}

void lcd_print(char *s, unsigned char line, unsigned char clear) {
	int i;

	lcd_write_command(line == 1 ? 0x80 : 0xC0);
	sleep(5);
	for (i = 0; s[i] != 0; ++i)
		lcd_write_data(s[i]);
	if (clear)
		for (; i < CHARS_PER_LINE; ++i)
			lcd_write_data(' ');
	return;
}

void lcd_write_command(unsigned char x) {
	LCD_RS_0;
	lcd_byte(x);
	sleep(5);
	return;
}

void lcd_write_data(unsigned char x) {
	LCD_RS_1;
	lcd_byte(x);
	sleep(2);
	return;
}

void lcd_byte(unsigned char x) {
	if (x & 0x80) LCD_D7_1; else LCD_D7_0;
	if (x & 0x40) LCD_D6_1; else LCD_D6_0;
	if (x & 0x20) LCD_D5_1; else LCD_D5_0;
	if (x & 0x10) LCD_D4_1; else LCD_D4_0;
	lcd_pulse();

	usleep(40);

	if (x & 0x08) LCD_D7_1; else LCD_D7_0;
	if (x & 0x04) LCD_D6_1; else LCD_D6_0;
	if (x & 0x02) LCD_D5_1; else LCD_D5_0;
	if (x & 0x01) LCD_D4_1; else LCD_D4_0;
	lcd_pulse();

	return;
}

void lcd_pulse(void) {
	LCD_E_1;
	usleep(40);
	LCD_E_0;
	return;
}