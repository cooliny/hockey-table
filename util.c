/*
 * Hockey Table: Contains delay functions (ms/µs) and lcd functions
 * util.c
 */

 #include "include/stm32l051xx.h"
 #include "util.h"
 
 // Delays x milliseconds
 void sleep(unsigned int ms) {
	 unsigned int i;
	 for (i = 0; i < 4*ms; ++i)
		 usleep(250);
	 return;
 }
 
 // Delays x microseconds
 void usleep(unsigned char us) {
	 SysTick->LOAD = (SYSCLK / 1000000L * us) - 1;
	 SysTick->VAL = 0;
	 SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
	 while ((SysTick->CTRL & BIT16) == 0);
	 SysTick->CTRL = 0x00;
	 return;
 }


 // Initializes the LCD display
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

// Writes a string to the LCD display
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

// Sends a command to the LCD
void lcd_write_command(unsigned char x) {
	LCD_RS_0;
	lcd_byte(x);
	sleep(5);
	return;
}

// Sends a character to the LCD
void lcd_write_data(unsigned char x) {
	LCD_RS_1;
	lcd_byte(x);
	sleep(2);
	return;
}

// Determines which byte is sent to the LCD
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

// Latches the data
void lcd_pulse(void) {
	LCD_E_1;
	usleep(40);
	LCD_E_0;
	return;
}


// ADC STUFF -> commented is not yet implemented

// In util.h: 
// void initADC(void);
// int readADC(unsigned int channel);

// In main: 
// initADC(); 
// gpio init as analog input
// readADC(ADC_CHSELR_CHSEL4); 

// All of this code is mostly copy/paste from the STM32L05X reference manual RM0451.

void initADC(void)
{
	RCC->APB2ENR |= BIT9; // peripheral clock enable for ADC (page 175 or RM0451)

	// ADC clock selection procedure (page 746 of RM0451)
	/* (1) Select PCLK by writing 11 in CKMODE */
	ADC1->CFGR2 |= ADC_CFGR2_CKMODE; /* (1) */
	
	// ADC enable sequence procedure (page 745 of RM0451)
	/* (1) Clear the ADRDY bit */
	/* (2) Enable the ADC */
	/* (3) Wait until ADC ready */
	ADC1->ISR |= ADC_ISR_ADRDY; /* (1) */
	ADC1->CR |= ADC_CR_ADEN; /* (2) */
	if ((ADC1->CFGR1 & ADC_CFGR1_AUTOFF) == 0)
	{
		while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (3) */
		{
			/* For robust implementation, add here time-out management */
		}
	}	

	// Calibration code procedure (page 745 of RM0451)
	/* (1) Ensure that ADEN = 0 */
	/* (2) Clear ADEN */
	/* (3) Set ADCAL=1 */
	/* (4) Wait until EOCAL=1 */
	/* (5) Clear EOCAL */
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
		ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	ADC1->CR |= ADC_CR_ADCAL; /* (3) */
	while ((ADC1->ISR & ADC_ISR_EOCAL) == 0) /* (4) */
	{
		/* For robust implementation, add here time-out management */
	}
	ADC1->ISR |= ADC_ISR_EOCAL; /* (5) */
}

int readADC(unsigned int channel)
{
	// Single conversion sequence code example - Software trigger (page 746 of RM0451)
	/* (1) Select HSI16 by writing 00 in CKMODE (reset value) */
	/* (2) Select the auto off mode */
	/* (3) Select channel */
	/* (4) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than17.1us */
	/* (5) Wake-up the VREFINT (only for VRefInt) */
	//ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* (1) */
	ADC1->CFGR1 |= ADC_CFGR1_AUTOFF; /* (2) */
	ADC1->CHSELR = channel; /* (3) */
	ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; /* (4) */
	if(channel==ADC_CHSELR_CHSEL17)
	{
		ADC->CCR |= ADC_CCR_VREFEN; /* (5) */
	}
	
	/* Performs the AD conversion */
	ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
	while ((ADC1->ISR & ADC_ISR_EOC) == 0) /* wait end of conversion */
	{
		/* For robust implementation, add here time-out management */
	}

	return ADC1->DR; // ADC_DR has the 12 bits out of the ADC
}
