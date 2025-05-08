/*
 * Hockey Table: Goal Detection and Lights
 * main.c
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 
 #include "include/stm32l051xx.h"
 #include "include/serial.h"
 #include "util.h"
 
 #define SYSCLK 32000000L
 #define TICK_FREQ 1000L
 

  // STM32L051 Pinout
  /*
  *                    ----------
  *              VDD -|1       32|- VSS
  *             PC14 -|2       31|- BOOT0
  *             PC15 -|3       30|- PB7    HOME_IR
  *             NRST -|4       29|- PB6    AWAY_IR
  *             VDDA -|5       28|- PB5    AWAY_LED
  *     TIM22    PA0 -|6       27|- PB4    HOME_LED
  *              PA1 -|7       26|- PB3    LCD_E
  *   Speaker    PA2 -|8       25|- PA15   LCD_RS
  *     25Q32    PA3 -|9       24|- PA14   LCD_D4
  *  TIME_LED    PA4 -|10      23|- PA13   LCD_D5
  *     25Q32    PA5 -|11      22|- PA12   LCD_D6
  *     25Q32    PA6 -|12      21|- PA11   LCD_D7
  *     25Q32    PA7 -|13      20|- PA10   RXD
  *              PB0 -|14      19|- PA9    TXD
  *              PB1 -|15      18|- PA8   
  *              VSS -|16      17|- VDD
  *                    ----------
  */

  // PA0: Used for measuring 22.05kHz on TIM22 (but can be any GPIO pin)
  // PA2: Speaker, general purpose output mode, configured to TIM21 CH1 to be used with PWM
  // PA3: SPI Chip Select, general purpose output mode
  // PA4: Period (time) LED, general purpose output mode
  // PA5: SPI1 SCK, general purpose output mode
  // PA6: SPI1 MISO, general purpose input mode
  // PA7: SPI1 MOSI, general purpose output mode
  // PA9: TXD (no declaration needed, for the BO230XS USB adapter) 
  // PA10: RXD (no declaration needed, for the BO230XS USB adapter)
  // PA11: LCD_D7, general purpose output mode
  // PA12: LCD_D6, general purpose output mode
  // PA13: LCD_D5, general purpose output mode
  // PA14: LCD_D4, general purpose output mode
  // PA15: LCD_RS, general purpose output mode

  // PB3: LCD_E, general purpose output mode
  // PB4: AWAY_LED, general purpose output mode
  // PB5: HOME_LED, general purpose output mode
  // PB6: AWAY_IR, general purpose input mode
  // PB7: HOME_IR, general purpose input mode

  // TIM2: Game clock counter
  // TIM21: Generates PWM signal for speaker 
  // TIM22: For playing sound with SPI


 void gpio_init(void);
 void timer2_init(void);

 volatile uint8_t minutes = 5;
 volatile uint8_t seconds = 0;
 volatile uint8_t period_over = 0;
 
 int main(void) 
 {

     char lcd_buff[MAXBUFFER];
     int home_score = 0, away_score = 0;

     gpio_init();
     lcd_init();
     timer2_init();

     sprintf(lcd_buff, "HOME  TIME  AWAY");
     lcd_print(lcd_buff, 1, 1);

     sprintf(lcd_buff, "  %d   %d:%2.2d   %d ", home_score, minutes, seconds, away_score);
     lcd_print(lcd_buff, 2, 1);

     sleep(100);

     while(1) 
     {

		 // Update the display every second of the time remaining and score
		 sprintf(lcd_buff, "  %d   %d:%2.2d   %d ", home_score, minutes, seconds, away_score);
		 lcd_print(lcd_buff, 2, 1);

		// Check if the home goal sensor is triggered 
        if(GPIOB->IDR & BIT7) {
            GPIOB->ODR &= ~BIT4; // Turn off Home Goal LED
        } else {
			TIM2->CR1 &= ~TIM_CR1_CEN;    // Pause timer
            home_score++;
            GPIOB->ODR |= BIT4; // Turn on Home Goal LED
            sprintf(lcd_buff, "  %d   %d:%2.2d   %d ", home_score, minutes, seconds, away_score);
            lcd_print(lcd_buff, 2, 1);
            sleep(5000);
			TIM2->CR1 |= TIM_CR1_CEN;    // Resume timer 
        }

		// Check if the away goal sensor is triggered
        if(GPIOB->IDR & BIT6) {
            GPIOB->ODR &= ~BIT5; // Turn off Away Goal LED
        } else {
			TIM2->CR1 &= ~TIM_CR1_CEN;    // Pause timer
            away_score++;
            GPIOB->ODR |= BIT5; // Turn on Away Goal LED
            sprintf(lcd_buff, "  %d   %d:%2.2d   %d ", home_score, minutes, seconds, away_score);
            lcd_print(lcd_buff, 2, 1);
            sleep(5000);
			TIM2->CR1 |= TIM_CR1_CEN;    // Resume timer 
        }

        sleep(100);
     }
	 
 }
 void gpio_init(void) 
 {
     /* Enable clock for port A and B */
	 RCC->IOPENR |= BIT0;
	 RCC->IOPENR |= BIT1;

	 /* Configure port A and B for very high speed (page 201). */
     GPIOA->OSPEEDR=0xFFFFFFFF;
	 GPIOB->OSPEEDR=0xFFFFFFFF;

	 /* Goal LEDs as general purpose output mode */
     GPIOB->MODER &= ~(BIT11 | BIT10 | BIT9 | BIT8); // for clearing
     GPIOB->MODER |= BIT10 | BIT8; // 01 (output mode)

	 /* Period LEDs as general purpose output mode */
	 GPIOA->MODER &= ~(BIT9 | BIT8); // for clearing 
	 GPIOA->MODER |= BIT8; // 01 (output mode)

	 /* IR Breakbeam Sensors as input, set to pull-down */	
     GPIOB->MODER &= ~(BIT15 | BIT14 | BIT13 | BIT12); // for clearing (input mode)
     GPIOB->PUPDR &= ~(BIT15 | BIT14 | BIT13 | BIT12); // for clearing (pull-down)
     GPIOB->PUPDR |= BIT15 | BIT13; // 10 (pull-down)

     /* LCD display as output, set to open-drain PA11-PA15, PB3 */
     GPIOA->MODER &= ~(BIT31 | BIT30 | BIT29 | BIT28 | BIT27 | BIT26 | BIT25 | BIT24 | BIT23 | BIT22); // for clearing
     GPIOA->MODER |= BIT30 | BIT28 | BIT26 | BIT24 | BIT22; // 01 (output mode)
     GPIOA->OTYPER &= ~(BIT11 | BIT12 | BIT13 | BIT14 | BIT15); // push-pull
     GPIOB->MODER &= ~(BIT7 | BIT6); // for clearing
     GPIOB->MODER |= BIT6; // 01 (output mode)
     GPIOB->OTYPER &= ~(BIT3); // push-pull

 }

 void timer2_init(void) 
{
     RCC->APB1ENR |= BIT0;  // Enable clock for TIM2 (p177)

     // p346: TIM2 Registers

     // Since SYSCLK is 32MHz, we can prescale the timer to 32000 to get a 1kHz (1s) clock
     TIM2->PSC = 32000 - 1;  // 32 MHz / 32000 = 1 kHz (timer divides clock every 32000 ticks)
     TIM2->ARR = 1000 - 1;   // 1 kHz / 1000 = 1 Hz (timer overflows every 1000 ticks)

	 NVIC->ISER[0] |= BIT15; // enable timer 2 interrupts in the NVIC (position 15 in the ISER register - p240)

	 TIM2->CR1 &= ~TIM_CR1_DIR; // Upcounting 
	 TIM2->CR1 |= TIM_CR1_ARPE; // ARPE enable       
     TIM2->DIER |= TIM_DIER_UIE;  // Enable DMA/interrupt register
     TIM2->CR1 |= TIM_CR1_CEN;    // Start timer 

	 __enable_irq();
}    

void TIM2_Handler(void) 
{

	 TIM2->SR &= ~TIM_SR_UIF; // Then clear the interrupt flag immediately

	 if (!period_over) { 
		 if (seconds == 0) {
			 if (minutes == 0) {
			 	 period_over = 1;
				 GPIOA->ODR |= BIT4; // Turn on Period LED
			 } 
			
			 else {
				 minutes--;
				 seconds = 59;
			 }
		 } 
		
		 else {
			 seconds--;
		 }
	 }
}
