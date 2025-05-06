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
 
 /*
  *                    ----------
  *              VDD -|1       32|- VSS
  *             PC14 -|2       31|- BOOT0
  *             PC15 -|3       30|- PB7
  *             NRST -|4       29|- PB6
  *             VDDA -|5       28|- PB5
  *              PA0 -|6       27|- PB4
  *              PA1 -|7       26|- PB3   LCD_E
  *              PA2 -|8       25|- PA15  LCD_RS
  *              PA3 -|9       24|- PA14  LCD_D4
  *              PA4 -|10      23|- PA13  LCD_D5
  *  TIME_LED    PA5 -|11      22|- PA12  LCD_D6
  *   HOME_IR    PA6 -|12      21|- PA11  LCD_D7
  *   AWAY_IR    PA7 -|13      20|- PA10  TXD
  *  HOME_LED    PB0 -|14      19|- PA9   RXD
  *  AWAY_LED    PB1 -|15      18|- PA8   
  *              VSS -|16      17|- VDD
  *                    ----------
  */

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

     GPIOB->ODR &= ~BIT0; // Turn off Goal LED
     GPIOB->ODR &= ~BIT1; // Turn off Goal LED

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
        if(GPIOA->IDR & BIT6) {
            GPIOB->ODR &= ~BIT1; // Turn off Home Goal LED
        } else {
			TIM2->CR1 &= ~TIM_CR1_CEN;    // Pause timer
            home_score++;
            GPIOB->ODR |= BIT1; // Turn on Home Goal LED
            sprintf(lcd_buff, "  %d   %d:%2.2d   %d ", home_score, minutes, seconds, away_score);
            lcd_print(lcd_buff, 2, 1);
            sleep(5000);
			TIM2->CR1 |= TIM_CR1_CEN;    // Resume timer 
        }

		// Check if the away goal sensor is triggered
        if(GPIOA->IDR & BIT7) {
            GPIOB->ODR &= ~BIT0; // Turn off Away Goal LED
        } else {
			TIM2->CR1 &= ~TIM_CR1_CEN;    // Pause timer
            away_score++;
            GPIOB->ODR |= BIT0; // Turn on Away Goal LED
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
     GPIOB->MODER &= ~(BIT3 | BIT2 | BIT1 | BIT0); // for clearing
     GPIOB->MODER |= BIT2 | BIT0; // 01 (output mode)

	 /* Period LEDs as general purpose output mode */
	 GPIOA->MODER &= ~(BIT11 | BIT10); // for clearing 
	 GPIOA->MODER |= BIT10; // 01 (output mode)

	 /* IR Breakbeam Sensors as input, set to pull-down */	
     GPIOA->MODER &= ~(BIT15 | BIT14 | BIT13 | BIT12); // for clearing (input mode)
     GPIOA->PUPDR &= ~(BIT17 | BIT16); // for clearing (pull-down)
     GPIOA->PUPDR |= BIT15 | BIT13; // 10 (pull-down)

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
				 GPIOA->ODR |= BIT5; // Turn on Period LED
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