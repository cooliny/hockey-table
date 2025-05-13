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
 #include "tim1637.h"
 
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
 *  AWAY_RDY    PA0 -|6       27|- PB4    HOME_LED
 *  HOME_RDY    PA1 -|7       26|- PB3    LCD_E
 *   Speaker    PA2 -|8       25|- PA15   LCD_RS
 *     25Q32    PA3 -|9       24|- PA14   LCD_D4
 *  TIME_LED    PA4 -|10      23|- PA13   LCD_D5
 *     25Q32    PA5 -|11      22|- PA12   LCD_D6
 *     25Q32    PA6 -|12      21|- PA11   LCD_D7
 *     25Q32    PA7 -|13      20|- PA10   RXD
 *   1637DIO    PB0 -|14      19|- PA9    TXD
 *   1637CLK    PB1 -|15      18|- PA8    TIM22 Output (for SPI)
 *              VSS -|16      17|- VDD
 *                    ----------
 */

 // PA2: Speaker, general purpose output mode, configured to TIM21 CH1 to be used with PWM 
 // PA3: SPI Chip Select, general purpose output mode
 // PA4: Period (time) LED, general purpose output mode
 // PA5: SPI1 SCK, general purpose output mode
 // PA6: SPI1 MISO, general purpose input mode
 // PA7: SPI1 MOSI, general purpose output mode
 // PA8: Used for measuring 22.05kHz on TIM22 (but can be any GPIO pin)
 // PA9: TXD (no declaration needed, for the BO230XS USB adapter) 
 // PA10: RXD (no declaration needed, for the BO230XS USB adapter)
 // PA11: LCD_D7, general purpose output mode
 // PA12: LCD_D6, general purpose output mode
 // PA13: LCD_D5, general purpose output mode
 // PA14: LCD_D4, general purpose output mode
 // PA15: LCD_RS, general purpose output mode

 // PB0: TM1637 DIO, general purpose output mode
 // PB1: TM1637 CLK, general purpose output mode
 // PB3: LCD_E, general purpose output mode
 // PB4: AWAY_LED, general purpose output mode
 // PB5: HOME_LED, general purpose output mode
 // PB6: AWAY_IR, general purpose input mode
 // PB7: HOME_IR, general purpose input mode

 // TIM2: Game clock counter
 // TIM21: Generates PWM signal for speaker 
 // TIM22: For playing sound with SPI

 // Function prototypes
 void gpio_init(void);
 void timer2_init(void);
 void tm1637Init(void);
 void tm1637DisplayDecimal(int v, int displaySeparator);
 void tm1637SetBrightness(char brightness);

 volatile uint8_t minutes = 5;
 volatile uint8_t seconds = 0;
 volatile uint8_t period_over = 0;
 volatile int home_ready_flag = 0;
 volatile int away_ready_flag = 0;
 
 int main(void) 
 {

     char lcd_buff[MAXBUFFER];
     int home_score = 0, away_score = 0;

     gpio_init();
     lcd_init();
     timer2_init();
     tm1637Init();

     sprintf(lcd_buff, "HOME  TIME  AWAY");
     lcd_print(lcd_buff, 1, 1);

     sprintf(lcd_buff, "  %d   %d:%2.2d   %d ", home_score, minutes, seconds, away_score);
     lcd_print(lcd_buff, 2, 1);

     tm1637DisplayDecimal(minutes*100+seconds, 1); // Display in MMSS format
     
     while (1) 
     {

         while (!(home_ready_flag && away_ready_flag))
         {
             if (GPIOA->IDR & BIT0) { // Home Ready button pressed
                 home_ready_flag = 1;
                 GPIOB->ODR |= BIT5; // Turn on Home Goal LED
             } 
             if (GPIOA->IDR & BIT1) { // Away Ready button pressed
                 away_ready_flag = 1;
                 GPIOB->ODR |= BIT4; // Turn on Away Goal LED
             }
         }

         while(home_ready_flag && away_ready_flag) 
         {
            
             GPIOA->ODR &= ~BIT4; // Turn off Period LED

             // Start the period timer
             TIM2->CR1 |= TIM_CR1_CEN; 

             // Update the display every second of the time remaining and score
             sprintf(lcd_buff, "  %d   %d:%2.2d   %d ", home_score, minutes, seconds, away_score);
             lcd_print(lcd_buff, 2, 1);

             tm1637DisplayDecimal(minutes*100+seconds, 1); // Display in MMSS format

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
        }

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

     /* Home/Away Ready Pushbuttons as input, set to pull-down */
     GPIOA->MODER &= ~(BIT3 | BIT2 | BIT1 | BIT0); // input mode
     GPIOA->PUPDR &= ~(BIT3 | BIT2 | BIT1 | BIT0); // for clearing 
     GPIOA->PUPDR |= BIT3 | BIT1; // 10 (pull-down)

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

     __enable_irq();
}    

void TIM2_Handler(void) 
{

     TIM2->SR &= ~TIM_SR_UIF; // Then clear the interrupt flag immediately

     if (!period_over) { 
         if (seconds == 0) {
             if (minutes == 0) {
                 TIM2->CR1 &= ~TIM_CR1_CEN; // Stop timer
                 period_over = 1;
                 GPIOA->ODR |= BIT4; // Turn on Period LED
                 home_ready_flag = 0;
                 away_ready_flag = 0;
                 minutes = 5; // Reset and wait for user ready
                 seconds = 0;
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

void tm1637Init(void)
{

    RCC->IOPENR |= BIT1; // Enable GPIOB clock
    GPIOB->OSPEEDR=0xFFFFFFFF; // Set GPIOB speed to very high

    // DIO Pin
    GPIOB->MODER &= ~(BIT1 | BIT0); // for clearing
    GPIOB->MODER |= BIT0; // 01 (output mode)
    GPIOB->OTYPER |= BIT0; // Open drain
    GPIOB->PUPDR &= ~(BIT1 | BIT0); // for clearing
    GPIOB->PUPDR |= BIT0; // 01 (pull-up)

    // CLK Pin
    GPIOB->MODER &= ~(BIT3 | BIT2); // for clearing
    GPIOB->MODER |= BIT2; // 01 (output mode)
    GPIOB->OTYPER |= BIT1; // Open drain
    GPIOB->PUPDR &= ~(BIT3 | BIT2); // for clearing
    GPIOB->PUPDR |= BIT2; // 01 (pull-up)

    tm1637SetBrightness(8);
}

// v: number to display
// displaySeparator: 0 = no separator, 1 = display :
void tm1637DisplayDecimal(int v, int displaySeparator)
{
     /* Segment Map: follows hgfe_dcba, converted to hexadecimal

         a
        -----
     f |  g  |b
        -----
     e |     |c   
        -----   . h
          d

     */
     const char segmentMap[] = {
         0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, // 0-7
         0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, // 8-9, A-F
         0x00
     };

     unsigned char digitArr[4];

     // Puts the number into the digit array in reverse order
     // E.g. if v= 1234, then digitArr[0] = 4, digitArr[1] = 3, digitArr[2] = 2, digitArr[3] = 1
     for (int i = 0; i < 4; ++i) {
         digitArr[i] = segmentMap[v % 10];
        
         // Set the decimal point for the third digit (h) 
         if (i == 2 && displaySeparator) {
             digitArr[i] |= 1 << 7;
         }
         v /= 10; 
     }

     // The following data transfer is done according to the TIM1637 datasheet, writing SRAM in address auto mode (p4)

     // Set the data write mode
     _tm1637Start(); // Start transmission
     _tm1637WriteByte(0x40); // Set data write mode (auto address) -> Use 0x44 for fixed address
     _tm1637ReadResult(); 
     _tm1637Stop(); // End transmission

     // Set the address
     _tm1637Start();
     _tm1637WriteByte(0xC0); // set display address to the first digit (0xC0)
     _tm1637ReadResult();

     // Write the digits in reverse order (to be correctly displayed)
     for (int i = 0; i < 4; ++i) {
         _tm1637WriteByte(digitArr[3 - i]);
         _tm1637ReadResult();
     }
     
     // Set the brightness
     tm1637SetBrightness(8); // Set brightness to maximum
}

// Valid brightness values: 0 - 8.
// 0 = display off.
void tm1637SetBrightness(char brightness)
{
    // Brightness command:
    // 1000 0XXX = display off
    // 1000 1BBB = display on, brightness 0-7
    // X = don't care
    // B = brightness
    _tm1637Start();
    _tm1637WriteByte(0x87 + brightness);
    _tm1637ReadResult();
    _tm1637Stop();
}
