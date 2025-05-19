/*
 * Hockey Table: Goal Detection, Lights, and Time Display
 * main.c
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>

 #include "globals.h"
 #include "include/stm32l051xx.h"
 #include "include/serial.h"
 #include "util.h"
 #include "tim1637.h"
 #include "ws2812b.h"
 
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
 *              PA0 -|6       27|- PB4    HOME_LED
 *  AWAY_RDY    PA1 -|7       26|- PB3    LCD_E
 *  HOME_RDY    PA2 -|8       25|- PA15   LCD_RS
 *              PA3 -|9       24|- PA14   LCD_D4
 *  TIME_LED    PA4 -|10      23|- PA13   LCD_D5
 *              PA5 -|11      22|- PA12   LCD_D6
 *              PA6 -|12      21|- PA11   LCD_D7
 *   WS2812B    PA7 -|13      20|- PA10   RXD
 *   1637DIO    PB0 -|14      19|- PA9    TXD
 *   1637CLK    PB1 -|15      18|- PA8    
 *              VSS -|16      17|- VDD
 *                    ----------
 */

// CHANGE PIN DEFINITIONS AS NECESSARY

// PUSHBUTTONS
#define HOME_READY_PORT GPIOA
#define HOME_READY_PIN 2
#define AWAY_READY_PORT GPIOA
#define AWAY_READY_PIN 1 

// LEDs
#define HOME_LED_PORT GPIOB
#define HOME_LED_PIN 4 
#define AWAY_LED_PORT GPIOB
#define AWAY_LED_PIN 5 
#define TIME_LED_PORT GPIOA
#define TIME_LED_PIN 4 

// IR Sensor
#define HOME_IR_PORT GPIOB
#define HOME_IR_PIN 7 
#define AWAY_IR_PORT GPIOB
#define AWAY_IR_PIN 6 

// LCD Display
#define LCD_E_PORT GPIOB
#define LCD_E_PIN 3 
#define LCD_RS_PORT GPIOA
#define LCD_RS_PIN 15
#define LCD_D4_PORT GPIOA
#define LCD_D4_PIN 14 
#define LCD_D5_PORT GPIOA
#define LCD_D5_PIN 13
#define LCD_D6_PORT GPIOA
#define LCD_D6_PIN 12
#define LCD_D7_PORT GPIOA
#define LCD_D7_PIN 11

 // Function prototypes
 void gpio_init(void);
 void timer21_init(void);
 void home_goal(void); 
 void away_goal(void); 

 // Global Variables 

 // Time tracking
 volatile uint8_t minutes = 3;
 volatile uint8_t seconds = 0;

 // Period tracking
 volatile uint8_t period = 1; 
 volatile uint8_t period_over = 0;

 // Score tracking
 volatile uint8_t home_score = 0;
 volatile uint8_t away_score = 0;

 // Ready flags
 volatile int home_ready_flag = 0;
 volatile int away_ready_flag = 0;
 
 int main(void) 
 {
     
     char lcd_buff[MAXBUFFER];

     gpio_init();
     lcd_init();
     timer21_init();
     tm1637Init();
     spi_init();

     sprintf(lcd_buff, "HOME PERIOD AWAY");
     lcd_print(lcd_buff, 1, 1);

     sprintf(lcd_buff, "  %d     %d    %d ", home_score, period, away_score);
     lcd_print(lcd_buff, 2, 1);

     // Set all 35 LEDs to white for lighting (if isolated 5V power supply use 0xFFFFFF)
     set_colour(0x1D, 0x1D, 0x1D);

     while (1) 
     {

         // Waiting for user ready: rolls a display message and handles end-game conditions
         while (!(home_ready_flag && away_ready_flag))
         {

             // If periods 1-3
             if(period <= 3) {
                 checkReady();
                 tm1637ScrollMessage(" rEAdY UP  ", 500);

             }

             // Display the final score once game ends
             if(period > 3) {

                if(home_score != away_score) {

                     TIM21->CR1 &= ~TIM_CR1_CEN; 

                     if(home_score > away_score) {
                         sprintf(lcd_buff, "HOME <WINS  AWAY");
                         lcd_print(lcd_buff, 1, 1);
                         sprintf(lcd_buff, "  %d          %d ", home_score, away_score);
                         lcd_print(lcd_buff, 2, 1);
                     }

                     else {
                         sprintf(lcd_buff, "HOME  WINS> AWAY");
                         lcd_print(lcd_buff, 1, 1);
                         sprintf(lcd_buff, "  %d          %d ", home_score, away_score);
                         lcd_print(lcd_buff, 2, 1);
                     }

                     tm1637ScrollMessage(" FInAL SCOrE ", 500);
                        
                }

                // If tied move to overtime
                else if(home_score == away_score) {
                     sprintf(lcd_buff, "  %d    OT    %d ", home_score, away_score);
                     lcd_print(lcd_buff, 2, 1);

                     checkReady();
                     tm1637ScrollMessage(" rEAdY UP  ", 500);
                }

             }
         }

         sleep(3000); // Wait for 3 seconds once ready before starting the period puck drop

         // While the game is in progress check for goal scores + score updates
         while(home_ready_flag && away_ready_flag) 
         {
             period_over = 0;
             TIME_LED_PORT->ODR &= ~(1 << TIME_LED_PIN); // Turn off Period LED

             // Start the period timer
             TIM21->CR1 |= TIM_CR1_CEN; 

             // For periods 1-3
             if(period <= 3) {

                 // Update the display every second of the time remaining and score
                 sprintf(lcd_buff, "  %d     %d    %d ", home_score, period, away_score);
                 lcd_print(lcd_buff, 2, 1);

                 tm1637DisplayDecimal(minutes*100+seconds, 1); // Display in MMSS format

                 // Check if the home goal sensor is triggered 
                 if(HOME_IR_PORT->IDR & (1 << HOME_IR_PIN)) {
                     HOME_LED_PORT->ODR &= ~(1 << HOME_LED_PIN); // Turn off Home Goal LED
                 } else {
                    home_goal();
                    sprintf(lcd_buff, "  %d     %d    %d ", home_score, period, away_score);
                    lcd_print(lcd_buff, 2, 1);
                    sleep(5000); // Give 5 seconds for user to take puck out of goal
                    TIM21->CR1 |= TIM_CR1_CEN;    // Resume timer 
                 }

                 // Check if the away goal sensor is triggered
                 if(AWAY_IR_PORT->IDR & (1 << AWAY_IR_PIN)) {
                     AWAY_LED_PORT->ODR &= ~(1 << AWAY_LED_PIN); // Turn off Away Goal LED
                 } else {
                    away_goal();
                    sprintf(lcd_buff, "  %d     %d    %d ", home_score, period, away_score);
                    lcd_print(lcd_buff, 2, 1);
                    sleep(5000); // Give 5 seconds for user to take puck out of goal
                    TIM21->CR1 |= TIM_CR1_CEN;    // Resume timer 
                 }

             }

             // If in overtime -> sudden death, game ends once goal scored
             if(period > 3) {

                 tm1637DisplayDecimal(minutes*100+seconds, 1); // Display in MMSS format
                 HOME_LED_PORT->ODR &= ~(1 << HOME_LED_PIN); // Turn off Home Goal LED
                 AWAY_LED_PORT->ODR &= ~(1 << AWAY_LED_PIN); // Turn off Home Goal LED

                 // Check if the home goal sensor is triggered 
                 if(HOME_IR_PORT->IDR & (1 << HOME_IR_PIN)) {
                     HOME_LED_PORT->ODR &= ~(1 << HOME_LED_PIN); // Turn off Home Goal LED
                 } else {
                     home_goal();
                     sprintf(lcd_buff, "  %d    OT    %d ", home_score, away_score);
                     lcd_print(lcd_buff, 2, 1);
                     home_ready_flag = 0;
                     away_ready_flag = 0;
                 }

                 // Check if the away goal sensor is triggered
                 if(AWAY_IR_PORT->IDR & (1 << AWAY_IR_PIN)) {
                     AWAY_LED_PORT->ODR &= ~(1 << AWAY_LED_PIN); // Turn off Away Goal LED
                 } else {
                     away_goal();
                     sprintf(lcd_buff, "  %d    OT    %d ", home_score, away_score);
                     lcd_print(lcd_buff, 2, 1);
                     home_ready_flag = 0;
                     away_ready_flag = 0;
                 }
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
     HOME_LED_PORT->MODER = ((HOME_LED_PORT->MODER & ~(0x3 << HOME_LED_PIN * 2)) | (0x1 << HOME_LED_PIN * 2));
     AWAY_LED_PORT->MODER = ((AWAY_LED_PORT->MODER & ~(0x3 << AWAY_LED_PIN * 2)) | (0x1 << AWAY_LED_PIN * 2));
     //  GPIOB->MODER &= ~(BIT11 | BIT10 | BIT9 | BIT8); // for clearing
     //  GPIOB->MODER |= BIT10 | BIT8; // 01 (output mode)

     /* Period LEDs as general purpose output mode */
     TIME_LED_PORT->MODER = ((TIME_LED_PORT->MODER & ~(0x3 << TIME_LED_PIN * 2)) | (0x1 << TIME_LED_PIN * 2));
     //  GPIOA->MODER &= ~(BIT9 | BIT8); // for clearing 
     //  GPIOA->MODER |= BIT8; // 01 (output mode)

     /* IR Breakbeam Sensors as input, set to pull-down */	
     HOME_IR_PORT->MODER = (HOME_IR_PORT->MODER & ~(0x3 << HOME_IR_PIN * 2));
     AWAY_IR_PORT->MODER = (AWAY_IR_PORT->MODER & ~(0x3 << AWAY_IR_PIN * 2));
     HOME_IR_PORT->PUPDR = ((HOME_IR_PORT->PUPDR & ~(0x3 << HOME_IR_PIN * 2)) | (0x2 << HOME_IR_PIN * 2));
     AWAY_IR_PORT->PUPDR = ((AWAY_IR_PORT->PUPDR & ~(0x3 << AWAY_IR_PIN * 2)) | (0x2 << AWAY_IR_PIN * 2));
    //  GPIOB->MODER &= ~(BIT15 | BIT14 | BIT13 | BIT12); // for clearing (input mode)
    //  GPIOB->PUPDR &= ~(BIT15 | BIT14 | BIT13 | BIT12); // for clearing (pull-down)
    //  GPIOB->PUPDR |= BIT15 | BIT13; // 10 (pull-down)

     /* Home/Away Ready Pushbuttons as input, set to pull-down */
     HOME_READY_PORT->MODER = (HOME_READY_PORT->MODER & ~(0x3 << HOME_READY_PIN * 2));
     AWAY_READY_PORT->MODER = (AWAY_READY_PORT->MODER & ~(0x3 << AWAY_READY_PIN * 2));
     HOME_READY_PORT->PUPDR = ((HOME_READY_PORT->PUPDR & ~(0x3 << HOME_READY_PIN * 2)) | (0x2 << HOME_READY_PIN * 2));
     AWAY_READY_PORT->PUPDR = ((AWAY_READY_PORT->PUPDR & ~(0x3 << AWAY_READY_PIN * 2)) | (0x2 << AWAY_READY_PIN * 2));
    //  GPIOA->MODER &= ~(BIT5 | BIT4 | BIT3 | BIT2); // input mode
    //  GPIOA->PUPDR &= ~(BIT5 | BIT4 | BIT3 | BIT2); // for clearing 
    //  GPIOA->PUPDR |= BIT5 | BIT3; // 10 (pull-down)

     /* LCD display as output, set to push-pull */
     LCD_E_PORT->MODER = ((LCD_E_PORT->MODER & ~(0x3 << LCD_E_PIN * 2)) | (0x1 << LCD_E_PIN * 2));
     LCD_RS_PORT->MODER = ((LCD_RS_PORT->MODER & ~(0x3 << LCD_RS_PIN * 2)) | (0x1 << LCD_RS_PIN * 2));
     LCD_D4_PORT->MODER = ((LCD_D4_PORT->MODER & ~(0x3 << LCD_D4_PIN * 2)) | (0x1 << LCD_D4_PIN * 2));
     LCD_D5_PORT->MODER = ((LCD_D5_PORT->MODER & ~(0x3 << LCD_D5_PIN * 2)) | (0x1 << LCD_D5_PIN * 2));
     LCD_D6_PORT->MODER = ((LCD_D6_PORT->MODER & ~(0x3 << LCD_D6_PIN * 2)) | (0x1 << LCD_D6_PIN * 2));
     LCD_D7_PORT->MODER = ((LCD_D7_PORT->MODER & ~(0x3 << LCD_D7_PIN * 2)) | (0x1 << LCD_D7_PIN * 2));
     
     LCD_E_PORT->OTYPER = (LCD_E_PORT->OTYPER & ~(0x1 << LCD_E_PIN));
     LCD_RS_PORT->OTYPER = (LCD_RS_PORT->OTYPER & ~(0x1 << LCD_RS_PIN));
     LCD_D4_PORT->OTYPER = (LCD_D4_PORT->OTYPER & ~(0x1 << LCD_D4_PIN));
     LCD_D5_PORT->OTYPER = (LCD_D5_PORT->OTYPER & ~(0x1 << LCD_D5_PIN));
     LCD_D6_PORT->OTYPER = (LCD_D6_PORT->OTYPER & ~(0x1 << LCD_D6_PIN));
     LCD_D7_PORT->OTYPER = (LCD_D7_PORT->OTYPER & ~(0x1 << LCD_D7_PIN));
    //  GPIOA->MODER &= ~(BIT31 | BIT30 | BIT29 | BIT28 | BIT27 | BIT26 | BIT25 | BIT24 | BIT23 | BIT22); // for clearing
    //  GPIOA->MODER |= BIT30 | BIT28 | BIT26 | BIT24 | BIT22; // 01 (output mode)
    //  GPIOA->OTYPER &= ~(BIT11 | BIT12 | BIT13 | BIT14 | BIT15); // push-pull
    //  GPIOB->MODER &= ~(BIT7 | BIT6); // for clearing
    //  GPIOB->MODER |= BIT6; // 01 (output mode)
    //  GPIOB->OTYPER &= ~(BIT3); // push-pull

 }

 void timer21_init(void) 
{
     RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;  // Enable clock for TIM21 (p176)

     // Since SYSCLK is 32MHz, we can prescale the timer to 32000 to get a 1kHz (1s) clock
     TIM21->PSC = 32000 - 1;  // 32 MHz / 32000 = 1 kHz (timer divides clock every 32000 ticks)
     TIM21->ARR = 1000 - 1;   // 1 kHz / 1000 = 1 Hz (timer overflows every 1000 ticks)

     NVIC->ISER[0] |= BIT20; // enable timer 2 interrupts in the NVIC (position 15 in the ISER register - p240)

     TIM21->CR1 &= ~TIM_CR1_DIR; // Upcounting 
     TIM21->CR1 |= TIM_CR1_ARPE; // ARPE enable       
     TIM21->DIER |= TIM_DIER_UIE;  // Enable DMA/interrupt register

     __enable_irq(); // Enable interrupts
}    

// Timer 2 interrupt handler: 
// This only handles the time display and increments the period once over
void TIM21_Handler(void) 
{
     char lcd_buff[MAXBUFFER];

     TIM21->SR &= ~TIM_SR_UIF; // Then clear the interrupt flag immediately

     if (!period_over) { 
         if (seconds == 0) {
             if (minutes == 0) {

                 // Period is over 
                 TIM21->CR1 &= ~TIM_CR1_CEN; // Stop timer
                 period_over = 1;
                 period++;
                 TIME_LED_PORT->ODR |= (1 << TIME_LED_PIN); // Turn on Period LED
                 home_ready_flag = 0;
                 away_ready_flag = 0;
                 minutes = 3; // Reset and wait for user ready
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

// The home goal sequence pauses the game clock, increments the score, turns on the goal LED, and flashes the light
void home_goal(void) 
{
    // char lcd_buff[MAXBUFFER];

    TIM21->CR1 &= ~TIM_CR1_CEN;    // Pause timer
    home_score++;
    HOME_LED_PORT->ODR |= (1 << HOME_LED_PIN); // Turn on Home Goal LED
    flash_animation(NUM_LEDS, 0xFF, 0x00, 0x00, 50, 100000);
    set_colour(0x1D, 0x1D, 0x1D);
}

// The away goal sequence pauses the game clock, increments the score, turns on the goal LED, and flashes the light
void away_goal(void) 
{
    TIM21->CR1 &= ~TIM_CR1_CEN;    // Pause timer
    away_score++;
    AWAY_LED_PORT->ODR |= (1 << AWAY_LED_PIN); // Turn on Away Goal LED
    flash_animation(NUM_LEDS, 0x00, 0x00, 0xFF, 50, 100000);
    set_colour(0x1D, 0x1D, 0x1D);
}
