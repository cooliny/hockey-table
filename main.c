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
 #include "dfplayer.h"
 
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
 *      BUSY    PA0 -|6       27|- PB4    SERVO
 *  AWAY_RDY    PA1 -|7       26|- PB3    LCD_E
 *  USART_TX    PA2 -|8       25|- PA15   LCD_RS
 *  USART_RX    PA3 -|9       24|- PA14   LCD_D4
 *  HOME_LED    PA4 -|10      23|- PA13   LCD_D5
 *  HOME_RDY    PA5 -|11      22|- PA12   LCD_D6
 *              PA6 -|12      21|- PA11   LCD_D7
 *   WS2812B    PA7 -|13      20|- PA10   RXD
 *   1637DIO    PB0 -|14      19|- PA9    TXD
 *   1637CLK    PB1 -|15      18|- PA8    TIME_LED
 *              VSS -|16      17|- VDD
 *                    ----------
 */

// CHANGE PIN DEFINITIONS AS NECESSARY

// Pushbuttons
#define HOME_READY_PORT GPIOA
#define HOME_READY_PIN 5
#define AWAY_READY_PORT GPIOA
#define AWAY_READY_PIN 1 

// LEDs
#define HOME_LED_PORT GPIOA
#define HOME_LED_PIN 4 
#define AWAY_LED_PORT GPIOB
#define AWAY_LED_PIN 5 
#define TIME_LED_PORT GPIOA
#define TIME_LED_PIN 8

// IR Sensor
#define HOME_IR_PORT GPIOB
#define HOME_IR_PIN 7 
#define AWAY_IR_PORT GPIOB
#define AWAY_IR_PIN 6 

// Servo for Puck Drop
#define SERVO_PORT GPIOB
#define SERVO_PIN 4

 // Function prototypes
 void gpio_init(void);
 void timer21_init(void);
 void timer22_init(void);
 void home_goal(void); 
 void away_goal(void); 
 void puck_drop(void);
 void ambient_music(void); 
 void home_goal_decoder(void);
 void away_goal_decoder(void);

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
//  volatile int song_flag; 
 
 int main(void) 
 {
     
     char lcd_buff[MAXBUFFER];

     gpio_init();
     lcd_init();
     timer21_init();
     timer22_init();
     tm1637Init();
     spi_init();

     sprintf(lcd_buff, "HOME PERIOD AWAY");
     lcd_print(lcd_buff, 1, 1);
     sprintf(lcd_buff, "  %d     %d    %d ", home_score, period, away_score);
     lcd_print(lcd_buff, 2, 1);

     // Set all 35 LEDs to white for lighting (if isolated 5V power supply use 0xFFFFFF)
     set_colour(0xFF, 0xFF, 0xFF);

     USART2_init();
     sleep(3000);
     DF_init();
     sleep(100);

     DF_play(0, 1); // Play the HNIC theme

     while (1) 
     {

         // Waiting for user ready: rolls a display message and handles end-game conditions
         while (!home_ready_flag || !away_ready_flag || !(GPIOA->IDR & BIT0))
         {

             // In between periods, will prompt user to ready up and put puck back in puck drop
             if(period <= 3) {
                 checkReady();
                 tm1637ScrollMessage(" rEAdY UP  ", 500);
             }

             if(period > 3) {

                // Display the final score once game ends
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

                     sleep(20000); // Display final score for 20 seconds
                     NVIC_SystemReset(); // Reset to the start
                        
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

         sleep(3000); // Wait for 3 seconds once both players ready before starting the period puck drop
         puck_drop(); 

         // Play ambient music
         ambient_music();

         // While the game is in progress check for goal scores + score updates
         while(home_ready_flag && away_ready_flag) 
         {

             period_over = 0;
             TIME_LED_PORT->ODR &= ~(1 << TIME_LED_PIN); // Turn off Period LED

             // Start the period timer
             TIM21->CR1 |= TIM_CR1_CEN; 

             // If puck is not returned in time use ready button to manually drop puck
             if((HOME_READY_PORT->IDR & (1 << HOME_READY_PIN)) | (AWAY_READY_PORT->IDR & (1 << AWAY_READY_PIN))) {
                puck_drop();
             }

             // For periods 1-3
             if(period <= 3) {

                 // Update the display every second of the time remaining and score
                 sprintf(lcd_buff, "  %d     %d    %d ", home_score, period, away_score);
                 lcd_print(lcd_buff, 2, 1);

                 tm1637DisplayDecimal(minutes*100+seconds, 1); // Display in MMSS format

                 // Check if the home goal sensor is triggered 
                 if(HOME_IR_PORT->IDR & (1 << HOME_IR_PIN)) {
                     HOME_LED_PORT->ODR &= ~(1 << HOME_LED_PIN); // Turn off Home Goal LED
                 } 

                 else {
                    home_goal_decoder();
                    home_goal();
                    sprintf(lcd_buff, "  %d     %d    %d ", home_score, period, away_score);
                    lcd_print(lcd_buff, 2, 1);
                    sleep(9000); // Give 18 seconds for user to take puck out of goal (finish goal song)
                    puck_drop();
                    TIM21->CR1 |= TIM_CR1_CEN;    // Resume timer 
                    ambient_music();
                 }

                 // Check if the away goal sensor is triggered
                 if(AWAY_IR_PORT->IDR & (1 << AWAY_IR_PIN)) {
                     AWAY_LED_PORT->ODR &= ~(1 << AWAY_LED_PIN); // Turn off Away Goal LED
                 } 

                 else {
                    away_goal_decoder();
                    away_goal();
                    sprintf(lcd_buff, "  %d     %d    %d ", home_score, period, away_score);
                    lcd_print(lcd_buff, 2, 1);
                    sleep(9000); // Give 15 seconds for user to take puck out of goal (finish goal song)
                    puck_drop(); 
                    TIM21->CR1 |= TIM_CR1_CEN;    // Resume timer 
                    ambient_music();
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
                 } 

                 else {
                     home_goal_decoder();
                     home_goal();
                     sprintf(lcd_buff, "  %d    OT    %d ", home_score, away_score);
                     lcd_print(lcd_buff, 2, 1);
                     home_ready_flag = 0;
                     away_ready_flag = 0;
                 }

                 // Check if the away goal sensor is triggered
                 if(AWAY_IR_PORT->IDR & (1 << AWAY_IR_PIN)) {
                     AWAY_LED_PORT->ODR &= ~(1 << AWAY_LED_PIN); // Turn off Away Goal LED
                 } 

                 else {
                     away_goal_decoder();
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
     RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
     RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
    
     /* Configure port A and B for very high speed (page 201). */
     GPIOA->OSPEEDR=0xFFFFFFFF;
     GPIOB->OSPEEDR=0xFFFFFFFF;

     /* Goal LEDs as general purpose output mode */
     HOME_LED_PORT->MODER = ((HOME_LED_PORT->MODER & ~(0x3 << HOME_LED_PIN * 2)) | (0x1 << HOME_LED_PIN * 2));
     AWAY_LED_PORT->MODER = ((AWAY_LED_PORT->MODER & ~(0x3 << AWAY_LED_PIN * 2)) | (0x1 << AWAY_LED_PIN * 2));

     /* Period LEDs as general purpose output mode */
     TIME_LED_PORT->MODER = ((TIME_LED_PORT->MODER & ~(0x3 << TIME_LED_PIN * 2)) | (0x1 << TIME_LED_PIN * 2));

     /* IR Breakbeam Sensors as input, set to pull-down */	
     HOME_IR_PORT->MODER = (HOME_IR_PORT->MODER & ~(0x3 << HOME_IR_PIN * 2));
     AWAY_IR_PORT->MODER = (AWAY_IR_PORT->MODER & ~(0x3 << AWAY_IR_PIN * 2));
     HOME_IR_PORT->PUPDR = ((HOME_IR_PORT->PUPDR & ~(0x3 << HOME_IR_PIN * 2)) | (0x2 << HOME_IR_PIN * 2));
     AWAY_IR_PORT->PUPDR = ((AWAY_IR_PORT->PUPDR & ~(0x3 << AWAY_IR_PIN * 2)) | (0x2 << AWAY_IR_PIN * 2));

     /* Home/Away Ready Pushbuttons as input, set to pull-down */
     HOME_READY_PORT->MODER = (HOME_READY_PORT->MODER & ~(0x3 << HOME_READY_PIN * 2));
     AWAY_READY_PORT->MODER = (AWAY_READY_PORT->MODER & ~(0x3 << AWAY_READY_PIN * 2));
     HOME_READY_PORT->PUPDR = ((HOME_READY_PORT->PUPDR & ~(0x3 << HOME_READY_PIN * 2)) | (0x2 << HOME_READY_PIN * 2));
     AWAY_READY_PORT->PUPDR = ((AWAY_READY_PORT->PUPDR & ~(0x3 << AWAY_READY_PIN * 2)) | (0x2 << AWAY_READY_PIN * 2));

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

     // Servo as alternate function mode, set to PWM output mode, push-pull (PB4 - TIM22 CH1)
     SERVO_PORT->MODER = ((SERVO_PORT->MODER & ~(0x3 << SERVO_PIN * 2)) | (0x2 << SERVO_PIN * 2));
     SERVO_PORT->AFR[0] = ((SERVO_PORT->AFR[0] & ~(0xF << SERVO_PIN * 4)) | (0x4 << SERVO_PIN * 4)); // AF4 selected (0100)
     SERVO_PORT->OTYPER = (SERVO_PORT->OTYPER & ~(0x1 << SERVO_PIN));

     // Initalize PA0 as input mode for DFPlayer Busy
     GPIOA->MODER &= ~(BIT1 | BIT0); 

 }

 // Keep tracks of game clock
 void timer21_init(void) 
{
     RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;  // Enable clock for TIM21 (p176)

     // Since SYSCLK is 32MHz, we can prescale the timer to 32000 to get a 1kHz (1s) clock
     TIM21->PSC = 32000 - 1;  // 32 MHz / 32000 = 1 kHz (timer divides clock every 32000 ticks)
     TIM21->ARR = 1000 - 1;   // 1 kHz / 1000 = 1 Hz (timer overflows every 1000 ticks)

     NVIC->ISER[0] |= (1 << TIM21_IRQn); // enable timer 2 interrupts in the NVIC (position 20 in the ISER register - p240)

     TIM21->CR1 &= ~TIM_CR1_DIR; // Upcounting 
     TIM21->CR1 |= TIM_CR1_ARPE; // ARPE enable       
     TIM21->DIER |= TIM_DIER_UIE;  // Enable DMA/interrupt register

     __enable_irq(); // Enable interrupts
}    

// For Servo 
void timer22_init(void)
{

    RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;  // Enable clock for TIM22 (p176)

    TIM22->PSC = 32-1; // 32 MHz / 32 = 1 MHz (timer divides clock every 32 ticks)
    TIM22->ARR = 20000-1; // 50Hz (20ms per servo datashet)

    TIM22->CCR1 = 600; 

    // Set PWM mode 1 on Channel 1 (OC1M = 110), enable preload
    TIM22->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM22->CCMR1 |= (BIT6 | BIT5);  // PWM mode 1
    TIM22->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload

    // Enable output compare on CH1
    TIM22->CCER |= TIM_CCER_CC1E;

    // Enable auto-reload preload
    TIM22->CR1 |= TIM_CR1_ARPE;

    TIM22->EGR |= TIM_EGR_UG; // Force update to load registers

    // Enable counter
    TIM22->CR1 |= TIM_CR1_CEN;

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
                 
                 if((period == 3) && (home_score != away_score)) {
                     if(home_score > away_score) 
                        home_goal_decoder();
                    else 
                        away_goal_decoder();
                 }

                 else {
                    DF_play(0, 2); // Play period end sound
                 }
                 
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
    AWAY_LED_PORT->ODR |= (1 << AWAY_LED_PIN); // Turn on Home Goal LED
    flash_animation(NUM_LEDS, 0xFF, 0x00, 0x00, 200);
    set_colour(0xFF, 0xFF, 0xFF);
}

// The away goal sequence pauses the game clock, increments the score, turns on the goal LED, and flashes the light
void away_goal(void) 
{
    TIM21->CR1 &= ~TIM_CR1_CEN;    // Pause timer
    away_score++;
    HOME_LED_PORT->ODR |= (1 << HOME_LED_PIN); // Turn on Away Goal LED
    flash_animation(NUM_LEDS, 0x00, 0x00, 0xFF, 240);
    set_colour(0xFF, 0xFF, 0xFF);
}

void puck_drop(void) 
{
    TIM22->CCR1 = 1500;
    sleep(1000);
    TIM22->CCR1 = 600;
}

void ambient_music(void) 
{
    // Period Start Sounds
    if(minutes == 3 && seconds == 0) {
        if(period == 1) 
            DF_play(0, 3);
        else if(period == 2)
            DF_play(0, 4);
        else if(period == 3)
            DF_play(0, 5);
        else if(period == 4)
            DF_play(0, 6);
    }

    // Ambient music after goal
    else {
        if(period == 1) 
            DF_play(0, 7);
        else if(period == 2)
            DF_play(0, 8);
        else if(period == 3)
            DF_play(0, 9);
    }

}

void home_goal_decoder(void)
{

    if(period <= 3) {

        if(minutes == 2) {
            if(seconds >= 45) 
                DF_play(0, 11); 
            else if(seconds <= 15)
                DF_play(0, 12);
            else 
                DF_play(0, 10); 
        }

        else if(minutes == 1) {
            if(seconds >= 45) 
                DF_play(0, 13); 
            else if(seconds <= 15)
                DF_play(0, 14);
            else 
                DF_play(0, 10); 
        }  
        
        else if(minutes == 0 && seconds <= 20) 
            DF_play(0, 15);

        else 
            DF_play(0, 10); 

    }

    else 
        DF_play(0, 16);

}

void away_goal_decoder(void) {

    if(period <= 3) {

        if(minutes == 2) {
            if(seconds >= 45) 
                DF_play(0, 18); 
            else if(seconds <= 15)
                DF_play(0, 19);
            else 
                DF_play(0, 17); 
        }

        else if(minutes == 1) {
            if(seconds >= 45) 
                DF_play(0, 20); 
            else if(seconds <= 15)
                DF_play(0, 21);
            else 
                DF_play(0, 17); 
        }  
        
        else if(minutes == 0 && seconds <= 20) 
            DF_play(0, 22);

        else 
            DF_play(0, 17); 

    }

    else 
        DF_play(0, 23);

}
