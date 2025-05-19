#include "include/stm32l051xx.h"
#include "string.h"
#include "util.h"
#include "tim1637.h"
#include "globals.h"

// Initializes the pins for the TIM1637 Module (changes GPIO pins if necessary)
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

// Start/Stop Conditions for the TIM1637:
// When CLK is high and DIO changes from high to low, data input starts
// When CLK is high and DIO changes from low to high, data input ends

/*
*  Start condition (signals that new command coming): 
*  1. DIO/CLK high
*  2. DIO goes low while CLK remains high
*  3. CLK low to begin the transmission
*/
void _tm1637Start(void)
{
    _tm1637ClkHigh();
    _tm1637DioHigh();
    _tm1637DelayUsec(2);
    _tm1637DioLow();
}

/*
*  Stop condition (transmission complete): 
*  1. CLK/DIO low
*  2. CLK goes high while DIO remains low
*  3. DIO high to stop the transmission
*/
void _tm1637Stop(void)
{
    _tm1637ClkLow();
    _tm1637DelayUsec(2);
    _tm1637DioLow();
    _tm1637DelayUsec(2);
    _tm1637ClkHigh();
    _tm1637DelayUsec(2);
    _tm1637DioHigh();
}

void _tm1637ReadResult(void)
{
    _tm1637ClkLow();
    _tm1637DelayUsec(5);
    // while (dio); // We're cheating here and not actually reading back the response.
    _tm1637ClkHigh();
    _tm1637DelayUsec(2);
    _tm1637ClkLow();
}

// Data command setting: 0x40 for auto address increment, 0x44 for fixed address
// Address command setting: 0xC0-0xC5 for display address
// Brightness command setting: 0x87-0x8F for brightness 0-7
void _tm1637WriteByte(unsigned char b)
{
    for (int i = 0; i < 8; ++i) {
        _tm1637ClkLow();
        if (b & 0x01) {
            _tm1637DioHigh();
        }
        else {
            _tm1637DioLow();
        }
        _tm1637DelayUsec(3);
        b >>= 1;
        _tm1637ClkHigh();
        _tm1637DelayUsec(3);
    }
}

void _tm1637DelayUsec(unsigned int i)
{
     for (; i>0; i--) {
        for (int j = 0; j < 10; ++j) {
            __asm__ __volatile__("nop\n\t":::"memory");
        }
     }
}

void _tm1637ClkHigh(void)
{
    GPIOB->ODR |= BIT1; // Set CLK high
}

void _tm1637ClkLow(void)
{
    GPIOB->ODR &= ~BIT1; // Set CLK low
}

void _tm1637DioHigh(void)
{
    GPIOB->ODR |= BIT0; // Set DIO high
}

void _tm1637DioLow(void)
{
    GPIOB->ODR &= ~BIT0; // Set DIO low
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
         0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, // 0-9
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


// Displays a scrolling message across the seven-segments
void tm1637ScrollMessage(const char* message, int delay_ms)
{
    int len = strlen(message);
    char window[4];

    for (int i = 0; i < len + 4; i++) {
        checkReady();
        char segments[4];

        for (int j = 0; j < 4; j++) {
            checkReady();
            int msg_index = i - 3 + j;
            if (msg_index >= 0 && msg_index < len) {
                segments[j] = chartosegment(message[msg_index]);
            } else {
                segments[j] = 0x00; // Blank
            }
        }

        checkReady();

        // Start data transmission
        _tm1637Start();
        _tm1637WriteByte(0x40); // Set to auto-increment mode
        _tm1637ReadResult();
        _tm1637Stop();

        checkReady();

        _tm1637Start();
        _tm1637WriteByte(0xC0); // Set starting address
        _tm1637ReadResult();

        for (int k = 0; k < 4; k++) {
            checkReady();
            _tm1637WriteByte(segments[k]);
            _tm1637ReadResult();
        }

        checkReady();

        _tm1637Stop();
        tm1637SetBrightness(8);

        checkReady();

        // Delay between scroll steps
        sleep(delay_ms);
    }
}

char chartosegment(char c) 
{
     switch(c) 
     {
         case 'A': return 0x77; 
         case 'b': return 0x7C;
         case 'C': return 0x39;
         case 'd': return 0x5E;
         case 'E': return 0x79;
         case 'F': return 0x71;
         case 'g': return 0x6F;
         case 'H': return 0x76;
         case 'I': return 0x06;
         case 'J': return 0x0E;
         case 'L': return 0x38;
         case 'n': return 0x54;
         case 'O': return 0x3F;
         case 'P': return 0x73;
         case 'r': return 0x50;
         case 'S': return 0x6D;
         case 'U': return 0x3E;
         case 'X': return 0x76;
         case 'Y': return 0x6E;
         case 'Z': return 0x5B;
         case ' ': return 0x00;
         default: return 0x00; // Default case for unsupported characters (K, M, Q, T, V, W)
     }
}

void checkReady(void) 
{
     if (GPIOA->IDR & BIT1) { // Home Ready button pressed
         home_ready_flag = 1;
         GPIOB->ODR |= BIT5; // Turn on Home Goal LED
     } 
     if (GPIOA->IDR & BIT2) { // Away Ready button pressed
         away_ready_flag = 1;
         GPIOB->ODR |= BIT4; // Turn on Away Goal LED
    }
}
