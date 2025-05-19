#include "include/stm32l051xx.h"
#include "string.h"
#include "util.h"
#include "tim1637.h"
#include "globals.h"

/* 
 *
 * TIM1637 module follows a two-wire data-sending protocol according to the datasheet
 * 
 * We write SRAM data in address auto increment 1 mode: 
 * 
 * 1. Start Condition
 * 2. Command 1 (set data) with 8 posedge CLK
 * 3. ACK
 * 4. Stop Condition
 * 5. Start Condition
 * 6. Command 2 (set address) with 8 posedge CLK
 * 7. ACK
 * 8. Transfer the actual data (each with 8 posedge CLK and an ACK)
 * 9. Start Condition
 * 10. Command 3 (control display)
 * 11. ACK
 * 12. Stop Condition
 * 
 * Start Condition: 
 * CLK is high and DIO changes from high to low
 * 
 * Stop Condition: 
 * CLK is high and DIO changes from low to high 
 * 
 * ACK: 
 * DIO is low and CLK changes from low to high 
 * 
 * Command 1 (set data): 
 * → 0x40 (auto address increment)
 * → 0x44 (fixed address)
 * 
 * Command 2 (set address): 
 * → 0xC0, 0xC1, 0xC2, 0xC3 (on which digit of TIM1637 to display data)
 * 
 * Command 3 (control display): 
 * → Pick between 0x87 (no brightness) to 0x8F (may brightness)
 * 
*/


// Initializes the CLK and DIO pins for the TIM1637 Module 
void tm1637Init(void)
{

    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    GPIOA->OSPEEDR=0xFFFFFFFF;
    GPIOB->OSPEEDR=0xFFFFFFFF;

    TIM1637_DIO_PORT->MODER = ((TIM1637_DIO_PORT->MODER & ~(0x3 << TIM1637_DIO_PIN * 2)) | (0x1 << TIM1637_DIO_PIN * 2));
    TIM1637_CLK_PORT->MODER = ((TIM1637_CLK_PORT->MODER & ~(0x3 << TIM1637_CLK_PIN * 2)) | (0x1 << TIM1637_CLK_PIN * 2));

    TIM1637_DIO_PORT->OTYPER = (TIM1637_DIO_PORT->OTYPER | (0x1 << TIM1637_DIO_PIN));
    TIM1637_CLK_PORT->OTYPER = (TIM1637_CLK_PORT->OTYPER | (0x1 << TIM1637_CLK_PIN));

    TIM1637_DIO_PORT->PUPDR = ((TIM1637_DIO_PORT->PUPDR & ~(0x3 << TIM1637_DIO_PIN * 2)) | (0x1 << TIM1637_DIO_PIN * 2));
    TIM1637_CLK_PORT->PUPDR = ((TIM1637_CLK_PORT->PUPDR & ~(0x3 << TIM1637_CLK_PIN * 2)) | (0x1 << TIM1637_CLK_PIN * 2));

    tm1637SetBrightness(8);
}

// Start condition for TIM1637
void _tm1637Start(void)
{
    _tm1637ClkHigh();
    _tm1637DioHigh();
    _tm1637DelayUsec(2);
    _tm1637DioLow();
}

// Stop condition for TIM1637
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

// Writes a byte with DIO (8 bits of data), followed by a posedge of CLK
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

// Acts as the ACK after each data bit is sent
void _tm1637ReadResult(void)
{
    _tm1637ClkLow();
    _tm1637DelayUsec(5);
    // while (dio); // We're cheating here and not actually reading back the response.
    _tm1637ClkHigh();
    _tm1637DelayUsec(2);
    _tm1637ClkLow();
}

// Delay function 
void _tm1637DelayUsec(unsigned int i)
{
     for (; i>0; i--) {
        for (int j = 0; j < 10; ++j) {
            __asm__ __volatile__("nop\n\t":::"memory");
        }
     }
}

// Sets CLK pin high
void _tm1637ClkHigh(void)
{
    TIM1637_CLK_PORT->ODR |= (1 << TIM1637_CLK_PIN);
}

// Sets CLK pin low
void _tm1637ClkLow(void)
{
    TIM1637_CLK_PORT->ODR &= ~(1 << TIM1637_CLK_PIN);
}

// Sets DIO pin high
void _tm1637DioHigh(void)
{
    TIM1637_DIO_PORT->ODR |= (1 << TIM1637_DIO_PIN);
}

// Sets DIO pin low
void _tm1637DioLow(void)
{
    TIM1637_DIO_PORT->ODR &= ~(1 << TIM1637_DIO_PIN);
}

// Sets a brightness value (only from 0-8) to allow for 0x87-0x8F
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

// Displays a number (up to 4 digits), with a colon (displaySeparator = 1) if wanted
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
        
         // If the displaySeparator is high, will set the colon ("h" of hgfe_dcba - MSB)
         if (i == 2 && displaySeparator) {
             digitArr[i] |= 1 << 7;
         }
         v /= 10; 
     }

     // Writing SRAM in address auto mode (p4

     // Set the data write mode
     _tm1637Start(); // Start transmission
     _tm1637WriteByte(0x40); // Set data write mode (address auto increment mode)
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
// Since this takes a while, a ready check is conducted to ensure CPU doesn't miss user ready
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

// Returns the hexadecimal (hgfe_dcba) of allowed characters (off if not allowed)
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

// Check ready function that is needed for the scrolling function waiting for user ready
void checkReady(void) 
{
     if (HOME_READY_PORT->IDR & (1 << HOME_READY_PIN)) { // Home Ready button pressed
         home_ready_flag = 1;
         HOME_LED_PORT->ODR |= (1 << HOME_LED_PIN);
     } 
     if (AWAY_READY_PORT->IDR & (1 << AWAY_READY_PIN)) { // Away Ready button pressed
         away_ready_flag = 1;
         AWAY_LED_PORT->ODR |= (1 << AWAY_LED_PIN);
    }
}
