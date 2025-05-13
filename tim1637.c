
#include "include/stm32l051xx.h"
#include "tim1637.h"

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
