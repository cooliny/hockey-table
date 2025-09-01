/*
 * Hockey Table: Contains functions used by the WS2812B LEDs
 * ws2812b.c
 */

#include "include/stm32l051xx.h"
#include "util.h"
#include "ws2812b.h"

/*
 *
 * WS2812B follows a one-wire data-sending protocol according to the datasheet
 *
 * T0H: 0.4µs ± 150ns
 * T1H: 0.8µs ± 150ns
 * T0L: 0.85µs ± 150ns
 * T1L: 0.45µs ± 150ns
 * RST: > 50µs
 *
 * Logical '1' code:          Logical '0' code:          Reset code:
 *
 * –––––––                    –––––––
 *   T1H  |  T1L                T0H  |  T0L                   RST
 *         –––––––                    –––––––            ––––––––––––––
 *
 * Each LED is set with 24 bits (RGB value from 0-255)
 * But in a different order: first 8 bits for 'G', second 8 bits for 'R', and last 8 bits for 'B'
 *
 * So using SPI with CLK of 4MHz (0.25µs), with the upper 3 bits ignored, each bit is set as:
 * → Logical '1': 0x1C (00011100) (High for 3*0.25µs = 0.75µs, low for 2*0.25µs = 0.5µs)
 * → Logical '0': 0x10 (00010000) (High for 1*0.25µs = 0.25µs, low for 4*0.25µs = 1µs)
 *
*/

uint8_t led_data[NUM_LEDS * 24];

// Initializes the SPI on the STM32
void spi_init(void)
{
    // Enable GPIOA and SPI1 clocks
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;  // Enable port A clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable SPI clock

    // Set pin to AF Mode (E.g. AF0 on PA7)
    WS2812B_PORT->MODER = ((WS2812B_PORT->MODER & ~(0x3 << WS2812B_PIN * 2)) | (0x2 << WS2812B_PIN * 2));
    WS2812B_PORT->AFR[0] &= ~(0XF << WS2812B_PIN * 4); // 0000

    // SPI1 config: master, software slave, baud rate = f_PCLK / 8 = 4 MHz
    SPI1->CR1 = SPI_CR1_MSTR   // Master
                | SPI_CR1_BR_1 // BR[2:0] = 010: f_pCLK/8
                | SPI_CR1_SSM  // Software slave management: Same value as SSI if set
                | SPI_CR1_SSI  // Internal slave select: Value of bit forced to NSS, IO value ignored
                | SPI_CR1_SPE; // Enable SPI
}

// Writes the data to the Data Register
void spi_send(uint8_t data)
{
    while (!(SPI1->SR & SPI_SR_TXE))
        ; // While the TX buffer is not empty, fill the data register with the LED
    *(volatile uint8_t *)&SPI1->DR = data;
}

// Encodes a byte (8 bits) of LED data according to SPI requirements
void encode_byte(uint8_t in, uint8_t *out)
{
    for (int i = 0; i < 8; i++)
    {
        out[i] = (in & (1 << (7 - i))) ? 0x1C : 0x10; // 0x1C (logic 1) and 0x10 (logic 0)
    }
}

// Sets a single LED to the desired rgb colour
// Note the function takes in the order of RGB for ease but passes the encoded version in GRB as per the format on the datasheet
void set_pixel(int led_index, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t encoded[8];

    int offset = led_index * 24;

    encode_byte(g, encoded);
    for (int i = 0; i < 8; i++)
        led_data[offset++] = encoded[i];

    encode_byte(r, encoded);
    for (int i = 0; i < 8; i++)
        led_data[offset++] = encoded[i];

    encode_byte(b, encoded);
    for (int i = 0; i < 8; i++)
        led_data[offset++] = encoded[i];
}

// Sends the encoded LED data with SPI
void send_buffer(int len)
{
    for (int i = 0; i < len; i++)
    {
        spi_send(led_data[i]);
    }
}

// Delays 50µs for reset during the end of a transmission 
void ws2812b_reset(void)
{
    usleep(50); 
}

// Sets a colour to all NUM_LEDS LEDs
void set_colour(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        set_pixel(i, r, g, b); // RGB format
    }

    send_buffer(sizeof(led_data));
    ws2812b_reset();
}

// If used in a while loop, will cause active_leds LEDS to spin around the ring
void spin_animation(int num_leds, uint8_t r, uint8_t g, uint8_t b, int active_leds, int delay)
{
    static int pos = 0;

    for (int i = 0; i < num_leds; i++)
    {
        // Check if LED is within the "active" spinning range
        int relative_pos = (i - pos + num_leds) % num_leds;
        if (relative_pos < active_leds)
        {
            set_pixel(i, r, g, b); // Active pixel
        }
        else
        {
            set_pixel(i, 0, 0, 0); // Off
        }
    }

    send_buffer(num_leds * 24);
    ws2812b_reset();

    usleep(delay); 

    pos = (pos + 1) % num_leds;
}

// Will flash all the LEDS a specified amount of times and reset game lighting to white
void flash_animation(int num_leds, uint8_t r, uint8_t g, uint8_t b, int flashes)
{
    for (int i = 0; i < flashes; i++)
    {
        // Turn LEDs ON
        for (int j = 0; j < num_leds; j++)
        {
            set_pixel(j, r, g, b);
        }
        send_buffer(num_leds * 24);
        ws2812b_reset();
        sleep(16);

        // Turn LEDs OFF
        for (int j = 0; j < num_leds; j++)
        {
            set_pixel(j, 0, 0, 0);
        }
        send_buffer(num_leds * 24);
        ws2812b_reset();
        sleep(16);
        set_colour(0x1D, 0x1D, 0x1D);
    }
}
