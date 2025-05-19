#include "include/stm32l051xx.h"
#include "util.h"
#include "ws2812b.h"

uint8_t led_data[NUM_LEDS * 24];

// =====================
// SPI INITIALIZATION
// =====================
void spi_init(void) {
    // Enable GPIOA and SPI1 clocks
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN; // Enable port A clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable SPI clock

    // PA7 = SPI1_MOSI (AF0)
    GPIOA->MODER &= ~(3 << (2 * 7));
    GPIOA->MODER |=  (2 << (2 * 7));  // Alternate function
    GPIOA->AFR[0] &= ~(0xF << (4 * 7)); // AF0 (BIT31-BIT28)

    // SPI1 config: master, software slave, baud rate = f_PCLK / 8 = 4 MHz
    SPI1->CR1 = SPI_CR1_MSTR           // Master
              | SPI_CR1_BR_1           // BR[2:0] = 010: f_pCLK/8 
              | SPI_CR1_SSM            // Software slave management: Same value as SSI if set
              | SPI_CR1_SSI            // Internal slave select: Value of bit forced to NSS, IO value ignored
              | SPI_CR1_SPE;           // Enable SPI
}

// =====================
// SPI SEND BYTE
// =====================
void spi_send(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE)); // While the TX buffer is not empty, fill the data register with the LED
        *(volatile uint8_t *)&SPI1->DR = data;
}

// =====================
// ENCODE ONE BYTE (8 bits → 8 encoded bytes)
// =====================
void encode_byte(uint8_t in, uint8_t *out) {
    for (int i = 0; i < 8; i++) {
        out[i] = (in & (1 << (7 - i))) ? 0x1C : 0x10; // 0x1C (logic 1) and 0x10 (logic 0)
    }
}

// =====================
// SET ONE PIXEL (GRB)
// =====================
void set_pixel(int led_index, uint8_t r, uint8_t g, uint8_t b) {
    uint8_t encoded[8];

    // Each LED = 24 bits → 24 * 5 = 120 bits = 15 bytes
    int offset = led_index * 24;

    encode_byte(g, encoded);
    for (int i = 0; i < 8; i++) led_data[offset++] = encoded[i];

    encode_byte(r, encoded);
    for (int i = 0; i < 8; i++) led_data[offset++] = encoded[i];

    encode_byte(b, encoded);
    for (int i = 0; i < 8; i++) led_data[offset++] = encoded[i];
}

// =====================
// SEND ALL LED DATA
// =====================
void send_buffer(int len) {
    for (int i = 0; i < len; i++) {
        spi_send(led_data[i]);
    }
}

// =====================
// LATCH (Reset time ≥ 50 µs)
// =====================
void ws2812_latch(void) {
    for (volatile int i = 0; i < 3000; i++); // crude ~50 µs delay
}

void set_colour(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < NUM_LEDS; i++) {
        set_pixel(i, r, g, b); // RGB format
    }

    send_buffer(sizeof(led_data));
    ws2812_latch();
}

void spin_animation(int num_leds, uint8_t r, uint8_t g, uint8_t b, int active_leds, int delay) {
    static int pos = 0;

    for (int i = 0; i < num_leds; i++) {
        // Check if LED is within the "active" spinning range
        int relative_pos = (i - pos + num_leds) % num_leds;
        if (relative_pos < active_leds) {
            set_pixel(i, r, g, b); // Active pixel
        } else {
            set_pixel(i, 0, 0, 0); // Off
        }
    }

    send_buffer(num_leds * 24);
    ws2812_latch();

    for (volatile int d = 0; d < delay; d++); // crude delay

    pos = (pos + 1) % num_leds;
}

void flash_animation(int num_leds, uint8_t r, uint8_t g, uint8_t b, int flashes, int delay) {
    for (int i = 0; i < flashes; i++) {
        // Turn LEDs ON
        for (int j = 0; j < num_leds; j++) {
            set_pixel(j, r, g, b);
        }
        send_buffer(num_leds * 24);
        ws2812_latch();
        for (volatile int d = 0; d < delay; d++);

        // Turn LEDs OFF
        for (int j = 0; j < num_leds; j++) {
            set_pixel(j, 0, 0, 0);
        }
        send_buffer(num_leds * 24);
        ws2812_latch();
        for (volatile int d = 0; d < delay; d++);
    }
}