#ifndef WS2812B_H
#define WS2812B_H

#define SYSCLK 32000000L
#define NUM_LEDS 35

#define WS2812B_PORT GPIOA
#define WS2812B_PIN 7

void spi_init(void);
void spi_send(uint8_t data);
void encode_byte(uint8_t in, uint8_t *out);
void set_pixel(int led_index, uint8_t r, uint8_t g, uint8_t b);
void send_buffer(int len);
void ws2812b_reset(void);
void set_colour(uint8_t r, uint8_t g, uint8_t b);
void spin_animation(int num_leds, uint8_t r, uint8_t g, uint8_t b, int active_leds, int delay); 
void flash_animation(int num_leds, uint8_t r, uint8_t g, uint8_t b, int flashes);

#endif /* WS2812B_H */
