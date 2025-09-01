#ifndef DFPLAYER_H
#define DFPLAYER_H

#define SYSCLK 32000000L

#define TX_PORT GPIOA
#define TX_PIN 2
#define RX_PORT GPIOA
#define RX_PIN 3

void USART2_init(void); 
void delay(volatile uint32_t count); 
void USART2_send_byte(uint8_t byte);
uint8_t USART2_receive_byte(void); 
void send_cmd(uint8_t cmd, uint8_t parameter1, uint8_t parameter2);
void DF_init(void);
void DF_play(uint8_t folder, uint8_t track);

#endif /* DFPLAYER_H */