#ifndef TIM1637_H
#define TIM1637_H

#define SYSCLK 32000000L

#define TIM1637_CLK_PORT GPIOB
#define TIM1637_CLK_PIN 1
#define TIM1637_DIO_PORT GPIOB
#define TIM1637_DIO_PIN 0 

#define HOME_READY_PORT GPIOA
#define HOME_READY_PIN 5
#define AWAY_READY_PORT GPIOA
#define AWAY_READY_PIN 1 

#define HOME_LED_PORT GPIOB
#define HOME_LED_PIN 4 
#define AWAY_LED_PORT GPIOB
#define AWAY_LED_PIN 5 

void tm1637Init(void); 
void _tm1637Start(void);
void _tm1637Stop(void);
void _tm1637WriteByte(unsigned char b);
void _tm1637ReadResult(void);
void _tm1637ClkHigh(void);
void _tm1637ClkLow(void);
void _tm1637DioHigh(void);
void _tm1637DioLow(void);
void tm1637SetBrightness(char brightness);
void tm1637DisplayDecimal(int v, int displaySeparator); 
void tm1637ScrollMessage(const char* message, int delay_ms); 
char chartosegment(char c); 
void checkReady(void);

#endif /* TIM1637_H */
