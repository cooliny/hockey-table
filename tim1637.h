#ifndef TIM1637_H
#define TIM1637_H

#define SYSCLK 32000000L

void tm1637Init(void); 
void _tm1637Start(void);
void _tm1637Stop(void);
void _tm1637ReadResult(void);
void _tm1637WriteByte(unsigned char b);
void _tm1637DelayUsec(unsigned int i);
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
