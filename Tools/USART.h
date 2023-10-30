#ifndef __USART_H
#define __USART_H

void USART_Init(void);
void USART_sand_byte(char data);
uint8_t USART_read_byte(void);
void SetSysClock(void);
void USART_sand_Num(uint32_t Number,uint8_t Length);
#endif
