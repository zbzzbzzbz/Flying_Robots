#ifndef __oled2_H
#define __oled2_H

void OLED_Init2(void);
void OLED_Clear2(void);
void OLED_ShowChar2(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString2(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum2(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum2(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum2(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum2(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_Showdecimal2(uint8_t x,uint8_t y,float num,uint8_t z_len,uint8_t f_len,uint8_t size2);
#endif
