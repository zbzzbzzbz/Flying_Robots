#ifndef __PWMIN_H
#define __PWMIN_H

struct recline{
	int lineup;//等于1时说明通过检查
	uint32_t chs[9];
	int now;
};
void PWMIN_Init(uint16_t ARR,uint16_t PSC);
void TIM1_CC_IRQHandler(void);
#endif
