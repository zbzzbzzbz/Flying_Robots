#ifndef __NVIC_H
#define __NVIC_H
void Nvic_PriorityGroupConfig(uint8_t Group);
void MY_NVIC_Init(uint8_t PreemptionPriority,uint8_t SubPriority,uint8_t Channel,uint8_t Group);
#endif
