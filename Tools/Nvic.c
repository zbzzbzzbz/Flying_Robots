#include "stm32f4xx.h"                  // Device header
#include "Nvic.h"

void MY_NVIC_Init(uint8_t PreemptionPriority,uint8_t SubPriority,uint8_t Channel,uint8_t Group)
{
	uint32_t temp;    

    uint8_t IPRADDR  =Channel/4;  //每组只能存4个,得到组地址 
    uint8_t IPROFFSET = Channel%4;//在组内的偏移
    IPROFFSET = IPROFFSET * 8 + 4;    //得到偏移的确切位置
    Nvic_PriorityGroupConfig(Group);//设置分组
    temp = PreemptionPriority << ( 4 - Group);      
    temp |= SubPriority & (0x0f >> Group);
    temp &= 0xf;//取低四位

    if(Channel<32)
			NVIC->ISER[0] |= 1 << Channel;//使能中断位(要清除的话,相反操作就OK)
    else 
			NVIC->ISER[1] |= 1 <<(Channel - 32);    
    NVIC->IP[IPRADDR] |= temp << IPROFFSET;//设置响应优先级和抢断优先级            

}

void Nvic_PriorityGroupConfig(uint8_t Group)
{
	uint16_t temp,temp1;
    temp1 = (~Group) & 0x07;//取后三位
    temp1 <<= 8;
    temp = SCB->AIRCR;  //读取先前的设置
    temp &= 0X0000F8FF; //清空先前分组
    temp |= 0X05FA0000; //写入钥匙
    temp |= temp1;       
    SCB->AIRCR = temp;  //设置分组   
}


