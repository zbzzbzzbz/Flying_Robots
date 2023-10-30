#include "stm32f4xx.h"  

void myLED()
{
	*(unsigned int *)(0x40023800+0x30) |= 1ul;//打开GPIOA的时钟
	
	*(unsigned int *)0x40020000 &= ~(3ul<<10);
	*(unsigned int *)0x40020000 |= (1ul<<10); //PO5口改为输出模式
	 
	*(unsigned int *)(0x40020000+0x04)&= ~(1ul<<5);//推挽模式

	*(unsigned int *)(0x40020000+0x08)|= (2ul<<10);//速度为快速（常用）

	*(unsigned int *)(0x40020000+0x0C)&= ~(3ul<<10);//无上拉或下拉

	*(unsigned int *)(0x40020000+0x14)|= (1ul<<5);//1输出高电平
//	
//	while(1)
//	{
//		*(unsigned int *)(0x40020000+0x14)|= (1ul<<5);//1输出高电平
//	}	

}

void myLEDOFF()
{
	*(unsigned int *)(0x40020000+0x14)&= ~(1ul<<5);//1输出高电平
}
