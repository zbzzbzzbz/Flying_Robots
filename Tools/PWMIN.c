#include "stm32f4xx.h"                  // Device header
#include "LED.h"
#include "includes.h"
#include "PWMIN.h"
//注意： 该PWM频率=84M/（PSC+1）/（ARR+1）
//APB2 42M TIM1:84MHz
//TIM1在APB2上


struct	recline recData;

struct	recline recline1;


static uint32_t PPM_Time=0;//暂时存放CNT的值

void PWMIN_Init(uint16_t ARR,uint16_t PSC)
{
	//初始化recline1结构体
	recData.now = 0;
	recData.lineup = 0;
	//配置RCC时钟
	RCC->AHB1ENR |= 0x01;//GPIOA PA8 TIM1_CH1
	RCC->APB2ENR |= 0x01;//TIM1
	
	// set input mode TI1 & no filter	
	TIM1->CR1 |= (0x1 << 7);

	//配置引脚为输入模式 复用模式
	GPIOA->MODER |=0x02<<(2*8);//input
	GPIOA->PUPDR &= ~(0x03<<2*8);//下拉输入
	GPIOA->PUPDR |= 0x02<<2*8;
	GPIOA->AFR[1] |= 0x01;
	
	//配置时基单元
	TIM1->ARR = ARR;//自动重载寄存器
	TIM1->PSC = 84-1;//预分频

	
	//配置输入捕获单元
	TIM1->CCMR1 |= 0x01;//01：CC1 通道配置为输入，IC1 映射到 TI1 上

	TIM1->DIER |= 0X01<<1;//捕获/比较 1 中断使能
	NVIC->ISER[0] |= 0x01<<27;//中断使能 对应中断号为27

	TIM1->CR1 |= 0x01;//使能计数器
	TIM1->CCER |= 0x01;//1：使能捕获。
}

//写中断函数
void TIM1_CC_IRQHandler(void)
{

//	#if	OS_CRITICAL_METHOD==3 
//		OS_CPU_SR cpu_sr;
//	#endif
//	
	//OS_ENTER_CRITICAL();
	PPM_Time = TIM1->CCR1;
	TIM1->CNT = 0;//获取计数器数值
	
	recData.chs[recData.now]=PPM_Time;
	
	
	if(recData.chs[recData.now]>3000||recData.now==8)
	{
		recData.now=-1;//重新开始
	}
	recData.now++;
	//OS_EXIT_CRITICAL();
	
	TIM1->SR &= ~(0x01);//清除中断标志位
}

