#include "stm32f4xx.h"                  // Device header

//PC8作为PWM输出口
//TIM3在APB1总线上，APB1:21Hz	,TIM3:42Hz
void PWM_Init(uint16_t ARR,uint16_t PSC,uint16_t CCR)
{
	//配置RCC时钟
	RCC->AHB1ENR |= 0x01<<2;//GPIOC
	RCC->APB1ENR |= 0x01<<1;//TIM3
	
	GPIOC->MODER&=~(0x03<<(2*8));//复用
	GPIOC->MODER|=(0x02<<(2*8));
	GPIOC->OTYPER&=~(0x01<<8);//推挽输出
	GPIOC->OSPEEDR&=~(0x03<<(2*8));
	GPIOC->OSPEEDR|=(0x03<<(2*8));//100MHz
	GPIOC->PUPDR &= ~(0x03<<2*8);//无需上下拉
	GPIOC->AFR[1]&=~(0xF);//配置引脚复用AF2
	GPIOC->AFR[1]|=0x02;
	
	
	//配置时基单元
	TIM3->ARR = ARR;//自动重载寄存器
	TIM3->PSC = PSC;//预分频
	
	
	//配置输出比较单元
	TIM3->CCMR2 |= 0x06<<4;//pwm模式1
	TIM3->CCMR2 |= 0x01<<3;//预装CCR载使能
	TIM3->CCER |= 0x01<<8;//OC3输出使能，且高电平有效
	TIM3->CR1 |= 0x01<<7;//自动重装预装载使能
	TIM3->CCR3 =  CCR;//写入CCR值
	TIM3->CR1 |= 0x01;//使能计数器
	
}

void myCCR3(uint16_t CCR)
{
	TIM3->CCR3&=~(0xFF);
	TIM3->CCR3 = CCR;
}
