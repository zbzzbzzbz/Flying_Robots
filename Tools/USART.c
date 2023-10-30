#include "stm32f4xx.h"                  // Device header
#define PLL_M 8
#define PLL_Q 7
#define PLL_N 336
#define PLL_P 2
#define RX 7
#define TX 6
//TX-PB7  RX-PB6
void USART_Init()
{
	RCC->AHB1ENR |= 0x01<<1;
	RCC->APB2ENR |= 0x01<<4;//打开USART1
	
	GPIOB->MODER&=~(0x03<<(2*RX));
	GPIOB->MODER&=~(0x03<<(2*TX));
	GPIOB->MODER|=(0x02<<(2*RX));
	GPIOB->MODER|=(0x02<<(2*TX));//配置模式成复用模式
	
	//PB7PB6推挽输出
	GPIOB->OTYPER&=~(0x01<<RX);
	GPIOB->OTYPER&=~(0x01<<TX);
	
	//速率：100MHz
	GPIOB->OSPEEDR&=~(0x03<<(2*RX));
	GPIOB->OSPEEDR&=~(0x03<<(2*TX));
	GPIOB->OSPEEDR|=(0x03<<(2*RX));
	GPIOB->OSPEEDR|=(0x03<<(2*TX));

	//不需要上拉或下拉 置为00
	GPIOB->PUPDR &= ~(0x03<<2*RX);
	GPIOB->PUPDR &= ~(0x03<<2*TX);
	//配置复用
	GPIOB->AFR[0]&=~(0xF<<4*RX);
	GPIOB->AFR[0]&=~(0xF<<4*TX);
	GPIOB->AFR[0]|=0x07<<4*RX;
	GPIOB->AFR[0]|=0x07<<4*TX;
	
	//USART1->CR1 |= 0x01<<13;//使能USART
	USART1->CR1 &= ~(0x01<<12);//字长
	USART1->CR1 &= ~(0x01<<10);//禁止奇偶校验
	USART1->CR2 &= ~(0x03<<12);//1位停止位
	USART1->CR1 &= ~(0x01<<15);//OVER8 0
	USART1->CR1 |= 0x01<<3;//发送器使能
	USART1->CR1 |= 0x01<<2;//接收器使能
	//设置时钟
	USART1->BRR=0x1111;
	//禁止硬件流控制
	USART1->CR3 &= ~(0x03<<8);
	USART1->CR3 &= ~(0x03<<9);
	USART1->CR1 |= 0x01<<13;//使能USART
}

void USART_sand_byte(char data)
{
	USART1->DR = data;
	while((USART1->SR&(0x01<<7))==0);//发送寄存器为空
	while(!(((USART1->SR>>6)&0x01)==0x01));//等待TC置1
	USART1->SR &= ~(0x01<<6);
}

uint8_t USART_read_byte()
{
	uint8_t data;
	while((USART1->SR&(0x01<<5))==0);//接收到字符
	data = USART1->DR & (uint16_t)0x01FF;
	return data;
}

uint32_t USART_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

void USART_sand_Num(uint32_t Number,uint8_t Length)
{      	
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		USART_sand_byte( Number / USART_Pow(10, Length - i - 1) % 10 + '0');
	}
}


