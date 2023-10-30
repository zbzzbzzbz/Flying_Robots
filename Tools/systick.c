#include "includes.h"
#include "stm32f4xx.h"                  // Device header

//AHB为84MHz ,systick的频率也是84MHz
uint32_t myticks = 84000;
uint32_t delay_time = 0;

#define PLL_M 8
#define PLL_Q 7
#define PLL_N 336
#define PLL_P 2

void SysTick_Handler()
{
	#if	OS_CRITICAL_METHOD==3 
		OS_CPU_SR cpu_sr;
	#endif
	OS_ENTER_CRITICAL();
	OSIntEnter();
	OS_EXIT_CRITICAL();
    OSTimeTick();                                /* Call uC/OS-II's OSTimeTick()                       */
    OSIntExit();                                 /* Tell uC/OS-II that we are leaving the ISR */    

}

//使用循环来进行延时(t的单位为ms)
void newdelay1(uint32_t t)
{
	SysTick_Config(myticks);//每个tick 1ms
	for(int i=0;i<t;i++)
	{
		while(!((SysTick->CTRL>>16)&&0x01))//计数器值为0
		{
			;
		}
	
	}
}

////使用中断来进行延时（没过一个tick就进入一次中断函数）
void newdelay2(uint32_t t)
{
	//SysTick_Config(myticks);
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;//先失能
	delay_time = t;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	while(delay_time!=0);
}



void SetSysClock(void)
{

  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
  /* Enable HSE */ //HSE时钟使能
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
  //等待HSE时钟稳定
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;
  } while((HSEStatus == 0) && (StartUpCounter != 0x0500));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01)
  {
    /* Select regulator voltage output Scale 1 mode */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;
	//AHB分2频
    /* HCLK = SYSCLK / 2*/
    RCC->CFGR |= RCC_CFGR_HPRE_DIV2 ;

 /* PCLK2 = HCLK / 2*/
    //RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
    /* PCLK1 = HCLK / 4*/
	  //APB1分4频
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
  }
  
  RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
                   (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);
  
  /* Enable the main PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till the main PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0);
  
  FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
   /* Select the main PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    /* Wait till the main PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL);
    
}

