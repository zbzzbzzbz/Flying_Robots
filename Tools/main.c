#include "includes.h"
#include "stm32f4xx.h"                  // Device header
#include "SEGGER_SYSVIEW.h"
#include <time.h>


//引入PWMIN.c结构体
extern struct recline recData;

//创建angle结构体
struct Angle * angle;

//start任务
#define START_TASK_PRIO 20
#define START_TASK_SIZE 64
OS_STK START_TASK_STK[START_TASK_SIZE];
void start_task(void *pdata);

//led任务
#define LED_TASK_PRIO 6
#define LED_TASK_SIZE 64
OS_STK LED_TASK_STK[LED_TASK_SIZE];
void led_task(void *pdata);

//ledoff任务
#define LEDOFF_TASK_PRIO 8
#define LEDOFF_TASK_SIZE 64
OS_STK LEDOFF_TASK_STK[LEDOFF_TASK_SIZE];
void ledoff_task(void *pdata);


//MPU6050
#define MPU6050_TASK_PRIO 11
#define MPU6050_TASK_SIZE 512
OS_STK MPU6050_TASK_STK[MPU6050_TASK_SIZE];
void mpu6050_task(void *pdata);

//ppm
#define PPM_TASK_PRIO 10
#define PPM_TASK_SIZE 1024
OS_STK PPM_TASK_STK[PPM_TASK_SIZE];
void ppm_task(void *pdata);

//pwmout
#define PWMOUT_TASK_PRIO 12
#define PWMOUT_TASK_SIZE 1024
OS_STK PWMOUT_TASK_STK[PWMOUT_TASK_SIZE];
void pwmout_task(void *pdata);

//void HardFault_Handler(void);


int main(void) 
{
	
	#if	OS_CRITICAL_METHOD==3 
		OS_CPU_SR cpu_sr;
	#endif
	myInit();
	angle =(struct Angle *)malloc(sizeof(struct Angle));
	angle->pitch = 0.0f;
	angle->roll = 0.0f;
	angle->yaw = 0.0f;
	

	//OS_TRACE_INIT();
	//OS_TRACE_START();



	
	OSInit();
	OS_ENTER_CRITICAL();     //进入临界区的代码
	OSTaskCreate(start_task,(void *)0,(OS_STK*)&START_TASK_STK[START_TASK_SIZE-1],START_TASK_PRIO);//创建开始任务
	OS_EXIT_CRITICAL();   //跳出临界区的代码
    OSStart();
}

void start_task(void *pdata)
{
	//myLED();
	#if	OS_CRITICAL_METHOD==3 
		OS_CPU_SR cpu_sr;
	#endif
	OS_ENTER_CRITICAL();     //进入临界区的代码
	OSTaskCreate(led_task,(void *)0,(OS_STK*)&LED_TASK_STK[LED_TASK_SIZE-1],LED_TASK_PRIO);
	//OSTaskCreate(ledoff_task,(void *)0,(OS_STK*)&LEDOFF_TASK_STK[LEDOFF_TASK_SIZE-1],LEDOFF_TASK_PRIO);
	//OSTaskNameSet(6,(INT8U *)"LED_ON",(INT8U *)"LED_ERR");
	//OSTaskNameSet(8,(INT8U *)"LED_OFF",(INT8U *)"LED_ERR");
	
	OSTaskCreate(mpu6050_task,(void *)0,(OS_STK*)&MPU6050_TASK_STK[MPU6050_TASK_SIZE-1],MPU6050_TASK_PRIO);
	//OSTaskCreate(ppm_task,(void *)0,(OS_STK*)&PPM_TASK_STK[PPM_TASK_SIZE-1],PPM_TASK_PRIO);
//	OSTaskNameSet(11,(INT8U *)"mpu",(INT8U *)"mpu_err");
//	OSTaskNameSet(10,(INT8U *)"pwmin",(INT8U *)"pwm_err");
	
	//电机驱动
	//OSTaskCreate(pwmout_task,(void *)0,(OS_STK*)&PWMOUT_TASK_STK[PWMOUT_TASK_SIZE-1],PWMOUT_TASK_PRIO);
	//OSTaskNameSet(12,(INT8U *)"pwmout",(INT8U *)"pwmout_err");
	
	OSTaskSuspend(START_TASK_PRIO);//后面用不到开始任务，所以将其永久挂起
	OS_EXIT_CRITICAL();   //跳出临界区的代码
}

void led_task(void * pdata)
{
	
	while(1)
	{
		myLED();
		newdelay1(10);
		OSTimeDly(1000);
		myLEDOFF();
		newdelay1(10);
		OSTimeDly(1000);

	}
}

void ledoff_task(void * pdata)
{
	
	while(1)
	{
		myLEDOFF();
		//newdelay1(100);
		//OSTimeDly(1000);
	}
}

void mpu6050_task(void *pdata)
{	

	while(1)
	{
//		OLED_ShowHexNum2(1,1,MPU_dataout(0x43,0x44),4);//陀螺仪
//		OLED_ShowHexNum2(1,6,MPU_dataout(0x45,0x46),4);
//		OLED_ShowHexNum2(1,11,MPU_dataout(0x47,0x48),4);
//		
//		OLED_ShowHexNum2(3,1,MPU_dataout(0x3B,0x3C),4);//加速度
//		OLED_ShowHexNum2(3,6,MPU_dataout(0x3D,0x3E),4);
//		OLED_ShowHexNum2(3,11,MPU_dataout(0x3F,0x40),4);
		//myLED();

		attitude_Solution(angle);


//		OLED_ShowSignedNum2(1,1,angle->roll,4);
//		OLED_ShowSignedNum2(2,1,angle->pitch,4);
//		OLED_ShowSignedNum2(3,1,angle->yaw,4);		//测出精准Yaw需要地磁计融合
//		short mxxx,myyy,mzzz;
//		OLED_ShowSignedNum2(3,1,HMC5883_Get_Angle(&mxxx,&myyy,&mzzz),4);
		
//		USART_sand_Num(angle->roll,4);
//		USART_sand_byte('A');
//		USART_sand_Num(angle->pitch,4);
//		USART_sand_byte(' ');
//		USART_sand_Num(angle->yaw,4);
//		USART_sand_byte(0x0A);
		//OSTimeDly(1);
	}	
}

void ppm_task(void *pdata)
{
	
	while(1)
	{
		if(recData.now==8)
		{
			USART_sand_Num(recData.chs[0],4);//通道1
			USART_sand_Num(recData.chs[1],4);//通道2
			USART_sand_Num(recData.chs[2],4);//通道3
			USART_sand_Num(recData.chs[3],4);//通道4
			USART_sand_byte(0x0A);
			OSTimeDly(200);
//			USART_sand_byte(0x38);
//			USART_sand_byte(0x0A);
			
		}
	}
}


void pwmout_task(void *pdata)
{
	while(1)
	{
		for(int i=0;i<100;i++)
		{
			myCCR3(i);
		}
		//newdelay1(100);
		//OSTimeDly(200);
	}
}

void myInit(void)
{
	SetSysClock();
	//设置系统时钟为PLL AHB为84	对于411板子，不加上该函数则系统时钟不是84MHz
	
	SysTick_Config(84000);
	
	I2C_Configuration();
//	OLED_Init2();
	MPU_Init2();
	HMC5883L_Init();
	gyzero_err();//传感器零偏矫正
	USART_Init();
	PWMIN_Init(0xFFFF,84-1);
	PWM_Init(100-1,840-1,25);//1s
	//myLED();
	
}



//void HardFault_Handler(void)
//{
//	while(1)
//	{
//	
//	}
//}
