#include <stdio.h>
#include <stdbool.h>
#include "includes.h"
#include "ucos_ii.h"

typedef unsigned char  INT8U;
typedef unsigned short INT16U;
typedef unsigned int INT32U;

typedef signed   char  INT8S;                   
typedef signed   short INT16S;                   
typedef signed   int   INT32S;                  
typedef float          FP32;                    
typedef double         FP64; 

typedef unsigned char BOOLEAN;
typedef unsigned int OS_STK;   //堆栈入口的宽度为32位
typedef unsigned int OS_CPU_SR; //定义CPU状态寄存器的宽度为16位，但是在ARM中是32位所以在移植时要做改动

#define OS_CRITICAL_METHOD   3        

#if	OS_CRITICAL_METHOD==3 
/********进入临界代码区********************/
#define OS_ENTER_CRITICAL()  {cpu_sr =OS_CPU_SR_Save();}
/*********退出临界代码区*****************/
#define OS_EXIT_CRITICAL()  {OS_CPU_SR_Restore(cpu_sr);}
#endif
 
OS_CPU_SR OS_CPU_SR_Save(void);
void OS_CPU_SR_Restore(OS_CPU_SR cpu_sr);


#define OS_TASK_SW()	OSCtxSw();
#define OS_STK_GROWTH   1  //定义堆栈的方向：1=向下递减，0=向上递增

void OSCtxSw(void);
void OSIntCtxSw(void);
//void PendSV_Handler(void);
void OSStartHighRdy(void);
void OSTaskSwHook(void);
//void OSTaskReturnHook(OS_TCB *ptcb);
