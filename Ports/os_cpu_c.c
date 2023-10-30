#include "includes.h"

void OSInitHookBegin(){}
void OSInitHookEnd(){}
//void OSIntCtxSw(){}
//void OSStartHighRdy(){}
//暂不考虑Hook函数,Hook函数主要作用为调试时供用户添加输出信息
void OSTCBInitHook(OS_TCB * ptcb){}
void OSTaskCreateHook(OS_TCB * ptcb){}
void OSTaskIdleHook(){}
void OSTaskStatHook(){}
void OSTimeTickHook(){}
void OSTaskDelHook(OS_TCB * ptcb){}
void OSTaskReturnHook(OS_TCB* ptcb){
	SYSVIEW_TaskSwitchedIn((U32)ptcb);
}
void OSTaskSwHook(void){}

OS_STK *OSTaskStkInit (void (*task)(void *p_arg), void *p_arg, OS_STK *ptos, INT16U opt)
{
    OS_STK *stk;


    (void)opt;                                   /* 'opt' 并没有用到，防止编译器提示警告  */
    stk       = ptos;                            /* 加载栈指针 */

    //发生中断后，xPSR, PC, LR, R12, R3-R0被自动保存在栈中
	//在初始化时我们要占好位置
    *(stk)    = (INT32U)0x01000000uL;            /* xPSR                                               */
    *(--stk)  = (INT32U)task;                    /* Entry Point(任务的入口)                            */
    *(--stk)  = (INT32U)OS_TaskReturn;           /* R14 (LR)                                           */
    *(--stk)  = (INT32U)0x12121212uL;            /* R12                                                */
    *(--stk)  = (INT32U)0x03030303uL;            /* R3                                                 */
    *(--stk)  = (INT32U)0x02020202uL;            /* R2                                                 */
    *(--stk)  = (INT32U)0x01010101uL;            /* R1                                                 */
    *(--stk)  = (INT32U)p_arg;                   /* R0 : 变量                                          */

    /* 剩下的寄存器需要手动保存在堆栈         */                                             
    *(--stk)  = (INT32U)0x11111111uL;            /* R11                                                */
    *(--stk)  = (INT32U)0x10101010uL;            /* R10                                                */
    *(--stk)  = (INT32U)0x09090909uL;            /* R9                                                 */
    *(--stk)  = (INT32U)0x08080808uL;            /* R8                                                 */
    *(--stk)  = (INT32U)0x07070707uL;            /* R7                                                 */
    *(--stk)  = (INT32U)0x06060606uL;            /* R6                                                 */
    *(--stk)  = (INT32U)0x05050505uL;            /* R5                                                 */
    *(--stk)  = (INT32U)0x04040404uL;            /* R4                                                 */

    return (stk);
}
