
#include <include.h>//各个模块的头文件
#include <IfxCpu.h>
#include <IfxScuCcu.h>
#include <IfxScuWdt.h>
#include <IfxStm.h>
#include <IfxStm_reg.h>
//#include <LQ_GPIO_KEY.h>
//#include <LQ_GPIO_LED.h>
//#include <LQ_Inductor.h>
#include <LQ_MotorServo.h>
#include <LQ_STM.h>
#include <LQ_TFT18.h>
#include <LQ_UART.h>
#include <Platform_Types.h>
#include <stdio.h>
#include "Came.h"
#include "allinit.h"
#include "ui_hanshu.h"
#include "get_ima01.h"
#include "cam_findmid.h"
#include "UI_run.h"
#include "W25Q.h"


App_Cpu0 g_AppCpu0; // brief CPU 0 global data
IfxCpu_mutexLock mutexCpu0InitIsOk = 1;   // CPU0 初始化完成标志位
volatile char mutexCpu0TFTIsOk=0;         // CPU1 0占用/1释放 TFT



//IfxCpu_syncEvent 不听话;
//IfxCpu_syncEvent 听话;
//IfxCpu_syncEvent CPU坏了;


/*IfxCpu_syncEvent CPU0要听话(IfxCpu_syncEvent 是否听话)
{
    if(是否听话==不听话)
    {
        return 不听话;
    }
    if(是否听话==听话)
    {
        return 听话;
    }
    return CPU坏了;

}*/


int core0_main (void)
{
    //char txt[16];

    /*
                       _ooOoo_
                      o8888888o
                      88" . "88
                      (| -_- |)
                      O\  =  /O
                   ____/`---'\____
                 .'  \\|     |//  `.
                /  \\|||  :  |||//  \
               /  _||||| -:- |||||-  \
               |   | \\\  -  /// |   |
               | \_|  ''\---/''  |   |
               \  .-\__  `-`  ___/-. /
             ___`. .'  /--.--\  `. . __
          ."" '<  `.___\_<|>_/___.'  >'"".
         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
         \  \ `-.   \_ __\ /__ _/   .-` /  /
    ======`-.____`-.___\_____/___.-`____.-'======
                       `=---='
    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                佛祖保佑       永无BUG
    */

	// 关闭CPU总中断
	IfxCpu_disableInterrupts();

	// 关闭看门狗，如果不设置看门狗喂狗需要关闭
	IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
	IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

	// 读取总线频率
	g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
	g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
	g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
	g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);


	// 开启CPU总中断
	IfxCpu_enableInterrupts();

	// 通知CPU2、CPU1，CPU0初始化完成
	IfxCpu_releaseMutex(&mutexCpu0InitIsOk);

	// 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
	mutexCpu0TFTIsOk=0;      // CPU1： 0占用/1释放 TFT

	init_all();
	while(1)
	{
	    UI_begin();
	    main_run(&planc,&runc,&line_fit,&Ind);
	}
    return 0;
}
