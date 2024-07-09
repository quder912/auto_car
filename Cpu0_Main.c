
#include <include.h>//����ģ���ͷ�ļ�
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
IfxCpu_mutexLock mutexCpu0InitIsOk = 1;   // CPU0 ��ʼ����ɱ�־λ
volatile char mutexCpu0TFTIsOk=0;         // CPU1 0ռ��/1�ͷ� TFT



//IfxCpu_syncEvent ������;
//IfxCpu_syncEvent ����;
//IfxCpu_syncEvent CPU����;


/*IfxCpu_syncEvent CPU0Ҫ����(IfxCpu_syncEvent �Ƿ�����)
{
    if(�Ƿ�����==������)
    {
        return ������;
    }
    if(�Ƿ�����==����)
    {
        return ����;
    }
    return CPU����;

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
                ���汣��       ����BUG
    */

	// �ر�CPU���ж�
	IfxCpu_disableInterrupts();

	// �رտ��Ź�����������ÿ��Ź�ι����Ҫ�ر�
	IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
	IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

	// ��ȡ����Ƶ��
	g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
	g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
	g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
	g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);


	// ����CPU���ж�
	IfxCpu_enableInterrupts();

	// ֪ͨCPU2��CPU1��CPU0��ʼ�����
	IfxCpu_releaseMutex(&mutexCpu0InitIsOk);

	// �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
	mutexCpu0TFTIsOk=0;      // CPU1�� 0ռ��/1�ͷ� TFT

	init_all();
	while(1)
	{
	    UI_begin();
	    main_run(&planc,&runc,&line_fit,&Ind);
	}
    return 0;
}
