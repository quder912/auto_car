#include "allinit.h"


#include <LQ_GTM.h>
#include "UI_run.h"
#include "Came.h"
#include "W25Q.h"



volatile unsigned long time_run =0;//10ms++




void ADC_init(void)
{
    ADC_InitConfig(Inleft_IO, 500000);
    ADC_InitConfig(Inlevelr_IO, 500000);
    ADC_InitConfig(Inlevell_IO, 500000);
    ADC_InitConfig(Inright_IO, 500000);
    ADC_InitConfig(Invertic_IO, 500000);

}

/*
 * led(���İ壬���������ѱ�pwmռ�ã�||
 */

void GPIO_init(void)
{
    //���İ�led
    PIN_InitConfig(P21_4,PIN_MODE_OUTPUT,1);
    PIN_InitConfig(P21_5,PIN_MODE_OUTPUT,0);
    PIN_InitConfig(P20_8,PIN_MODE_OUTPUT,1);
    PIN_InitConfig(P20_9,PIN_MODE_OUTPUT,0);


    //����
    PIN_InitConfig(P11_10,PIN_MODE_INPUT,1);
    PIN_InitConfig(P11_6,PIN_MODE_INPUT,1);


    //������
    PIN_InitConfig(P20_3,PIN_MODE_OUTPUT,0);


    //OLED��������
    PIN_InitConfig(P20_13,PIN_MODE_INPUT,1);//Key6   ȷ��
    PIN_InitConfig(P15_0,PIN_MODE_INPUT,1);//       �¼�
    PIN_InitConfig(P15_2,PIN_MODE_INPUT,1);//       ���
    PIN_InitConfig(P15_3,PIN_MODE_INPUT,1);//       �ϼ�
    PIN_InitConfig(P15_4,PIN_MODE_INPUT,1);//       �Ҽ�
    PIN_InitConfig(P15_5,PIN_MODE_INPUT,1);//Key1   ����


    //OLED ��Ļ
    PIN_InitConfig(P20_10,PIN_MODE_OUTPUT,1);// RST
    PIN_InitConfig(P20_12,PIN_MODE_OUTPUT,1);// D/C
    PIN_InitConfig(P20_11,PIN_MODE_OUTPUT,1);// SCK
    PIN_InitConfig(P20_14,PIN_MODE_OUTPUT,1);// SDI


    //��Դʹ��
    PIN_InitConfig(P00_4,PIN_MODE_OUTPUT,1);


    //������
    PIN_InitConfig(P10_3,PIN_MODE_INPUT,0);// LSB
    PIN_InitConfig(P10_1,PIN_MODE_INPUT,0);// DIR



}

/*
 *  ��������ʼ��
 */

void PWM_init(void)
{

    ATOM_PWM_InitConfig(MOTOR_Forward_IO, 0, 10000);
    ATOM_PWM_InitConfig(MOTOR_Backward_IO, 0, 17000);
    ATOM_PWM_InitConfig(STEER_IO, planc.D_middle, 100);//���

    ATOM_PWM_SetDuty(IfxGtm_ATOM0_2N_TOUT56_P21_5_OUT, 0, 17000);
    ATOM_PWM_SetDuty(IfxGtm_ATOM0_1N_TOUT55_P21_4_OUT, 0, 10000);
    ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, planc.D_middle, 100);

}


/*
 * @use ��ʼ����ʱ��
 */
void tim_init(void)//�õ���STM��ʱ�жϣ��Ժ�Ҫ�ĳ�CCU6
{
    STM_InitConfig(STM0, STM_Channel_0, 10000);//����������ת�� 10ms
   // STM_InitConfig(STM0, STM_Channel_1, 1000000);//1000 000*1us=1s 1s�ص�һ��
    STM_EnableInterrupt(STM0, STM_Channel_0);
 //   STM_EnableInterrupt(STM0, STM_Channel_0);
  //  STM_EnableInterrupt(STM0, STM_Channel_1);

}

/*
 * @use ���pid��ʼ��
 */







void init_all(void)
{
    time_run=0;
    using_OLED=0;
    using_UART0=0;
    ADC_init();
    GPIO_init();
    UI_init();
    PWM_init();
    UART_InitConfig(UART0_RX_P14_1,UART0_TX_P14_0, 115200);
    ENC_InitConfig(ENC5_InPut_P10_3, ENC5_Dir_P10_1);

    ui_first_line=1;
    ui_page=1;
    ui_floor=1;
    ui_report_line=1;
    ui_report_floor=1;
    cam_Ramp_flag=0;
    cam_finish_flag=0;
    cam_Ring_flag=0;
    cam_Ringin_flag=0;
    cam_Ringout_flag=0;
    cam_Ring_num=0;
    cam_Ringin_bit=-180;
    cam_finish_flag=0;
    cam_Ramp_flag=0;
    cam_Stop_flag=0;
    cam_finished_time=0;
    cam_finished_gar=0;
    cam_Three_quar=-350;

    delayms(100);
    W25QXX_Init();
    delayms(100);
    Flash_data_change();
    delayms(100);
    OLED_P6x8Str(2,7," f ");
    CAMERA_Init(100);

}
