#include "ENcoder.h"

//unsigned int i_enc=0; //enc����err����


/*
 * @use ��ȡ��������ֵ��ʾ������������
 */
signed short GetENC(void)
{
    signed short encspeed;
    char enc[40];
    encspeed=ENC_GetCounter(ENC5_InPut_P10_3);
    sprintf(enc, "Enc: %05d;", encspeed);
    OLED_P6x8Str(10,0,enc);
 //   UART_PutStr(UART0,enc);
    return encspeed;
}


/*
 * @use ��err����pid���㲢����
 */
/*int ENC_pid(signed short enc,Dian *speed,PID_E *pid)
{
    int ie=0;
    pid->err_bb=pid->err_b;
    pid->err_b=pid->err;
    pid->err=(float)speed->speedaim-enc;

    if(i_enc>=15) i_enc=0; //ǰ200ms����
    pid->err_sum[i_enc]=pid->err;
    for(ie=0,pid->errsum=0;ie<15;ie++)
    {
        pid->errsum+=pid->err_sum[ie];
    }
    return ((pid->err*pid->p + (pid->err-pid->err_b)*pid->d + (pid->i*pid->errsum)/20)/100);
}*/


/*
 * @use ��speed ͨ��pid������
 */
/*void Setspeed(Dian *speed,PID_E *pid)
{
//    Enc_pid(GetENC(),speed,pid);
    int speed2dian;
    char speed2lan[7];
    speed2dian=speed->speedbase+ENC_pid(GetENC(),speed,pid);
    sprintf(speed2lan, "%07d;", speed2dian);
    UART_PutStr(UART0,speed2lan);
    if(speed2dian>=0)
    {
        if(speed2dian>=9999)
        {
            speed2dian=9999;
        }
        ATOM_PWM_SetDuty(IfxGtm_ATOM0_1N_TOUT55_P21_4_OUT, speed2dian, 17000);
        ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0, 17000);
    }
    else
    {
        speed2dian=-speed2dian;
        if(speed2dian>=2000)
        {
            speed2dian=2000;
        }
        ATOM_PWM_SetDuty(IfxGtm_ATOM0_1N_TOUT55_P21_4_OUT, 0, 17000);
        ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, speed2dian, 17000);
    }
}*/



/*
 * @use �ٶȸ���"Dian"�ṹ��
 *
 */
void Change_speed(Dian *speed , int speedba ,int speedai)
{
    speed->speedaim=speedai;
    speed->speedbase=speedba;
}

/*
 * @use ���ֹͣ����
 */
void Stop_car(void)
{
    ATOM_PWM_SetDuty(IfxGtm_ATOM0_1N_TOUT55_P21_4_OUT, 0, 17000);
    ATOM_PWM_SetDuty(IfxGtm_ATOM0_3_TOUT56_P21_5_OUT, 0, 17000);
}


