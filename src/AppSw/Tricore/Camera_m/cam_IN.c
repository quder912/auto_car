#include "cam_IN.h"
#include "Came.h"
#include "UI_run.h"
#include "allinit.h"
#include "ui_hanshu.h"
#include "LQ_DMA.h"
#include "ui_hanshu.h"
#include "stdio.h"
#include "stdlib.h"

uint8 Ring_in_bit;

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
                            ·ð×æ±£ÓÓ       ÓÀÎÞBUG
*/


void camgetIN(CAM_Inductance *ind)
{
    ind->Inleft =ADC_ReadAverage(Inleft_IO,8);
    ind->Inright=ADC_ReadAverage(Inright_IO,8);
    ind->Inlevell=ADC_ReadAverage(Inlevell_IO,8);
    ind->Inlevelr=ADC_ReadAverage(Inlevelr_IO,8);

    ind->Inlevell = (ind->Inlevell>1 )?ind->Inlevell :1 ;
    ind->Inlevelr= (ind->Inlevelr>1)?ind->Inlevelr :1 ;
    ind->Inleft = (ind->Inleft>1 )?ind->Inleft :1 ;
    ind->Inright= (ind->Inright>1)?ind->Inright :1 ;
    ind->levellonce=(float)ind->Inlevell*100/ind->Inlevellmax ;
    ind->levelronce=(float)ind->Inlevelr*100/ind->Inlevelrmax;
    ind->leftonce =(float)ind->Inleft *100/ind->Inleftmax ;
    ind->rightonce=(float)ind->Inright*100/ind->Inrightmax;

    ind->lefttwice=ind->leftonce*600.0/(ind->leftonce+ind->rightonce);
    ind->levelltwice=ind->levellonce *600.0/(ind->levelronce +ind->levellonce);
    return;
}

unsigned char camIN2pidcamp(CAM_Inductance *ind,CAM_var *pla)
{
    char txt[21];
    ind->lefttwice=ind->leftonce*600.00/(ind->leftonce+ind->rightonce);
    ind->err_b=ind->err;
    ind->err=(short)(ind->lefttwice-300);
    sprintf(txt,"ercamp:  %05d   ",ind->err);
    OLED_Write68(3,1,txt,0);
    ind->PID_OUT=(ind->err*pla->Ist.p + (ind->err-ind->err_b) * pla->Ist.d)/50;
    ind->PID_OUT =(ind->PID_OUT>300)?300:((ind->PID_OUT<-300)?-300:ind->PID_OUT);
    ATOM_PWM_SetDuty(STEER_IO, pla->D_middle+ind->PID_OUT, 100);
    return 0;
}

unsigned char camIN2pidcamp1(CAM_Inductance *ind,CAM_var *pla)
{
    char txt[21];
    ind->lefttwice=ind->leftonce*600.00/(ind->leftonce+ind->rightonce);
    ind->err_b=ind->err;
    ind->err=(short)(ind->lefttwice-300);
    sprintf(txt,"ercamp:  %05d   ",ind->err);
    OLED_Write68(3,1,txt,0);
    ind->PID_OUT=(ind->err*16 + (ind->err-ind->err_b) * 65)/50;
    ind->PID_OUT =(ind->PID_OUT>300)?300:((ind->PID_OUT<-300)?-300:ind->PID_OUT);
    ATOM_PWM_SetDuty(STEER_IO, pla->D_middle+ind->PID_OUT, 100);
    return 0;
}


unsigned char camIN2pidcampout(CAM_Inductance *ind,CAM_var *pla)
{
    char txt[21];
    ind->lefttwice=ind->leftonce*600.00/(ind->leftonce+ind->rightonce);
    ind->err_b=ind->err;
    ind->err=(short)(ind->lefttwice-300);
    sprintf(txt,"ercamp:  %05d   ",ind->err);
    OLED_Write68(3,1,txt,0);
    ind->PID_OUT=(2*ind->err*pla->Ist.p +2* (ind->err-ind->err_b) * pla->Ist.d)/100;
    ind->PID_OUT =(ind->PID_OUT>300)?300:((ind->PID_OUT<-300)?-300:ind->PID_OUT);
    ATOM_PWM_SetDuty(STEER_IO, pla->D_middle+ind->PID_OUT, 100);
    return 0;
}

void camIN2pid(CAM_Inductance *ind,CAM_var *pla)
{
    char txt[21];
    ind->lefttwice=ind->leftonce*600.00/(ind->leftonce+ind->rightonce);
    ind->err_b=ind->err;
    ind->err=(short)(ind->lefttwice-300.00);
    sprintf(txt,"err:   %05d  ",ind->err);
    OLED_Write68(3,1,txt,0);
    if(abs(ind->err)>pla->err_istcu)
    {
        ind->PID_OUT=(ind->err*pla->Icu.p + (ind->err-ind->err_b) * pla->Icu.d)/50;
    }
    else
    {
        ind->PID_OUT=(ind->err*pla->Ist.p + (ind->err-ind->err_b) * pla->Ist.d)/50;
    }
    ind->PID_OUT =(ind->PID_OUT>300)?300:((ind->PID_OUT<-300)?-300:ind->PID_OUT);
    ATOM_PWM_SetDuty(STEER_IO, pla->D_middle+ind->PID_OUT, 100);
    return;
}

void camrunIN(CAM_Inductance *indu,CAM_var *plan)
{
    char txt[20];
    OLED_CLS();
    cam_jishi=0;
    UI_writeinflash();
    ATOM_PWM_SetDuty(MOTOR_Forward_IO, 2500, 17000);
    runbeginflag=1;
    while(1)
    {
        if(Camera_Flag == 2)
        {
  //          camgetIN(indu);
            camIN2pid(indu,plan);
            Camera_Flag =0;
            cam_jishi++;
            sprintf(txt,"time: %d ",cam_jishi);
            OLED_Write68(10,0,txt,1);
        }
    }
}

unsigned char cam_Ring_judge(CAM_Inductance *indu)
{
  //  char txt[20];

    if(indu->levellonce+indu->levelronce>planc.Ring_sumIN && indu->leftonce+indu->rightonce>planc.Ring_sum8)
    {
        Ring_in_bit++;
        if(Ring_in_bit>2)
        {
            cam_Ringin_flag=1;
 //           PIN_Write(P20_3,1);
            return 1;
        }
        return 0;
    }
    else
    {
        cam_Ringin_flag=0;
        Ring_in_bit=0;
        cam_Ring_in=0;
        return 0;
    }
}

void cam_Outgarage(CAM_var *pla)
{
    uint8 mid_dirdelay;

    mid_dirdelay=pla->delay_outgarmid;
    while(mid_dirdelay>1)
    {
        if(Camera_Flag==2)
        {
            ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle, 100);
            Camera_Flag=0;
            mid_dirdelay--;
            cam_jishi++;
        }

    }
    mid_dirdelay=pla->delay_Out;
    while(mid_dirdelay>1)
    {
        if(Camera_Flag==2)
        {
            if(pla->Outga_direction)
            {
                ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle+pla->angle_Outgar, 100);
            }
            else
            {
                ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle-pla->angle_Outgar, 100);
            }
            Camera_Flag=0;
            mid_dirdelay--;
            cam_jishi++;
        }
    }
    cam_Outga=0;
}


