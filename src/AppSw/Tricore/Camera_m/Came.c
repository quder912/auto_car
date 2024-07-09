#include "Came.h"
#include "get_ima01.h"
#include "cam_findmid.h"
#include "LQ_OLED096.h"
#include "stdio.h"
#include "math.h"
#include <LQ_GTM.h>
#include <LQ_GPT12_ENC.h>
#include "forreport.h"
#include <LQ_UART.h>
#include <LQ_GPIO.h>
#include "UI_run.h"
#include "stdlib.h"
#include "allinit.h"
#include "cam_IN.h"
#include "LQ_DMA.h"
#include "UI_hanshu.h"

/*
 * 0    time
 * 1    errs
 * 2    (errenc) In8
 * 3    Inlevel
 * 4    Three_stamid
 * 5    Three_stasid
 * 6    Three_up_rl
 * 7    Three_up_down
 */

unsigned short cam_err_queue[22];
char cam_err_queuenum;

_Bool Exist_Three_left[50];
_Bool Exist_Three_right[50];
unsigned char Point_Three_left[50];
unsigned char Point_Three_right[50];

cam_judge *now_judge =&rungo;
Getinfo *now_info =&runinfo;
cam_jurun *now_stat =&runstate;

volatile uint8 imagin_init[120][188];
volatile _Bool imagin_cam[Iwidth][Ilength]; //°×µãÎª1 ºÚµãÎª0
_Bool Lost_Center;
_Bool Lost_Left;
_Bool Lost_Right;
volatile _Bool cam_Ramp_flag=0;
volatile _Bool cam_finished_in=0;
volatile _Bool cam_Stop_flag=0;
volatile _Bool cam_havefinished=0;
volatile _Bool cam_finish_flag=0;
volatile _Bool onefinished=0;
_Bool cam_three_flag;
volatile _Bool cam_Ring_flag=0;
_Bool cam_Ringin_flag;
_Bool cam_Ringout_flag;
unsigned char cam_Ring_num=0;
unsigned char cam_Three_num=0;
int cam_Ringin_bit=0;
unsigned int cam_Ringout_bit=0;
unsigned int Sum_Three_up_Yl;
volatile _Bool finished_stop=0;
unsigned int Sum_Three_up_Yr;
volatile int jishi_quanju=0;
volatile _Bool yuanhuanfinished=0;
volatile _Bool sanchafinished=0;
volatile _Bool sanchaneibu=0;
unsigned char cam_ringout_bit=0;
short ramp_time;
unsigned char npxysta1[2][3000];
_Bool IsVsta1[50][94];
unsigned char numbup;
volatile _Bool ringfinished=0;
unsigned char npxyup[2][3000];
_Bool IsVup[50][94];
short mowpup;
volatile _Bool huandaojiansu=0;
float Sumup_X,Sumup_XX,Sumup_XY,Sumup_Y;
unsigned short blackpnum=0;
_Bool IsVth[60][94];
unsigned char sidepoint[60];
_Bool Exist_side[60];
unsigned char npxyth[2][3000];
unsigned char   Valith;
short nowpth;
float Sumth_X,Sumth_XX,Sumth_XY,Sumth_Y;
int cam_Three_quar=-300;
int cam_finished_gar;
unsigned char cam_finished_time;
unsigned int cam_ramp_gar=0;

CAM_var planc;
CAM_run runc;
Fit_line line_fit;
Subs_proc Three_dec;
Subs_proc FMid_dec;
float kk_tupl=0,kk_tupr=0,kk_tdown=0;
int Cam_Process_Result=0;
float Cam_Err,Pre_Cam_Err;
char cam_Stop_bit;

volatile _Bool cam_Ring_in=0;
volatile _Bool cam_Three_in=0;
volatile _Bool cam_Three_out=0;
volatile _Bool cam_Outga;

_Bool cam_Three_Leftin;
_Bool cam_Three_Rightin;

unsigned short Duo_limitplus=300;
unsigned short Duo_limitminus=300;


void Get_Cam_Process_Val(CAM_var *pla, Fit_line *lin,CAM_Inductance *indu)
{
    char txt[20]={0};

    now_info->getimagin();


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


    FMid_dec.part_4();

    sprintf(txt,"L: %04.1f %04.1f e:%04.1f",indu->levellonce,indu->levelronce,indu->levelltwice-300);
    OLED_P6x8Str(2,3,txt);
    sprintf(txt,"8: %04.1f %04.1f e:%04.1f",indu->leftonce,indu->rightonce,indu->lefttwice-300);
    OLED_P6x8Str(2,2,txt);

    if(cam_jishi<2)
    {
        cam_Outga=1;
        cam_Outgarage(pla);
        onefinished=0;
        return;
    }

//    if(Three_outjudge()==1)
//    {
//        PIN_Write(P20_3,1);
//        return;
//    }
//    Three_Rightin();
//    Three_Leftin();
//    return;
//    if( fabsf(indu->lefttwice-25)<24 && fabsf(indu->levelltwice-25)<23  &&indu->leftonce<77 && indu->rightonce<77
//            && indu->levellonce>12 && indu->levelronce>12 && indu->levellonce <107 && indu->levelronce<107 )
//    {
//        if(Three_outjudge()==1)
//        {
//            PIN_Write(P20_3,1);
//            while(PIN_Read(key_in_IO)==1)
//            {
//            }
//            while(1)
//            {
//                PIN_Write(P20_3,0);
//                    CAMERA_Reprot_switch(2);
//                    delayms(10);
//            }
//            return;
//        }
//        else
//        {
//            PIN_Write(P20_3,0);
//            return;
//        }
        if(now_judge->judge_three()==1 && ((cam_jishi-cam_Three_quar>200)||(cam_jishi<500)))
        {
            PIN_Write(P20_3,1);
//////            cam_Stop_flag=1;
//            while(PIN_Read(key_in_IO)==1)
//            {   sprintf(txt,"l%05.2f",kk_tupl);
//                UART_PutStr(UART0,txt);
//                delayms(500);
//                sprintf(txt,"r%05.2f",kk_tupr);
//                UART_PutStr(UART0,txt);
//                delayms(500);
//            }
            if(onefinished!=0 || sanchafinished==1 || cam_jishi-cam_ramp_gar<40) goto Runnormal;
//                ;
//            PIN_Write(P20_3,0);
//            while(1)
//            {
//                    CAMERA_Reprot_switch(2);
//                    delayms(10);
//            }
////            while(1);
            cam_Three_run(pla,indu,lin);
            sanchafinished=1;
//           cam_Three_in=0;
//          return;
        }
//    }
//    else
//    {
//        OLED_P6x8Str(2,5,".");
//        OLED_P6x8Str(2,4," ");
//        OLED_P6x8Str(10,5," sta66 ");
//    }
//        PIN_Write(P20_3,0);
        if(((cam_jishi-cam_ramp_gar>55 && cam_jishi-cam_ramp_gar<130)||(cam_jishi-cam_ramp_gar>210 && cam_jishi-cam_ramp_gar<240)) && cam_jishi>300 && onefinished!=0)
        {

            huandaojiansu=1;
        }
        else if((cam_jishi-cam_ramp_gar>30 && cam_jishi-cam_ramp_gar<60) && cam_jishi>300 && onefinished==0)
        {
            huandaojiansu=1;
        }
        else
        {
            huandaojiansu=0;
        }



        if( cam_jishi-cam_ramp_gar>100)
        {
            if(now_judge->judge_ramp()==1)
            {
                if(ringfinished==0) goto Runnormal;
                cam_Camprun(pla,indu);
                cam_Ramp_flag=0;
                if(!onefinished)
                {
                    ringfinished=0;
                }
                sanchafinished=0;
            }
        }
//            now_judge->judge_finish();
        if(now_judge->judge_ring(indu)==1 && cam_jishi-cam_Ringin_bit>200)
        {
            if(onefinished!=0) goto Runnormal;
            cam_Ring_run(pla ,indu , lin);
            onefinished=0;
            yuanhuanfinished=1;
            ringfinished=1;
        }

        if(cam_jishi+5-cam_Ringin_bit>0 && cam_jishi-cam_Ringin_bit<35 && cam_jishi>150)
        {
            yuanhuanfinished=1;
        }
        else
        {
            yuanhuanfinished=0;
        }

        Runnormal:

        Find_Center_Line(lin,pla);
        GetCenter(lin);
        make_cam_err(pla,indu);
}


void make_cam_err(CAM_var *plan,CAM_Inductance *ind)
{
    uint8 i=0 , j=0 ,k=0,m=0;
    char txt[21];
    Cam_Err=0;
    float errmax,errmin,errnow;

    for(j=plan->errlinebegin;j<50;j++)
    {
        if(Exist_Center[j])
        {
            errmax=(46.50-(float)Point_Center[j] )*8;
            errmin=(46.50-(float)Point_Center[j] )*8;
            break;
        }
    }

    for(j=plan->errlinebegin,i=0;j<50;j++)
    {
        if(Exist_Center[j])
        {
            errnow=(46.50-(float)Point_Center[j] )*8;
            Cam_Err +=errnow;
            i++;
            if(fabsf(errnow)>fabsf(errmax))
            {
                errmax=errnow;
            }
            else if(fabsf(errnow)<fabsf(errmin))
            {
                errmin=errnow;
            }
        }
        if(i>5) break;
    }

    if(i>4)
    {
        i--;
        Cam_Err-=errmin/2;
        Cam_Err-=errmax/2;
    }

    for(j=13,m=0;j<58;j++)
    {
        if(imagin_cam[j][3])
        {
            m++;
        }
        if(imagin_cam[j][88])
        {
            m++;
        }
    }

    for(j=4;j<87;j++)
    {
        if(imagin_cam[14][j])
        {
            m++;
        }
        if(imagin_cam[55][j])
        {
            m++;
        }
    }


    for(k=1;k<25;k++)
    {
         if(!(imagin_cam[k][(unsigned char)(46.5+(plan->k_nihe*k)/2)+2] && imagin_cam[k][(unsigned char)(46.5-(plan->k_nihe*k)/2)-2]))
         {
             break;
         }
    }
    if((k>23 || (m>255&&cross09bit>105)) && !cam_Ring_flag)
    {
        Cam_Err=(i>0)?(Cam_Err*0.900/i):0;
   //     cam_Stop_flag=0;
        Cam_Process_Result =((int)(plan->Dst.p*0.8* Cam_Err + plan->Dst.d*0.7 * (Cam_Err - Pre_Cam_Err)))>>7;
        Cam_Process_Result =(Cam_Process_Result>=Duo_limitplus)?Duo_limitplus:(Cam_Process_Result<=-Duo_limitminus?-Duo_limitminus:Cam_Process_Result);
        ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, plan->D_middle+Cam_Process_Result, 100);
        Pre_Cam_Err = Cam_Err;

        if(cam_err_queuenum>=19)
        {
            cam_err_queuenum=0;
        }
        else
        {
            cam_err_queuenum++;
        }
        cam_err_queue[cam_err_queuenum]= abs((short)(Cam_Err*10));


        sprintf(txt,"ercams: %05.2f  ",Cam_Err);
        OLED_Write68(3,1,txt,1);
        OLED_P6x8Str(110,0,"s");
//        PIN_Write(P20_3,1);

    }
    else if(i>3 && !cam_Ring_flag)
    {
        Cam_Err=(Cam_Err/i);
   //     cam_Stop_flag=0;

        if(fabsf(Cam_Err)>plan->errcamcust)
        {
            if(Cam_Err>0.0f)
            {
                Cam_Process_Result = ((int)(plan->Dcup * (Cam_Err-plan->errcamcust)+plan->Dst.p*plan->errcamcust + plan->Dcud * (Cam_Err - Pre_Cam_Err)))>>7;
            }
            else
            {
                Cam_Process_Result = ((int)(plan->Dcup * (Cam_Err+plan->errcamcust)-plan->Dst.p*plan->errcamcust + plan->Dcud * (Cam_Err - Pre_Cam_Err)))>>7;
            }
        }
        else
        {
            Cam_Process_Result = ((int)(plan->Dst.p * Cam_Err + plan->Dst.d * (Cam_Err - Pre_Cam_Err)))>>7;
        }
        Cam_Process_Result =(Cam_Process_Result>=Duo_limitplus)?Duo_limitplus:(Cam_Process_Result<=-Duo_limitminus?-Duo_limitminus:Cam_Process_Result);
        ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, plan->D_middle+Cam_Process_Result, 100);
        Pre_Cam_Err = Cam_Err;
        if(cam_err_queuenum>=19)
        {
            cam_err_queuenum=0;
        }
        else
        {
            cam_err_queuenum++;
        }
        cam_err_queue[cam_err_queuenum]= abs((short)(Cam_Err*10));

        sprintf(txt,"ercam_: %05.2f  ",Cam_Err);
        OLED_Write68(3,1,txt,1);
        PIN_Write(P20_3,0);
        if(huandaojiansu)
        {
            PIN_Write(P20_3,1);
        }
        OLED_P6x8Str(110,0,"c");
    }
    else if(i>3 && cam_Ring_flag)
    {
        Cam_Err=(Cam_Err/i);
   //     cam_Stop_flag=0;
        if(Cam_Err>0.0f)
        {
            Cam_Process_Result = ((int)(plan->Dcup * (Cam_Err-plan->errcamcust)+plan->Dst.p*plan->errcamcust + plan->Dcud*0.5 * (Cam_Err - Pre_Cam_Err)))>>7;
        }
        else
        {
            Cam_Process_Result = ((int)(plan->Dcup * (Cam_Err+plan->errcamcust)-plan->Dst.p*plan->errcamcust + plan->Dcud*0.5 * (Cam_Err - Pre_Cam_Err)))>>7;
        }

        Cam_Process_Result =(Cam_Process_Result>=Duo_limitplus)?Duo_limitplus:(Cam_Process_Result<=-Duo_limitminus?-Duo_limitminus:Cam_Process_Result);
        ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, plan->D_middle+Cam_Process_Result, 100);
        Pre_Cam_Err = Cam_Err;
        if(cam_err_queuenum>=19)
        {
            cam_err_queuenum=0;
        }
        else
        {
            cam_err_queuenum++;
        }
        cam_err_queue[cam_err_queuenum]= abs((short)(Cam_Err*10));

        sprintf(txt,"ercam_: %05.2f  ",Cam_Err);
        OLED_Write68(3,1,txt,1);
        PIN_Write(P20_3,0);
        OLED_P6x8Str(110,0,"cr");

    }
    else
    {
        for(i=24,j=0;i>1;i--)
        {
            if(imagin_cam[i][49])
            {
                j++;
            }
            if(imagin_cam[i][47])
            {
                j++;
            }
            if(imagin_cam[i][44])
            {
                j++;
            }
        }
        for(i=30;i<66;i++)
        {
            if(imagin_cam[3][i])
            {
                j++;
            }
        }
        if(j<77 || cam_Ring_flag)
        {
            ind->lefttwice=ind->leftonce*600.00/(ind->leftonce+ind->rightonce);
            Cam_Err=(ind->lefttwice-300.0);
            if(cam_Ring_flag)
            {
                Cam_Process_Result=(plan->Icu.p *Cam_Err+plan->Icu.d*0.5*(Cam_Err - Pre_Cam_Err))/50;
            }
            else if(fabsf(Cam_Err)>plan->err_istcu )
            {
                Cam_Process_Result=(plan->Icu.p *Cam_Err+plan->Icu.d*(Cam_Err - Pre_Cam_Err))/50;
            }
            else
            {
                Cam_Process_Result=(plan->Ist.p *Cam_Err +plan->Ist.d*(Cam_Err - Pre_Cam_Err))/50;
            }
            Cam_Process_Result =(Cam_Process_Result>=Duo_limitplus)?Duo_limitplus:(Cam_Process_Result<=-Duo_limitminus?-Duo_limitminus:Cam_Process_Result);
            ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, plan->D_middle+Cam_Process_Result, 100);
            Pre_Cam_Err = Cam_Err;

            if(cam_err_queuenum>=19)
            {
                cam_err_queuenum=0;
            }
            else
            {
                cam_err_queuenum++;
            }
            cam_err_queue[cam_err_queuenum]= abs((short)(Cam_Err*10));

            sprintf(txt,"erInd : %05.2f  ",Cam_Err);
            OLED_Write68(3,1,txt,0);
//            sprintf(txt,"l: %03f %03f ",ind->levellonce,ind->levelronce);
//            OLED_P6x8Str(2,3,txt);
//            PIN_Write(P20_3,1);
        }
        else
        {
            OLED_Write68(3,1," qiguai",0);
        }
    }
}



unsigned char Ramp_judge(void)
{
    unsigned char i,j;
//    char j=1,k=1,l=1,m=1,n=1;
    for(i=0,j=1;i<30;i++)
    {
        if(!(imagin_cam[i][23] &&imagin_cam[i][47]&&imagin_cam[i][72]))
        {
            j=0;
            break;
        }
    }

    if(j==0)
    {
        for(i=0,j=1;i<30;i++)
        {
            if(!(imagin_cam[i][19] &&imagin_cam[i][42]&&imagin_cam[i][60]))
            {
                j=0;
                break;
            }
        }
    }

    if(j==0)
    {
        for(i=0,j=1;i<30;i++)
        {
            if(!(imagin_cam[i][21] &&imagin_cam[i][42]&&imagin_cam[i][63]))
            {
                j=0;
                break;
            }
        }
    }

    if(j==0)
    {
        for(i=0,j=1;i<30;i++)
        {
            if(!(imagin_cam[i][32] &&imagin_cam[i][52]&&imagin_cam[i][73]))
            {
                j=0;
                break;
            }
        }
    }
    if(j==0)
        {
            for(i=0,j=1;i<30;i++)
            {
                if(!(imagin_cam[i][35] &&imagin_cam[i][52]&&imagin_cam[i][75]))
                {
                    j=0;
                    break;
                }
            }
        }

    return j;
}

unsigned char Finish_judge(void)
{
    char i,judgenum1,judgenum;
    _Bool colorbit=1;
    cam_havefinished=0;

    for(i=18,judgenum1=0;i<76;i++)
    {
        if( imagin_cam[7][i]==colorbit)
        {
            judgenum1++;
            if(colorbit)
            {
                colorbit=0;
            }
            else
            {
                colorbit=1;
            }
        }
    }
    if(judgenum1>11)
    {
        for(i=30;i<64;i++)
        {
            if(!imagin_cam[10][i] ||!imagin_cam[12][i] ||!imagin_cam[15][i])
            {
                judgenum1=0;
                break;
            }
        }
    }


    if(judgenum1<12)
    {
        for(i=17,judgenum1=0;i<77;i++)
        {
            if( imagin_cam[8][i]==colorbit)
            {
                judgenum1++;
                if(colorbit)
                {
                    colorbit=0;
                }
                else
                {
                    colorbit=1;
                }
            }
        }
        if(judgenum1>11)
        {
            for(i=30;i<64;i++)
            {
                if(!imagin_cam[11][i] ||!imagin_cam[13][i] ||!imagin_cam[15][i])
                {
                    judgenum1=0;
                    break;
                }
            }
        }
    }

    if(judgenum1<12)
        {
            for(i=10,judgenum1=0;i<84;i++)
            {
                if( imagin_cam[9][i]==colorbit)
                {
                    judgenum1++;
                    if(colorbit)
                    {
                        colorbit=0;
                    }
                    else
                    {
                        colorbit=1;
                    }
                }
            }
            if(judgenum1>11)
            {
                for(i=30;i<64;i++)
                {
                    if(!imagin_cam[12][i] ||!imagin_cam[14][i] ||!imagin_cam[15][i])
                    {
                        judgenum1=0;
                        break;
                    }
                }
            }
        }

    if(judgenum1<12)
    {
        for(i=10,judgenum1=0;i<84;i++)
        {
            if( imagin_cam[10][i]==colorbit)
            {
                judgenum1++;
                if(colorbit)
                {
                    colorbit=0;
                }
                else
                {
                    colorbit=1;
                }
            }
        }
        if(judgenum1>11)
        {
            for(i=30;i<64;i++)
            {
                if(!imagin_cam[13][i] ||!imagin_cam[15][i]||!imagin_cam[17][i])
                {
                    judgenum1=0;
                    break;
                }
            }
        }
    }

    if(judgenum1<12)
    {
        for(i=10,judgenum1=0;i<84;i++)
        {
            if( imagin_cam[11][i]==colorbit)
            {
                judgenum1++;
                if(colorbit)
                {
                    colorbit=0;
                }
                else
                {
                    colorbit=1;
                }
            }
        }
        if(judgenum1>11)
        {
            for(i=30;i<64;i++)
            {
                if(!imagin_cam[14][i] ||!imagin_cam[16][i] ||!imagin_cam[18][i])
                {
                    judgenum1=0;
                    break;
                }
            }
        }
    }

    if(judgenum1<12)
    {
        for(i=10,judgenum1=0;i<84;i++)
        {
            if( imagin_cam[12][i]==colorbit)
            {
                judgenum1++;
                if(colorbit)
                {
                    colorbit=0;
                }
                else
                {
                    colorbit=1;
                }
            }
        }
        if(judgenum1>11)
        {
            for(i=30;i<64;i++)
            {
                if(!imagin_cam[15][i] ||!imagin_cam[17][i] ||!imagin_cam[19][i])
                {
                    judgenum1=0;
                    break;
                }
            }
        }
    }

    if(judgenum1<12)
    {
        for(i=10,judgenum1=0;i<84;i++)
        {
            if( imagin_cam[13][i]==colorbit)
            {
                judgenum1++;
                if(colorbit)
                {
                    colorbit=0;
                }
                else
                {
                    colorbit=1;
                }
            }
        }
        if(judgenum1>12)
        {
            for(i=30;i<64;i++)
            {
                if(!imagin_cam[16][i] ||!imagin_cam[18][i] ||!imagin_cam[20][i])
                {
                    judgenum1=0;
                    break;
                }
            }
        }
    }




    if(judgenum1<13)
    {
        for(i=12,judgenum=0;i<88;i++)
        {
            if( imagin_cam[11][i]==colorbit && imagin_cam[12][i]==colorbit)
            {
                judgenum++;
                if(colorbit)
                {
                    colorbit=0;
                }
                else
                {
                    colorbit=1;
                }
            }
        }
        if(judgenum<10)
           {
               for(i=12,judgenum=0;i<88;i++)
               {
                   if( imagin_cam[12][i]==colorbit && imagin_cam[13][i]==colorbit)
                   {
                       judgenum++;
                       if(colorbit)
                       {
                           colorbit=0;
                       }
                       else
                       {
                           colorbit=1;
                       }
                   }
               }
           }
           if(judgenum<10)
           {
               for(i=10,judgenum=0;i<84;i++)
               {
                   if( imagin_cam[13][i]==colorbit && imagin_cam[14][i]==colorbit)
                   {
                       judgenum++;
                       if(colorbit)
                       {
                           colorbit=0;
                       }
                       else
                       {
                           colorbit=1;
                       }
                   }
               }
           }
           if(judgenum<10)
           {
               for(i=10,judgenum=0;i<84;i++)
               {
                   if( imagin_cam[14][i]==colorbit && imagin_cam[15][i]==colorbit)
                   {
                       judgenum++;
                       if(colorbit)
                       {
                           colorbit=0;
                       }
                       else
                       {
                           colorbit=1;
                       }
                   }
               }
           }
           if(judgenum<10)
           {
               for(i=10,judgenum=0;i<84;i++)
               {
                   if( imagin_cam[15][i]==colorbit && imagin_cam[16][i]==colorbit)
                   {
                       judgenum++;
                       if(colorbit)
                       {
                           colorbit=0;
                       }
                       else
                       {
                           colorbit=1;
                       }
                   }
               }
           }
           if(judgenum<10)
           {
               for(i=10,judgenum=0;i<84;i++)
               {
                   if( imagin_cam[16][i]==colorbit && imagin_cam[17][i]==colorbit)
                   {
                       judgenum++;
                       if(colorbit)
                       {
                           colorbit=0;
                       }
                       else
                       {
                           colorbit=1;
                       }
                   }
               }
           }
    }


    if((judgenum1>11||judgenum>9) && cam_jishi-cam_finished_gar>500)
    {
        PIN_Write(P20_3,1);
        cam_finished_time++;
        cam_Three_num=1;
        cam_finished_gar=cam_jishi;


        if(cam_finished_time>1)
        {
            cam_finished_in=1;
            judgenum=planc.delay_In;
            while(judgenum>1)
            {
                if(Camera_Flag==2)
                {
                    Ind.lefttwice=Ind.leftonce*600.00/(Ind.leftonce+Ind.rightonce);
                    Cam_Err=Ind.lefttwice-300;
                    Cam_Process_Result=(planc.Ist.p *Cam_Err+planc.Ist.d*(Cam_Err - Pre_Cam_Err))/50;
                    Cam_Process_Result =(Cam_Process_Result>=Duo_limitplus)?Duo_limitplus:(Cam_Process_Result<=-Duo_limitminus?-Duo_limitminus:Cam_Process_Result);
                    ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, planc.D_middle+Cam_Process_Result, 100);
                    Pre_Cam_Err = Cam_Err;
                    if(judgenum>0) judgenum--;
                    Camera_Flag=0;
                }
            }
            cam_finished_in=0;
            cam_finish_flag=1;
            if(planc.Outga_direction)
            {
                ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, planc.D_middle+planc.angle_Ingar, 100);
            }
            else
            {
                ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, planc.D_middle-planc.angle_Ingar, 100);
            }
            cam_havefinished=1;
            cam_finish_flag=1;
            planc.bangminuspwm=6000;
            judgenum1=planc.finished_time;
            while(judgenum1>1)
            {
                if(Camera_Flag==2)
                {
                    if(planc.Outga_direction)
                    {
                        ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, planc.D_middle+planc.angle_Ingar, 100);
                    }
                    else
                    {
                        ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, planc.D_middle-planc.angle_Ingar, 100);
                    }
                    if(judgenum1>-5) judgenum1--;
                    Camera_Flag=0;
                }
            }
            cam_finish_flag=0;
            while(1)
            {
                cam_havefinished=1;
                cam_Stop_flag=1;
                cam_finish_flag=0;
                PIN_Write(P20_3,1);
                delayms(200);
                PIN_Write(P20_3,0);
                delayms(200);
                PIN_Write(P20_3,1);
                delayms(600);
                PIN_Write(P20_3,0);
                delayms(600);
                PIN_Write(P20_3,1);
                delayms(1500);
                PIN_Write(P20_3,0);
                delayms(1500);
            }

            return 1;
        }
    }
    return 0;
}

void cam_getspeed(CAM_var *pla,CAM_run *ru)
{
    uint8 i=0;
    unsigned short j=0,serrmaxp=0,serrmaxd=0;
    char txt[20];
    static unsigned short j_before;
    ru->now_speed=ENC_GetCounter(ENC5_InPut_P10_3);

    if(jishi_quanju/200>pla->stop_times)
    {
        finished_stop=1;
        cam_Stop_flag=1;
    }
    if(jishi_quanju/400<pla->stop_times)
    {
        jishi_quanju++;
    }

    if(Ind.Inleft+Ind.Inright+Ind.Inlevell+Ind.Inlevelr<pla->stop_sum && cam_jishi>300)
    {
        cam_Stop_flag=1;
    }
    if(PIN_Read(key_in_IO)==0)
        cam_Stop_flag=0;



    for(i=Iwidth-3,j=0;i>1;i--)
    {
        if(imagin_cam[i][47]||imagin_cam[i][46]||imagin_cam[i][45]||imagin_cam[i][48])
        {
            j++;
        }
        else break;
    }
    if(!cam_Stop_flag /*||cam_finish_flag*/)
    {
        if(cam_Outga || cam_jishi<70)
        {
            ru->aim_speed=pla->speedslow+10;
        }
        else if(cam_finish_flag)
        {
            ru->aim_speed=(short)(pla->speed_finished);
        }
        else if(cam_finished_in)
        {
            ru->aim_speed=(short)(pla->speed_finishin);
        }
        else if(cam_Ramp_flag)
        {
            if(onefinished==0)
            {
                ru->aim_speed=(short)(pla->speed_camp);
            }
            else
            {
                ru->aim_speed=(short)(pla->speed_camp);
            }
        }
        else if(cam_Ring_in)
        {
            ru->aim_speed=(short)(pla->Ring_speedin);
        }
//        else if(cam_Ring_flag)
//        {
//            ru->aim_speed=(short)(pla->speed_Ring+ pla->speedadd*(0.0295*j*j-0.3388*j)-pla->speedD*fabsf(0.0295*j*j-0.3388*j-0.0295*j_before*j_before+0.3388*j_before));
//        }
        else if(cam_Three_in)
        {
            ru->aim_speed=(short)(pla->Three_speedin);
        }
        else if(cam_Three_out)
        {
            ru->aim_speed=(short)(pla->Three_speedout);
        }

        else
        {
            for(i=0;i<10;i++)
            {
                if(cam_err_queuenum-i<0)
                {
                    if(cam_err_queue[cam_err_queuenum+20-i]>serrmaxp)
                    {
                        serrmaxp=cam_err_queue[cam_err_queuenum+20-i];
                    }
                    if(abs(cam_err_queue[cam_err_queuenum+20-i]-cam_err_queue[cam_err_queuenum+15-i])>serrmaxd)
                    {
                        serrmaxd=abs(cam_err_queue[cam_err_queuenum+20-i]-cam_err_queue[cam_err_queuenum+15-i]);
                    }
                }
                else
                {
                    if(cam_err_queue[cam_err_queuenum-i]>serrmaxp)
                    {
                        serrmaxp=cam_err_queue[cam_err_queuenum-i];
                    }
                    if(cam_err_queuenum-i-5<0)
                    {
                        if(abs(cam_err_queue[cam_err_queuenum-i]-cam_err_queue[cam_err_queuenum+15-i])>serrmaxd)
                        {
                            serrmaxd=abs(cam_err_queue[cam_err_queuenum-i]-cam_err_queue[cam_err_queuenum+15-i]);
                        }
                    }
                    else
                    {
                        if(abs(cam_err_queue[cam_err_queuenum-i]-cam_err_queue[cam_err_queuenum-i-5])>serrmaxd)
                        {
                            serrmaxd=abs(cam_err_queue[cam_err_queuenum-i]-cam_err_queue[cam_err_queuenum-i-5]);
                        }
                    }
                }
            }

            if(huandaojiansu )
            {
                ru->aim_speed=(short)((80+pla->speedslow/2+ pla->speedadd*(0.0295*j*j-0.3388*j)-pla->speedD*fabsf(0.0295*j*j-0.3388*j-0.0295*j_before*j_before+0.3388*j_before))
                                    -(pla->speederrp*serrmaxp+pla->speederrd*serrmaxd)/40);
            }
            else if(cam_Ring_flag)
            {
                ru->aim_speed=(short)((((pla->speed_Ring+ pla->speedadd*(0.0295*j*j-0.3388*j)-pla->speedD*fabsf(0.0295*j*j-0.3388*j-0.0295*j_before*j_before+0.3388*j_before))
                        -(pla->speederrp*serrmaxp+pla->speederrd*serrmaxd)/40)>0.85*pla->speed_Ring)?((pla->speed_Ring+ pla->speedadd*(0.0295*j*j-0.3388*j)-pla->speedD*fabsf(0.0295*j*j-0.3388*j-0.0295*j_before*j_before+0.3388*j_before))
                                -(pla->speederrp*serrmaxp+pla->speederrd*serrmaxd)/40):(0.85*pla->speed_Ring));
//                ru->aim_speed=(short)((pla->speed_Ring+ pla->speedadd*(0.0295*j*j-0.3388*j)-pla->speedD*fabsf(0.0295*j*j-0.3388*j-0.0295*j_before*j_before+0.3388*j_before))
//                                        -(pla->speederrp*serrmaxp+pla->speederrd*serrmaxd)/40);
            }
            else if(yuanhuanfinished)
            {
                ru->aim_speed=(short)(0.9*pla->speed_Ring);
            }
            else if(sanchaneibu)
            {
                ru->aim_speed=(short)((75+pla->speedslow/2+ pla->speedadd*(0.0295*j*j-0.3388*j)-pla->speedD*fabsf(0.0295*j*j-0.3388*j-0.0295*j_before*j_before+0.3388*j_before))
                        -(pla->speederrp*serrmaxp+pla->speederrd*serrmaxd)/40);
            }
            else
            {
                ru->aim_speed=(short)((pla->speedslow+ pla->speedadd*(0.0295*j*j-0.3388*j)-pla->speedD*fabsf(0.0295*j*j-0.3388*j-0.0295*j_before*j_before+0.3388*j_before))
                    -(pla->speederrp*serrmaxp+pla->speederrd*serrmaxd)/40);
            }
        }
    }
    else
    {
        ru->aim_speed=0;
    }
    if(cam_havefinished && ru->now_speed<5)
    {
        ru->give_speed_b=0;
        ru->speed_err_b=0;
        ru->speed_err_bb=0;

    }
    ru->speed_err=(ru->aim_speed-ru->now_speed);
    ru->give_speed= ((ru->speed_err * planc.Dian.i + (ru->speed_err-ru->speed_err_b)* planc.Dian.p
                    + (ru->speed_err-2*ru->speed_err_b+ru->speed_err_bb) *planc.Dian.d)>>4)+ru->give_speed_b;
    ru->give_speed=(ru->give_speed<-pla->bangminuspwm)?-pla->bangminuspwm:ru->give_speed;
    ru->give_speed=(ru->give_speed>pla->bangpluspwm)?pla->bangpluspwm:ru->give_speed;

    ru->speed_err_bb=ru->speed_err_b;
    ru->give_speed_b=ru->give_speed;
    ru->speed_err_b=ru->speed_err;

    if(ru->speed_err>pla->bangbangplus  && !cam_Ramp_flag)
    {
        ru->give_speed=pla->bangpluspwm;
        ru->give_speed=(ru->give_speed<-6000)?-6000:ru->give_speed;
        ru->give_speed=(ru->give_speed>9750)?9750:ru->give_speed;
        ru->give_speed_b=ru->give_speed;

    }
    else if(ru->speed_err<-pla->bangbangminus && !cam_Ramp_flag)
    {
        ru->give_speed=-pla->bangminuspwm;
        ru->give_speed=(ru->give_speed<-6000)?-6000:ru->give_speed;
        ru->give_speed=(ru->give_speed>9750)?9750:ru->give_speed;
        ru->give_speed_b=ru->give_speed;
    }

    ru->give_speed=(ru->give_speed<-6000)?-6000:ru->give_speed;
    ru->give_speed=(ru->give_speed>9800)?9800:ru->give_speed;




    if(finished_stop==0)
    {
        if(cam_Outga)
        {
            ATOM_PWM_SetDuty(MOTOR_Backward_IO, 0, 17000);
            delayus(3);
            ATOM_PWM_SetDuty(MOTOR_Forward_IO, 7000, 17000);
        }
        /*if(yuanhuanfinished &&!cam_Stop_flag)
        {
            ATOM_PWM_SetDuty(MOTOR_Backward_IO, 0, 17000);
            delayus(3);
            ATOM_PWM_SetDuty(MOTOR_Forward_IO, 8000, 17000);
            ru->give_speed=7000;
            ru->give_speed_b=7000;
        }*/
//        else if(cam_Ramp_flag &&!cam_Stop_flag)
//        {
//            if(onefinished==0)
//            {
//                ATOM_PWM_SetDuty(MOTOR_Backward_IO, 0, 17000);
//                delayus(3);
//                ATOM_PWM_SetDuty(MOTOR_Forward_IO, 6800, 17000);
//                ru->give_speed_b=6000;
//                ru->give_speed=6500;
//            }
//            else
//            {
//                ATOM_PWM_SetDuty(MOTOR_Backward_IO, 0, 17000);
//                delayus(3);
//                ATOM_PWM_SetDuty(MOTOR_Forward_IO, 7800, 17000);
//                ru->give_speed_b=5000;
//                ru->give_speed=6000;
//            }
//        }
        else if(ru->give_speed>=0)
        {
            ATOM_PWM_SetDuty(MOTOR_Backward_IO, 0, 17000);
            delayus(3);
            ATOM_PWM_SetDuty(MOTOR_Forward_IO, runc.give_speed, 17000);
        }
        else
        {
            ATOM_PWM_SetDuty(MOTOR_Forward_IO, 0, 17000);
            delayus(3);
            ATOM_PWM_SetDuty(MOTOR_Backward_IO, -runc.give_speed, 17000);
        }
    }
    else
    {
        ATOM_PWM_SetDuty(MOTOR_Forward_IO, 0, 17000);
        ATOM_PWM_SetDuty(MOTOR_Backward_IO, 0, 17000);
    }

    j_before=j;

//    sprintf(txt,"%d,%d,%d",ru->now_speed,ru->aim_speed,ru->give_speed);

 //   OLED_P6x8Str(5,2,txt);

    sprintf(txt,"%d,%d,%d\n",ru->now_speed,ru->aim_speed,ru->give_speed);  //TODO
    UART_PutStr(UART0,txt);                                                   //TODO
}

void cam_Camprun(CAM_var *pla,CAM_Inductance *indu)
{


        cam_Ramp_flag=1;
        PIN_Write(P20_3,1);
        camIN2pidcamp(indu,pla);
        ramp_time=30;
        while(ramp_time>0)
        {
            if(Camera_Flag == 2)
            {
                PIN_Write(P20_3,1);
                ramp_time--;
                now_info->getimagin();
                if(onefinished==0)
                {
                    camIN2pidcamp1(indu,pla);
                }
                else
                {
                    camIN2pidcamp(indu,pla);
                }
                Camera_Flag=0;
                cam_jishi++;
            }
        }
        ramp_time=pla->Ramp_delay;
        while(now_judge->judge_ramp()!=1 && ramp_time >0)
        {
            if(Camera_Flag == 2)
            {
                PIN_Write(P20_3,0);
                ramp_time--;
                now_info->getimagin();
                if(onefinished==0)
                {
                    camIN2pidcamp1(indu,pla);
                }
                else
                {
                    camIN2pidcamp(indu,pla);
                }
                Camera_Flag=0;
                cam_jishi++;
            }
        }
        cam_Ramp_flag=0;
        ramp_time=4;
        while(ramp_time>0)
        {
            if(Camera_Flag == 2)
            {
                PIN_Write(P20_3,1);
                ramp_time--;
                now_info->getimagin();
                if(onefinished==0)
                {
                    camIN2pidcamp1(indu,pla);
                }
                else
                {
                    camIN2pidcamp(indu,pla);
                }
//                camIN2pidcampout(indu,pla);
                Camera_Flag=0;
                cam_jishi++;
            }
        }
        cam_ramp_gar=cam_jishi;
        PIN_Write(P20_3,0);

        onefinished=Differto(onefinished);
        if(onefinished==0)
        {
            cam_Ring_num=0;
            cam_Three_num=Differto(cam_Three_num);
//            cam_finished_time++;
        }



}


unsigned char cam_judge_threemid(void)
{
    unsigned char i,j;

    cam_three_flag=0;

    /*    for(i=2;i<14;i++)
        {
            if(imagin_cam[13][i]||imagin_cam[34][i]|| imagin_cam[21][i]|| imagin_cam[27][i]||imagin_cam[6][i]) break;
        }
        for(i=i+1;i<14;i++)
        {
            if(imagin_cam[13][i]||imagin_cam[34][i]|| imagin_cam[21][i]|| imagin_cam[27][i]||imagin_cam[5][i]) break;
        }*/

    if(cross09bit>170) //TODO
    {
//        char txt[20];
        //     cam_three_flag=0;
//        OLED_P6x8Str(2,5," ");
//        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,5," sta110 ");
//        sprintf(txt," %d ",cross09bit);
//        OLED_P6x8Str(52,4,txt);
        return 0;
    }

    if(ThreeupLsidewhi<17) // ´ý²â
    {
        OLED_P6x8Str(10,5," sta111 ");
//        OLED_P6x8Str(2,5," ");
//        OLED_P6x8Str(2,4,".");
        return 0;
    }

    if(ThreeupRsidewhi<17)
    {
        //   cam_three_flag=0;
//        OLED_P6x8Str(2,5," ");
//        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,5," sta112 ");
        return 0;
    }

    for(i=50;i>13;i--)
    {
        if(!(imagin_cam[i][36]&&imagin_cam[i][47]&&imagin_cam[i][58]))
        {
            //        cam_three_flag=0;
//            OLED_P6x8Str(2,5," ");
//            OLED_P6x8Str(2,4,".");
            OLED_P6x8Str(10,5," sta113 ");
            return 0;
        }
    }

    for(i=2;i<14;i++)
    {
        if(imagin_cam[i][10]||imagin_cam[i][25]|| imagin_cam[i][20]|| imagin_cam[i][15]||imagin_cam[i][5]) break;
    }
    for(i=i+1;i<15;i++)
    {
        if(imagin_cam[i][10]||imagin_cam[i][25]|| imagin_cam[i][20]|| imagin_cam[i][15]||imagin_cam[i][5]) break;
    }
        if(i>12)
        {
            OLED_P6x8Str(10,5," sta114 ");
//            OLED_P6x8Str(2,4,".");
//            OLED_P6x8Str(2,5," ");
            return 0;
        }

        for(i=40,j=1;i>10;i--)
        {
            if(!imagin_cam[i][0] && !imagin_cam[i][1])
            {
                break;
            }
        }
        if(i>12)
        {
            while(j<38 && i>3)
            {
                if(!imagin_cam[i][j+1])
                {
                    j++;
                }
                else if(!imagin_cam[i-1][j])
                {
                    i--;
                }
                else break;
            }
            if(j>30)
            {
//                OLED_P6x8Str(2,5," ");
//                            OLED_P6x8Str(2,4,".");
                            OLED_P6x8Str(10,5," sta120 ");
                return 0;
            }
            if(i<6)
            {
//                OLED_P6x8Str(2,5," ");
//                            OLED_P6x8Str(2,4,".");
                            OLED_P6x8Str(10,5," sta121 ");
                return 0;
            }

            if(j<3)
            {
                if(!(imagin_cam[i-3][j] && imagin_cam[i-2][j]))
                {
//                    OLED_P6x8Str(2,5," ");
//                    OLED_P6x8Str(2,4,".");
                    OLED_P6x8Str(10,5," sta125 ");
                    return 0;
                }
            }
            else if(!(imagin_cam[i-3][j-1] && imagin_cam[i-3][j-2] && imagin_cam[i-2][j-1])) //TODO
            {
//                OLED_P6x8Str(2,5," ");
//                            OLED_P6x8Str(2,4,".");
                            OLED_P6x8Str(10,5," sta122 ");
                return 0;
            }

        }
        else
        {
//            OLED_P6x8Str(2,5," ");
//            OLED_P6x8Str(2,4,".");
            OLED_P6x8Str(10,5," sta124 ");
            return 0;
        }

        for(i=2;i<14;i++)
        {
            if(imagin_cam[i][70]||imagin_cam[i][83]|| imagin_cam[i][75]|| imagin_cam[i][62]||imagin_cam[i][90]) break;
        }
        for(i=i+1;i<14;i++)
        {
            if(imagin_cam[i][70]||imagin_cam[i][83]|| imagin_cam[i][75]|| imagin_cam[i][62]||imagin_cam[i][90]) break;
        }
                if(i>12)
                {
//                    OLED_P6x8Str(2,5," ");
//                    OLED_P6x8Str(2,4,".");
                    OLED_P6x8Str(10,5," sta115 ");
                    return 0;
                }

            for(i=40,j=92;i>10;i--)
            {
                if(!imagin_cam[i][91] && !imagin_cam[i][92])
                {
                    break;
                }
            }
            if(i>12)
            {
                while(j>55 && i>3)
                {
                    if(!imagin_cam[i][j-1])
                    {
                        j--;
                    }
                    else if(!imagin_cam[i-1][j])
                    {
                        i--;
                    }
                    else break;
                }
                if(j<63)
                {
//                    OLED_P6x8Str(2,5," ");
//                    OLED_P6x8Str(2,4,".");
                    OLED_P6x8Str(10,5," sta116 ");
                    return 0;
                }
                if(i<6)
                {
//                    OLED_P6x8Str(2,5," ");
//                                OLED_P6x8Str(2,4,".");
                                OLED_P6x8Str(10,5," sta117 ");
                    return 0;
                }

                if(j>89)
                {
                    if(!(imagin_cam[i-2][j] && imagin_cam[i-3][j]))
                    {
//                        OLED_P6x8Str(2,5," ");
//                        OLED_P6x8Str(2,4,".");
                        OLED_P6x8Str(10,5," sta126 ");
                        return 0;
                    }
                }
                else  if(!(imagin_cam[i-3][j+1] && imagin_cam[i-3][j+2] && imagin_cam[i-2][j+1]))
                {
//                    OLED_P6x8Str(2,5," ");
//                    OLED_P6x8Str(2,4,".");
                    OLED_P6x8Str(10,5," sta118 ");
                    return 0;
                }

            }
            else
            {
//                OLED_P6x8Str(2,5," ");
//                OLED_P6x8Str(2,4,".");
                OLED_P6x8Str(10,5," sta123 ");
                return 0;
            }

    for(i=46;i>40;i--)
    {
        if(!imagin_cam[i][2] || !imagin_cam[i][90])
        {
//            OLED_P6x8Str(2,5," ");
//                        OLED_P6x8Str(2,4,".");
                        OLED_P6x8Str(10,5," sta119 ");
            return 0;
        }
    }
    return 1;

}


unsigned char DFS_UPblack(void)
{
    unsigned char i=0,j=0;
    short nowp;
    char txt[20];
    uint8 mostmid=0;
    uint8 mostmii;

    for(i=33;i>12;i--)
    {
        if(!imagin_cam[i][1] && !imagin_cam[i][0])
        {
            break;
        }
    }

    mowpup=-1;
    blackpnum=0;
    memset(IsVup,0,sizeof(IsVup));
    if(!imagin_cam[i][2])
    {
        mowpup++;
        npxyup[0][mowpup] = i;
        npxyup[1][mowpup] = 2;
        blackpnum++;
    }
    if(!imagin_cam[i][1])
    {
        mowpup++;
        npxyup[0][mowpup] = i;
        npxyup[1][mowpup] = 1;
        blackpnum++;
    }
    if(!imagin_cam[i-1][1])
    {
        mowpup++;
        npxyup[0][mowpup] = i-1;
        npxyup[1][mowpup] = 1;
        blackpnum++;
    }
    if(!imagin_cam[i-1][2])
    {
        mowpup++;
        npxyup[0][mowpup] = i-1;
        npxyup[1][mowpup] = 2;
        blackpnum++;
    }
    if(blackpnum<1)
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta41 ");
        return 0;
    }

    char leftmostmid;

    while(mowpup>=0)
    {
        i = npxyup[0][mowpup];
        j = npxyup[1][mowpup];
        mowpup--;

        if((i<4)||(i>47)||(j<1)||(j>60))
        {
            continue;
        }

        if (!IsVup[i][j + 1])
        {
            IsVup[i][j + 1] = 1;
            if(!imagin_cam[i][j + 1])
            {
                mowpup++;
                npxyup[0][mowpup] = i;
                npxyup[1][mowpup] = j + 1;
                blackpnum++;
            }
            else if(imagin_cam[i][j+3] && imagin_cam[i][j+2] && (!imagin_cam[i][j]) && j<50)
            {
                Exist_Three_left[i]=1;
                Point_Three_left[i]=j;
                if(j>mostmid)
                {
                    mostmid=j;
                    mostmii=i;
                }
            }
        }

        if (!IsVup[i+1][j] && !imagin_cam[i+1][j])
        {
            IsVup[i+1][j] = 1;
            mowpup++;
            npxyup[0][mowpup] = i+1;
            npxyup[1][mowpup] = j;
            blackpnum++;
        }

        if (!IsVup[i - 1][j] && !imagin_cam[i - 1][j] && j>1)
        {
            IsVup[i - 1][j] = 1;
            mowpup++;
            npxyup[0][mowpup] = i - 1;
            npxyup[1][mowpup] = j;
            blackpnum++;
        }
    }
    if(blackpnum>450)
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta39 ");
        return 0;
    }

    if(mostmid>33)
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta40 ");
        return 0;
    }
    if(mostmii>42 || mostmii<6 )
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta43 ");
        return 0;
    }
//    sprintf(txtl,"L%d %d %d",blackpnum,mostmid,mostmii);

    leftmostmid=mostmid;


    for(i=33;i>12;i--)
    {
        if(!(imagin_cam[i][91]||imagin_cam[i][92]))
        {
            break;
        }
    }

    mostmid=92;
    mowpup=-1;
    blackpnum=0;
    memset(IsVup,0,sizeof(IsVup));
    if(!imagin_cam[i][91])
    {
        mowpup++;
        npxyup[0][mowpup] = i;
        npxyup[1][mowpup] = 91;
        blackpnum++;
    }
    if(!imagin_cam[i][90])
    {
        mowpup++;
        npxyup[0][mowpup] = i;
        npxyup[1][mowpup] = 90;
        blackpnum++;
    }
    if(!imagin_cam[i-1][91])
    {
        mowpup++;
        npxyup[0][mowpup] = i-1;
        npxyup[1][mowpup] = 91;
        blackpnum++;
    }
    if(!imagin_cam[i-1][90])
    {
        mowpup++;
        npxyup[0][mowpup] = i-1;
        npxyup[1][mowpup] = 90;
        blackpnum++;
    }
    if(blackpnum<1)
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta36 ");
        return 0;
    }


    char rightmostmid;

    while(mowpup>=0)
    {
        i = npxyup[0][mowpup];
        j = npxyup[1][mowpup];
        mowpup--;

        if((i<7)||(i>47)||(j<40)||(j>91))
        {
            continue;
        }

        if (!IsVup[i][j - 1])
        {
            IsVup[i][j - 1] = 1;
            if(!imagin_cam[i][j - 1])
            {
                mowpup++;
                npxyup[0][mowpup] = i;
                npxyup[1][mowpup] = j - 1;
                blackpnum++;
            }
            else if(imagin_cam[i][j-3] && imagin_cam[i][j-2] && (imagin_cam[i][j]==0) && j>40)
            {
                Exist_Three_left[i]=1;
                Point_Three_left[i]=j;
                if(j<mostmid)
                {
                    mostmid=j;
                    mostmii=i;
                }
            }
        }

        if (!IsVup[i+1][j] && !imagin_cam[i+1][j])
        {
            IsVup[i+1][j] = 1;
            mowpup++;
            npxyup[0][mowpup] = i+1;
            npxyup[1][mowpup] = j;
            blackpnum++;
        }

        if (!IsVup[i - 1][j] && !imagin_cam[i - 1][j] && j<93)
        {
            IsVup[i - 1][j] = 1;
            mowpup++;
            npxyup[0][mowpup] = i - 1;
            npxyup[1][mowpup] = j;
            blackpnum++;
        }
    }
    if(blackpnum>450)
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta37 ");
        return 0;
    }

    if(mostmid<60)
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta38 ");
        return 0;
    }
    if(mostmii<6 || mostmii>44)
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta42 ");
        return 0;
    }
    rightmostmid=mostmid;


    if(rightmostmid-leftmostmid<55)
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta42 ");
        return 0;
    }

//    sprintf(txtr,"R%d %d %d\n",blackpnum,mostmid,mostmii);

    memset(IsVsta1,0,sizeof(IsVsta1));
    blackpnum=0;
    nowp=-1;
    if(!imagin_cam[2][47])
    {
        blackpnum++;
        nowp++;
        npxysta1[0][nowp] = 2;
        npxysta1[1][nowp] = 47;
    }
    if(!imagin_cam[2][46])
    {
        blackpnum++;
        nowp++;
        npxysta1[0][nowp] = 2;
        npxysta1[1][nowp] = 46;
    }
    if(!imagin_cam[2][44])
    {
        blackpnum++;
        nowp++;
        npxysta1[0][nowp] = 2;
        npxysta1[1][nowp] = 44;
    }
    if(!imagin_cam[2][49])
    {
        blackpnum++;
        nowp++;
        npxysta1[0][nowp] = 2;
        npxysta1[1][nowp] = 49;
    }
    if(!imagin_cam[2][35])
    {
        blackpnum++;
        nowp++;
        npxysta1[0][nowp] = 2;
        npxysta1[1][nowp] = 35;
    }
    if(!imagin_cam[4][34])
    {
        blackpnum++;
        nowp++;
        npxysta1[0][nowp] = 4;
        npxysta1[1][nowp] = 34;
    }
    if(!imagin_cam[2][59])
    {
        blackpnum++;
        nowp++;
        npxysta1[0][nowp] = 2;
        npxysta1[1][nowp] = 59;
    }

    if(!imagin_cam[4][61])
    {
        blackpnum++;
        nowp++;
        npxysta1[0][nowp] = 4;
        npxysta1[1][nowp] = 61;
    }

    if(nowp<0)
    {
     //   cam_three_flag=0;
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta23  ");
        return 0;
    }

    memset(Exist_Three_left,0,sizeof(Exist_Three_left));
    memset(Exist_Three_right,0,sizeof(Exist_Three_right));

    unsigned char threeupmaxdowni=0;
    unsigned char threeupmaxdownj=0;



    while(nowp>=0)
    {
        i = npxysta1[0][nowp];
        j = npxysta1[1][nowp];
        nowp--;

        if((i<2)||(i>40)||(j<3)||(j>90))
        {
            continue;
        }
        if (!IsVsta1[i][j - 1])
        {
            IsVsta1[i][j - 1] = 1;
            if(!imagin_cam[i][j - 1])
            {
                nowp++;
                npxysta1[0][nowp] = i;
                npxysta1[1][nowp] = j - 1;
                blackpnum++;
            }
            else if(!imagin_cam[i][j] && imagin_cam[i][j-2] && imagin_cam[i+2][j-1] && imagin_cam[i+4][j-1]&&imagin_cam[i+1][j-1] &&imagin_cam[i+1][j-1] &&j<65 && j>10)
            {
                Exist_Three_left[i]=1;
                Point_Three_left[i]=j;
            }
        }

        if (!IsVsta1[i][j + 1])
        {
            IsVsta1[i][j + 1] = 1;
            if(!imagin_cam[i][j + 1])
            {
                nowp++;
                npxysta1[0][nowp] = i;
                npxysta1[1][nowp] = j + 1;
                blackpnum++;
            }
            else if(!imagin_cam[i][j] && imagin_cam[i][j+2]&& imagin_cam[i+2][j+1]&&imagin_cam[i+3][j+1]&& imagin_cam[i+1][j+1] && imagin_cam[i+4][j+1] && j>28 && j<84)
            {
                Exist_Three_right[i]=1;
                Point_Three_right[i]=j;
            }
        }

        if (!IsVsta1[i + 1][j] &&!imagin_cam[i + 1][j] && j>2 && j<90)
        {
                IsVsta1[i + 1][j] = 1;
                nowp++;
                npxysta1[0][nowp] = i + 1;
                npxysta1[1][nowp] = j;
                blackpnum++;
                if(threeupmaxdowni<i)
                {
                    threeupmaxdowni=i;
                    threeupmaxdownj=j;
                }

        }
        if(blackpnum>420)
        {
  //          cam_three_flag=0;
            OLED_P6x8Str(2,5," ");
            OLED_P6x8Str(2,4,".");
            OLED_P6x8Str(10,4," sta24  ");
            return 0;
        }
    }
    if(blackpnum<10)
    {
 //       cam_three_flag=0;
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta25  ");
        return 0;
    }
    for(Sum_Three_up_Yl=0,Valith=0,Sumup_Y=Sumup_X=Sumup_XX=Sumup_XY=0,i=40;i>1;i--)
    {
        if(Exist_Three_left[i])
        {
            Sumup_Y+=Point_Three_left[i];
            Valith++;
            Sumup_X+=i;
            Sumup_XX+=(float)i*i;
            Sumup_XY+=(float)i*Point_Three_left[i];
            if(Valith>4)  break;
        }
    }
    if(Valith>0)
    {
        if(Valith>1)
        {
            kk_tupl = ( Sumup_XY * Valith - Sumup_X * Sumup_Y  ) /( Sumup_XX *Valith - Sumup_X * Sumup_X );
            sprintf(txt," %05.2f    ",kk_tupl);
            OLED_P6x8Str(2,6,txt);
            if(kk_tupl<3.3)
            {
                OLED_P6x8Str(2,5," ");
                OLED_P6x8Str(2,4,".");
                OLED_P6x8Str(10,4," sta65  ");
                return 0;
            }
        }

        Sum_Three_up_Yl=Sumup_Y/Valith;
    }
    else
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta56  ");
        return 0;
    }

    for(Sum_Three_up_Yr=0,Valith=0,Sumup_Y=Sumup_X=Sumup_XX=Sumup_XY=0,i=40;i>1;i--)
    {
        if(Exist_Three_right[i])
        {
            Sumup_Y+=Point_Three_right[i];
            Valith++;
            Sumup_X+=i;
            Sumup_XX+=(float)i*i;
            Sumup_XY+=(float)i*Point_Three_right[i];
            if(Valith>4)  break;
        }
    }
    if(Valith>0)
    {
        if(Valith>1)
        {
            kk_tupr = ( Sumup_XY * Valith - Sumup_X * Sumup_Y  ) / ( Sumup_XX *Valith - Sumup_X * Sumup_X );
            printf(txt," %05.2f   ",kk_tupr);
            OLED_P6x8Str(80,6,txt);
            if(kk_tupr>-3.3)
            {
                OLED_P6x8Str(2,5," ");
                OLED_P6x8Str(2,4,".");
                OLED_P6x8Str(10,4," sta64  ");
                return 0;
            }
        }
        Sum_Three_up_Yr=Sumup_Y/Valith;
    }
    else
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta57  ");
        return 0;
    }
    if(Sum_Three_up_Yr<Sum_Three_up_Yl+15)
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta58  ");
        return 0;
    }

    if(Sum_Three_up_Yl<7)
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta59  ");
        return 0;
    }
    if(Sum_Three_up_Yr>86)
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta60  ");
        return 0;
    }
    if(Sum_Three_up_Yr<37)
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta61  ");
        return 0;
    }
    if(Sum_Three_up_Yl>57)
    {
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        OLED_P6x8Str(10,4," sta62  ");
        return 0;
    }



    OLED_P6x8Str(96,4,"sta70");
    OLED_P6x8Str(109,5," ");
//    sprintf(txt,"s70 %d %d %d",blackpnum,Sum_Three_up_Yl,Sum_Three_up_Yr);
//    UART_PutStr(UART0,txt);
//    UART_PutStr(UART0,txtl);
//    UART_PutStr(UART0,txtr);
    cam_three_flag=1;
    return 1;
}


unsigned char cam_Ringout_judge(void)
{
    unsigned char i,j;
    unsigned short white_ringnum=0;
    for(i=1;i<94;i++)
    {
        for(j=10;j<20;j++)
        {
            if(imagin_cam[j][i])
            {
                white_ringnum++;
            }
        }
    }
    if(white_ringnum>860)
    {
        return 1;
    }
    else
    {
        for(i=1,white_ringnum=0;i<94;i++)
        {
            for(j=7;j<11;j++)
            {
                if(imagin_cam[j][i])
                {
                    white_ringnum++;
                }
            }
        }
        if(white_ringnum>340)
        {
            return 1;
        }



        ;
    }
    return 0;
}

unsigned char cam_Threeinfinished(CAM_Inductance *indu)
{
    unsigned char i=0,j=0;
    short blacknumth;
    for(i=7,blacknumth=0;i<17;i++)
    {
        for(j=0;j<15;j++)
        {
            if(!imagin_cam[i][j])
            {
                blacknumth++;
            }
        }
        for(j=79;j<94;j++)
        {
            if(!imagin_cam[i][j])
            {
                blacknumth++;
            }
        }
    }
    if(blacknumth>190)
    {
        return 1;
    }
    else if(indu->rightonce+indu->leftonce<9)
    {
        return 1;
    }

    return 0;
}


unsigned char cam_Threeoutfinished(CAM_Inductance *indu)
{
    unsigned char i=0,j=0;
    short blacknumth;
    for(i=7,blacknumth=0;i<17;i++)
    {
        for(j=0;j<15;j++)
        {
            if(!imagin_cam[i][j])
            {
                blacknumth++;
            }
        }
        for(j=79;j<94;j++)
        {
            if(!imagin_cam[i][j])
            {
                blacknumth++;
            }
        }
    }
    if(blacknumth>200)
    {
        return 1;
    }
    else if(indu->levelronce+indu->levellonce>110)
    {
        return 1;
    }
    return 0;
}



void cam_Ring_run(CAM_var * pla ,CAM_Inductance *indu , Fit_line *linec)
{
    char txt[21];
    short Ring_constraint=300;
    cam_Ring_flag=1;
    char ring_insidedelay=pla->befringindelay[cam_Ring_num];
    cam_Ring_in=1;
    while(ring_insidedelay>1)
    {
        if(Camera_Flag == 2)
        {
            ring_insidedelay--;
            if(pla->Ring_direction[cam_Ring_num])
            {
                ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle+10, 100);
            }
            else
            {
                ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle-10, 100);
            }
            Camera_Flag = 0;
        }
    }
    PIN_Write(P20_3,1);

//    while(Camera_Flag!=2);
        if(pla->Ring_direction[cam_Ring_num])
        {

            ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle+pla->angle_Ring[cam_Ring_num]-50, 100);
        }
        else
        {
            ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle-pla->angle_Ring[cam_Ring_num]+50, 100);
        }

    while(indu->levellonce+indu->levelronce>pla->Ring_turnover && Ring_constraint>0)
    {
        if(Camera_Flag == 2)
        {
            sprintf(txt,"l: %03f %03f ",indu->levellonce,indu->levelronce);
            OLED_P6x8Str(2,3,txt);
  //          PIN_Write(P20_3,0);
  //        now_info->getinducrance(indu);
            if(pla->Ring_direction[cam_Ring_num])
            {
                ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle+pla->angle_Ring[cam_Ring_num], 100);
            }
            else
            {
                ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle-pla->angle_Ring[cam_Ring_num], 100);
            }
          //  cam_getspeed(pla,&runc);
            Camera_Flag = 0;
            cam_jishi++;
            Ring_constraint--;
        }
    }
        cam_Ring_in=0;
        ring_insidedelay=20;
    while( (indu->levellonce+indu->levelronce< pla->Ring_inmax ||ring_insidedelay>2)&& Ring_constraint>0)
    {
        if(Camera_Flag == 2)
        {
            sprintf(txt,"l: %03f %03f ",indu->levellonce,indu->levelronce);
            OLED_P6x8Str(2,3,txt);
            PIN_Write(P20_3,1);
  //          now_info->getinducrance(indu);
            now_info->getimagin();
            FMid_dec.part_4();
            Find_Center_Line(linec,pla);
            GetCenter(linec);
            if(cam_Ringout_judge()==1)
            {
                if(pla->Ring_direction[cam_Ring_num])
                {
                    ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle+pla->angle_Ring[cam_Ring_num]-25, 100);
                }
                else
                {
                    ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle-pla->angle_Ring[cam_Ring_num]+25, 100);
                }
            }
            else
            {
                make_cam_err(pla,indu);
            }
     //       cam_getspeed(pla,&runc);
            cam_Ring_in=0;
            Camera_Flag = 0;
            cam_jishi++;
            Ring_constraint--;
            if(ring_insidedelay>0) ring_insidedelay--;
        }
    }

    yuanhuanfinished=1;
    while(indu->levellonce+indu->levelronce< pla->Ring_over && Ring_constraint>0)
    {
        if(Camera_Flag == 2)
        {
            sprintf(txt,"l: %03f %03f ",indu->levellonce,indu->levelronce);
            OLED_P6x8Str(2,3,txt);
            PIN_Write(P20_3,0);
   //         now_info->getinducrance(indu);
            indu->lefttwice=indu->leftonce*600.00/(indu->leftonce+indu->rightonce);
            Cam_Err=indu->lefttwice-300;
            Cam_Process_Result=(pla->Ist.p *Cam_Err+pla->Ist.d*(Cam_Err - Pre_Cam_Err))/40;
            Cam_Process_Result =(Cam_Process_Result>=Duo_limitplus)?Duo_limitplus:(Cam_Process_Result<=-Duo_limitminus?-Duo_limitminus:Cam_Process_Result);
            ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle+Cam_Process_Result, 100);
            Pre_Cam_Err = Cam_Err;

            if(cam_err_queuenum>=19)
            {
                cam_err_queuenum=0;
            }
            else
            {
                cam_err_queuenum++;
            }
            cam_err_queue[cam_err_queuenum]= abs((short)(Cam_Err*10));

      //      cam_getspeed(pla,&runc);
            Camera_Flag = 0;
            cam_jishi++;
            Ring_constraint--;
        }
    }
    yuanhuanfinished=1;
    cam_Ring_num++;
    cam_Ring_flag=0;
    cam_Ringin_bit=cam_jishi;
    cam_Ringin_flag=0;
    cam_Ring_in=0;
    return;
}


void cam_Three_run(CAM_var * pla,CAM_Inductance *indu , Fit_line *linec)
{
    char delayth;
    if(pla->Three_direction[cam_Three_num]) delayth=pla->delay_threein1;
    else delayth=pla->delay_threein0;
    short delayall=pla->delay_threeall;
    if(delayall<8)
    {
        return;
    }

    char delay_threeline=50;
    cam_Three_in=1;
    while(delayth>1)
    {
        if(Camera_Flag == 2)
        {
            cam_Three_in=1;
            PIN_Write(P20_3,1);
            indu->lefttwice=indu->leftonce*600.00/(indu->leftonce+indu->rightonce);
            Cam_Err=indu->lefttwice-300;
            Cam_Process_Result=(pla->Ist.p *Cam_Err+pla->Ist.d*(Cam_Err - Pre_Cam_Err))/50;
            Cam_Process_Result =(Cam_Process_Result>=Duo_limitplus)?Duo_limitplus:(Cam_Process_Result<=-Duo_limitminus?-Duo_limitminus:Cam_Process_Result);
            ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle+Cam_Process_Result, 100);
            Pre_Cam_Err = Cam_Err;

            if(cam_err_queuenum>=19)
            {
                cam_err_queuenum=0;
            }
            else
            {
                cam_err_queuenum++;
            }
            cam_err_queue[cam_err_queuenum]= abs((short)(Cam_Err*10));

            Camera_Flag = 0;
            cam_jishi++;
            delayall--;
            delayth--;
        }
    }

    cam_Three_in=1;

    if(pla->Three_direction[cam_Three_num])
    {
        ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle+pla->angle_Threein[cam_Three_num]-40, 100);
    }
    else
    {
        ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle-pla->angle_Threein[cam_Three_num]+40, 100);
    }

    cam_jishi++;
    Camera_Flag =0;


        char delayinturn=12;
    while(( cam_Threeinfinished(indu)!=1 && delayall>0 && delayinturn>0 )|| delayinturn>8)
    {
        if(Camera_Flag == 2)
        {
            PIN_Write(P20_3,1);
            now_info->getimagin();
            if(pla->Three_direction[cam_Three_num])
            {
                ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle+pla->angle_Threein[cam_Three_num], 100);
            }
            else
            {
                ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle-pla->angle_Threein[cam_Three_num], 100);
            }
            if(delayinturn>-5) delayinturn--;
            Camera_Flag =0;
            delayall--;
            cam_jishi++;
        }
    }


    delay_threeline=80;
    while(delayall>0)
    {
        if(Camera_Flag == 2)
        {
            cam_Three_in=0;

            now_info->getimagin();
            FMid_dec.part_4();
            if(delay_threeline<30)
            {
                sanchaneibu=1;
                if(Three_dec.part_12()==1 && delay_threeline<10)
                {
                        break;
                }
            }
            Find_Center_Line(linec,pla);
            GetCenter(linec);
            make_cam_err(pla,indu);
            PIN_Write(P20_3,1);
            if(delay_threeline>-20) delay_threeline--;
            delayall--;
            Camera_Flag =0;
            cam_jishi++;
        }
    }

    cam_Three_out=1;

    if(pla->Three_direction[cam_Three_num]) delayth=pla->delay_threeout1;
    else delayth=pla->delay_threeout0;;
    sanchaneibu=0;

    while(delayth>0 && delayall>0)
    {
        if(Camera_Flag == 2)
        {
     //       PIN_Write(P20_3,0);
            indu->lefttwice=indu->leftonce*600.00/(indu->leftonce+indu->rightonce);
            Cam_Err=indu->lefttwice-300;
            Cam_Process_Result=(pla->Ist.p *Cam_Err+pla->Ist.d*(Cam_Err - Pre_Cam_Err))/50;
            Cam_Process_Result =(Cam_Process_Result>=Duo_limitplus)?Duo_limitplus:(Cam_Process_Result<=-Duo_limitminus?-Duo_limitminus:Cam_Process_Result);
            ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle+Cam_Process_Result, 100);
            Pre_Cam_Err = Cam_Err;

            if(cam_err_queuenum>=19)
            {
                cam_err_queuenum=0;
            }
            else
            {
                cam_err_queuenum++;
            }
            cam_err_queue[cam_err_queuenum]= abs((short)(Cam_Err*10));

            Camera_Flag = 0;
            delayall--;
            if(delayth>-5) delayth--;
            cam_jishi++;
        }
    }

        cam_Three_out=1;

        if(pla->Three_direction[cam_Three_num])
        {
            ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle+pla->angle_Threeout[cam_Three_num]-30, 100);
        }
        else
        {
            ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle-pla->angle_Threeout[cam_Three_num]+30, 100);
        }
        cam_Three_out=1;
        Camera_Flag = 0;



        delayinturn=15;
    while((cam_Threeoutfinished(indu)!=1 && delayall>0 && delayinturn>0 )||delayinturn>11)
    {
        if(Camera_Flag == 2)
        {
            PIN_Write(P20_3,1);
            now_info->getimagin();
            if(pla->Three_direction[cam_Three_num])
            {
                ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle+pla->angle_Threeout[cam_Three_num], 100);
            }
            else
            {
                ATOM_PWM_SetDuty(IfxGtm_ATOM0_1_TOUT31_P33_9_OUT, pla->D_middle-pla->angle_Threeout[cam_Three_num], 100);
            }
            if(delayinturn>-5) delayinturn--;
            Camera_Flag = 0;
            delayall--;
            cam_jishi++;
        }
    }
    cam_Three_quar=cam_jishi;
    PIN_Write(P20_3,0);
    cam_Three_out=0;
    return;

}



unsigned char cam_Three_judge_side(void)
{
    unsigned char i;
    cam_three_flag=0;
    cam_Three_Rightin=0;
    cam_Three_Leftin=0;

    for(i=50;i>15;i--)
    {
        if(!(imagin_cam[i][33]&&imagin_cam[i][47]&&imagin_cam[i][60]))
        {
            OLED_P6x8Str(10,5," sta01 ");
            OLED_P6x8Str(2,5,".");
            OLED_P6x8Str(2,4," ");
            return 0;
        }
    }
    if(Three_Rightin()==1) return 1;
    else if(Three_Leftin()==1) return 1;

    return 0;
}

void getdajinima(void)
{
    Get_Image_grey();
    Get_Image01(0);
}

unsigned char DFS_UPblack_side(void)
{
    char txt[20];
    unsigned char i;
    if(blackpnum>600)
    {
        OLED_P6x8Str(2,5,".");
        OLED_P6x8Str(2,4," ");
        OLED_P6x8Str(10,5," sta06");
//        sprintf(txt,"b:%d",blackpnum);
//        OLED_P6x8Str(70,5,txt);
        return 0;
    }

    if(blackpnum<17)
    {
        OLED_P6x8Str(2,5,".");
        OLED_P6x8Str(2,4," ");
        OLED_P6x8Str(10,5," sta52");
//        sprintf(txt,"b:%d",blackpnum);
//        OLED_P6x8Str(70,5,txt);
        return 0;
    }

    for(i=53;i>37;i--)
    {
        if(!imagin_cam[i][3] || !imagin_cam[i][89])
        {
            OLED_P6x8Str(2,5,".");
            OLED_P6x8Str(2,4," ");
            OLED_P6x8Str(10,5," sta07");
            return 0;
        }
    }

    for(i=20,numbup=0,Sum_Three_up_Yl=0,kk_tupl=0,Sumup_X=Sumup_XX=Sumup_XY=Sumup_Y=0;i>2;i--)
    {
        if(Exist_Three_left[i])
        {
            Sumup_X+=i;
            Sumup_XX+=i*i;
            Sumup_XY+=i*Point_Three_left[i];
            Sumup_Y+=Point_Three_left[i];
            numbup++;
        }
        if(numbup>6)
        {
            for(i=20;i>3;i--)
            {
                if(Exist_Three_left[i])
                {
                    Exist_Three_left[i]=0;
                    numbup--;
                    Sumup_X-=i;
                    Sumup_XX-=i*i;
                    Sumup_XY-=i*Point_Three_left[i];
                    Sumup_Y-=Point_Three_left[i];
                    break;
                }
            }
            break;
        }
    }

    if(numbup>0)
    {
        Sum_Three_up_Yl=Sumup_Y/numbup;
    }
    if(numbup>1)
    {
        kk_tupl= (Sumup_XY*numbup - Sumup_X * Sumup_Y  ) / (float)(Sumup_XX*numbup - Sumup_X * Sumup_X );
        sprintf(txt," %05.2f    ",kk_tupl);
        OLED_P6x8Str(2,6,txt);
    }
    else
    {
        if(numbup>0 && Sum_Three_up_Yl>28 && Sum_Three_up_Yl<47)
        {
                kk_tupl=8;
//            sprintf(txt," %05.2f    ",kk_tupl);
//            OLED_P6x8Str(2,6,txt);
        }
        else
        {
            OLED_P6x8Str(22,6," 0 ");
        }
    }
    numbup=0;
    Sum_Three_up_Yr=0;
    kk_tupr=Sumup_X=Sumup_XX=Sumup_XY=Sumup_Y=0;
    for(i = 20 ; i>2;i--)
    {
        if(Exist_Three_right[i])
        {
            Sumup_X+=i;
            Sumup_XX+=i*i;
            Sumup_XY+=i*Point_Three_right[i];
            Sumup_Y+=Point_Three_right[i];
            numbup++;
        }
        if(numbup>6)
        {
            for(i=20;i>3;i--)
            {
                if(Exist_Three_right[i])
                {
                    Exist_Three_right[i]=0;
                    numbup--;
                    numbup--;
                    Sumup_X-=i;
                    Sumup_XX-=i*i;
                    Sumup_XY-=i*Point_Three_right[i];
                    Sumup_Y-=Point_Three_right[i];
                    break;
                }
            }
            break;
        }
    }
    if(numbup>0)
    {
        Sum_Three_up_Yr=Sumup_Y/numbup;
    }
    if(numbup>1)
    {
        kk_tupr = ( Sumup_XY * numbup - Sumup_X * Sumup_Y  ) / (float)( Sumup_XX *numbup - Sumup_X * Sumup_X );
        sprintf(txt," %05.2f   ",kk_tupr);
        OLED_P6x8Str(80,6,txt);
    }
    else
    {

        if(Sum_Three_up_Yr<65 && Sum_Three_up_Yr>47 && numbup>0)
        {
                kk_tupr=-8;
//            sprintf(txt," %05.2f   ",kk_tupr);
//            OLED_P6x8Str(80,6,txt);
        }
        else
        {
            OLED_P6x8Str(74,6," 0 ");
        }
    }

    if(Sum_Three_up_Yr<Sum_Three_up_Yl)
    {
        OLED_P6x8Str(20,5," sta08  ");
        OLED_P6x8Str(2,5,".");
        OLED_P6x8Str(2,4," ");
        return 0;
    }

    if(kk_tupl<3 || kk_tupr>-3)
    {
        OLED_P6x8Str(20,5," sta09 ");
        OLED_P6x8Str(2,5,".");
        OLED_P6x8Str(2,4," ");
        return 0;
    }
    return 1;
}


unsigned char threeDFSside(void)
{
    unsigned char i,j;
    char txt[20];
    nowpth=-1;
    unsigned char mosmid;
    unsigned char mosmii;

    memset(IsVth,0,sizeof(IsVth));
    for(i=0;i<60;i++)
    {
        Exist_side[i]=0;
    }

    if(cam_Three_Leftin==1)
    {
        if(!imagin_cam[16][1] ||!imagin_cam[18][1] ||!imagin_cam[20][1] || !imagin_cam[22][1])
        {
            if(!imagin_cam[16][1])
            {
                nowpth++;
                npxyth[0][nowpth] = 16;
                npxyth[1][nowpth] = 1;
                IsVth[16][1] = 1;
            }
            else if(!imagin_cam[18][1])
            {
                nowpth++;
                npxyth[0][nowpth] = 18;
                npxyth[1][nowpth] = 1;
                IsVth[18][1] = 1;
            }
            else if(!imagin_cam[20][1])
            {
                nowpth++;
                npxyth[0][nowpth] = 20;
                npxyth[1][nowpth] = 1;
                IsVth[20][1] = 1;
            }
            else if(!imagin_cam[22][1])
            {
                nowpth++;
                npxyth[0][nowpth] = 22;
                npxyth[1][nowpth] = 1;
                IsVth[22][1] = 1;
            }
            if(nowpth<0)
            {
                OLED_P6x8Str(45,5,".");
                                            OLED_P6x8Str(2,4," ");
                                            OLED_P6x8Str(50,5,"sta10");
                return 0;
            }

            mosmid=0;
            while(nowpth>=0)
            {
                i=npxyth[0][nowpth];
                j=npxyth[1][nowpth];
                nowpth--;

                if(i<8 || i>50 || j<1 || j>50)
                {
                    continue;
                }

                if(!IsVth[i+1][j] && !imagin_cam[i+1][j])
                {
                    IsVth[i+1][j]=1;
                    nowpth++;
                    npxyth[0][nowpth]=i+1;
                    npxyth[1][nowpth]=j;

                }
                if(!IsVth[i-1][j] && !imagin_cam[i-1][j])
                {
                    IsVth[i-1][j]=1;
                    nowpth++;
                    npxyth[0][nowpth]=i-1;
                    npxyth[1][nowpth]=j;

                }
                if(!IsVth[i][j+1])
                {
                    IsVth[i][j+1]=1;
                    if(!imagin_cam[i][j+1])
                    {
                        nowpth++;
                        npxyth[0][nowpth]=i;
                        npxyth[1][nowpth]=j+1;
                    }
                    else if(!imagin_cam[i][j] && imagin_cam[i][j+2])
                    {
                        sidepoint[i]=j+1;
                        Exist_side[i]=1;
                        if(j+1>mosmid)
                        {
                            mosmid=j+1;
                            mosmii=i;
                        }
                    }
                }
            }

            for(i=40,Valith=0,Sumth_XY=Sumth_X=Sumth_Y=Sumth_XX=0;i>10;i--)
            {
                if(Exist_side[i])
                {
                    Sumth_XX+=i*i;
                    Sumth_X+=i;
                    Sumth_Y+=sidepoint[i];
                    Sumth_XY+=i*sidepoint[i];
                    Valith++;
                }
                if(Valith>6)
                {
                    for(i=40;i>15;i--)
                    {
                        if(Exist_side[i])
                        {
                            Exist_side[i]=0;
                            Valith--;
                            Sumth_XX-=i*i;
                            Sumth_X-=i;
                            Sumth_Y-=sidepoint[i];
                            Sumth_XY-=i*sidepoint[i];
                            break;
                        }
                    }
                    break;
                }
            }

            if(mosmid>29)
            {
                OLED_P6x8Str(45,5,".");
                                            OLED_P6x8Str(2,4," ");
                                            OLED_P6x8Str(50,5,"sta44");
                return 0;
            }
            if(mosmii<10)
            {
                OLED_P6x8Str(45,5,".");
                                            OLED_P6x8Str(2,4," ");
                                            OLED_P6x8Str(50,5,"sta45");
                return 0;
            }

            if(Valith>1)
            {
                kk_tdown=(Sumth_XY*Valith - Sumth_X * Sumth_Y) / (float)(Sumth_XX*Valith - Sumth_X * Sumth_X );
                sprintf(txt," dl:%05.2f",kk_tdown);
                OLED_P6x8Str(10,7,txt);
                if(fabsf(kk_tdown-kk_tupr)<4.3)
                {
                 //   cam_three_flag=0;
                    OLED_P6x8Str(45,5,".");
                    OLED_P6x8Str(2,4," ");
                    OLED_P6x8Str(50,5,"sta11");
                    return 0;
                }
                else
                {
                    cam_three_flag=1;
                    OLED_P6x8Str(10,5,"              sta72 ");
                    OLED_P6x8Str(108,4," ");

               //     UART_PutStr(UART0,"sta72");
                    return 1;
                }
            }
            else
            {
          //      OLED_P6x8Str(10,7,"           state4 ");
                if(kk_tupr<-6 && kk_tupl>6)
                {
               //     cam_three_flag=1;
                    OLED_P6x8Str(10,5,"              sta73  ");
               //     UART_PutStr(UART0,"sta73");
                    return 1;
                }
                else
                {
                    OLED_P6x8Str(45,5,".");
                                                OLED_P6x8Str(2,4," ");
                                                OLED_P6x8Str(50,5,"sta12");
                //    cam_three_flag=0;
                    return 0;
                }
            }
        }
    }

    if(cam_Three_Rightin==1)
    {
        if(!imagin_cam[16][92] || !imagin_cam[18][92] || !imagin_cam[20][92] ||!imagin_cam[22][92])
        {
            if(!imagin_cam[16][92])
            {
                nowpth++;
                npxyth[0][nowpth] = 16;
                npxyth[1][nowpth] = 92;
                IsVth[16][92] = 1;
            }
            else if(!imagin_cam[18][92])
            {
                nowpth++;
                npxyth[0][nowpth] = 18;
                npxyth[1][nowpth] = 92;
                IsVth[18][92] = 1;
            }
            else if(!imagin_cam[20][92])
            {
                nowpth++;
                npxyth[0][nowpth] = 20;
                npxyth[1][nowpth] = 92;
                IsVth[20][92] = 1;
            }
            else if(!imagin_cam[22][92])
            {
                nowpth++;
                npxyth[0][nowpth] = 22;
                npxyth[1][nowpth] = 92;
                IsVth[22][92] = 1;
            }
            mosmid=92;
            if(nowpth<0)
            {
                OLED_P6x8Str(2,5,".");
                                    OLED_P6x8Str(2,4," ");
                                    OLED_P6x8Str(8,5,"sta135");
            }
            while(nowpth>=0)
            {
                i=npxyth[0][nowpth];
                j=npxyth[1][nowpth];
                nowpth--;
                if(i<8 || i>50 || j<50 || j>92)
                {
                    continue;
                }

                if(!IsVth[i+1][j] && !imagin_cam[i+1][j])
                {
                    IsVth[i+1][j]=1;
                    nowpth++;
                    npxyth[0][nowpth]=i+1;
                    npxyth[1][nowpth]=j;
                }
                if(!IsVth[i-1][j]  && !imagin_cam[i-1][j])
                {
                    IsVth[i-1][j]=1;
                    nowpth++;
                    npxyth[0][nowpth]=i-1;
                    npxyth[1][nowpth]=j;
                }
                if(!IsVth[i][j-1])
                {
                    IsVth[i][j-1]=1;
                    if(!imagin_cam[i][j-1])
                    {
                        nowpth++;
                        npxyth[0][nowpth]=i;
                        npxyth[1][nowpth]=j-1;
                    }
                    else if(!imagin_cam[i][j] && imagin_cam[i][j-2])
                    {
                        sidepoint[i]=j-1;
                        Exist_side[i]=1;
                        if(j-1<mosmid)
                        {
                            mosmid=j-1;
                            mosmii=i;
                        }
                    }
                }
            }
            for(i=40,Valith=0,Sumth_XY=Sumth_X=Sumth_Y=Sumth_XX=0;i>10;i--)
            {
                if(Exist_side[i])
                {
                    Sumth_XX+=i*i;
                    Sumth_X+=i;
                    Sumth_Y+=sidepoint[i];
                    Sumth_XY+=i*sidepoint[i];
                    Valith++;
                }
                if(Valith>6)
                {
                    for(i=40;i>15;i--)
                    {
                        if(Exist_side[i])
                        {
                            Exist_side[i]=0;
                            Valith--;
                            Sumth_XX-=i*i;
                            Sumth_X-=i;
                            Sumth_Y-=sidepoint[i];
                            Sumth_XY-=i*sidepoint[i];
                            break;
                        }
                    }
                    break;
                }
            }
            if(mosmid<64)
            {
                OLED_P6x8Str(2,5,".");
                                    OLED_P6x8Str(2,4," ");
                                    OLED_P6x8Str(8,5,"sta46");
                return 0;
            }
            if(mosmii<10)
            {
                OLED_P6x8Str(2,5,".");
                OLED_P6x8Str(2,4," ");
                OLED_P6x8Str(8,5,"sta47");
                return 0;
            }

            if(Valith>1)
            {
                kk_tdown=(Sumth_XY*Valith - Sumth_X * Sumth_Y ) / (float)(Sumth_XX*Valith - Sumth_X * Sumth_X);
                sprintf(txt," dr:%05.2f  ",kk_tdown);
                OLED_P6x8Str(5,7,txt);
                if(fabsf(kk_tdown-kk_tupl)<4.3)
                {
                    OLED_P6x8Str(2,5,".");
                                    OLED_P6x8Str(2,4," ");
                                    OLED_P6x8Str(8,5,"sta13");
               //     cam_three_flag=0;
                    return 0;
                }
                else
                {
                    cam_three_flag=1;

                    OLED_P6x8Str(10,5,"         sta74 ");
                    sprintf(txt,"s74 %03f %03f %03f",kk_tupl,kk_tupr,kk_tdown);
              //      UART_PutStr(UART0,txt);
                    return 1;
                }
            }
            else
            {
            //    OLED_P6x8Str(10,7,"          state4 ");
                if(kk_tupr<-6 && kk_tupl>6 )
                {
                //    cam_three_flag=1;
                    OLED_P6x8Str(10,5,"         sta75  ");
//                    UART_PutStr(UART0,"sta75");
                    return 1;
                }
                else
                {
                    OLED_P6x8Str(2,5,".");
                                    OLED_P6x8Str(2,4," ");
                                    OLED_P6x8Str(8,5,"sta14");
                //    cam_three_flag=0;
                    return 0;
                }
            }
        }
    }
    OLED_P6x8Str(2,5,".");
                    OLED_P6x8Str(2,4," ");
                    OLED_P6x8Str(8,5,"sta15");
        return 0;

}


unsigned char cam_three(void)
{
    char txt[20];
    cam_three_flag=0;
    if(cross09bit<125)
    {
        if(Three_dec.part_1()==1 )
        {
            return Three_dec.part_2();
        }
        else if(Three_dec.part_3()==1)
        {
            Three_dec.part_4();
            Three_dec.part_13();
            Three_dec.part_6();
            Three_dec.part_7();
            if(Three_dec.part_8()==1)
            {
               return Three_dec.part_9();
            }
        }
    }
    else
    {
        OLED_P6x8Str(2,5,".");
        OLED_P6x8Str(2,4," ");
        sprintf(txt,"B%d",cross09bit);
        OLED_P6x8Str(10,5," sta61 ");
        OLED_Write68(55,5,txt,0);
    }

    return 0;
}




unsigned char cam_Three_judge_outl(void) //TODO
{
    unsigned char i,j;
    cam_three_flag=0;

    for(i=50;i>16;i--)
    {
        if(!(imagin_cam[i][36]&&imagin_cam[i][47]&&imagin_cam[i][61]))
        {
            OLED_P6x8Str(5,4,"sta216 ");
            return 0;
        }
    }

    for(i=2;i<14;i++)
    {
        if(imagin_cam[i][10]||imagin_cam[i][25]|| imagin_cam[i][20]|| imagin_cam[i][15]||imagin_cam[i][5]) break;
    }
    for(i=i+1;i<15;i++)
    {
        if(imagin_cam[i][10]||imagin_cam[i][25]|| imagin_cam[i][20]|| imagin_cam[i][15]||imagin_cam[i][5]) break;
    }
    if(i>12)
    {
        OLED_P6x8Str(5,4,"sta217 ");
        return 0;
    }

    if(ThreeupLsidewhi<20) //TODO ´ý²â
    {
        OLED_P6x8Str(5,4,"sta218 ");
        return 0;
    }


    for(i=40,j=1;i>10;i--)
    {
        if(!imagin_cam[i][0] && !imagin_cam[i][1])
        {
            break;
        }
    }
    if(i>12)
    {
        while(j<38 && i>2)
        {
            if(!imagin_cam[i][j+1])
            {
                j++;
            }
            else if(!imagin_cam[i-1][j])
            {
                i--;
            }
            else break;
        }
        if(j>34)
        {
            OLED_P6x8Str(5,4,"sta218 ");
            return 0;
        }
        if(i<6)
        {
            OLED_P6x8Str(5,4,"sta219 ");
            return 0;
        }
        if(j<3)
        {
            if(!(imagin_cam[i-3][j] && imagin_cam[i-2][j]))
            {
                OLED_P6x8Str(5,4,"sta220 ");
                return 0;
            }
        }
        else if(!(imagin_cam[i-3][j-1] && imagin_cam[i-3][j-2] && imagin_cam[i-2][j-1])) //TODO
        {
            OLED_P6x8Str(5,4,"sta221 ");
            return 0;
        }
        if(Threeleftinupr>30)
        {
            OLED_P6x8Str(5,4,"sta222 ");
            return 0;
        }
        return DFS_UPblack_out(); //TODO
    }

    return 0;
}


unsigned char cam_Three_judge_outr(void)
{
    unsigned char i,j;
    cam_three_flag=0;

    for(i=50;i>16;i--)
    {
        if(!(imagin_cam[i][36]&&imagin_cam[i][47]&&imagin_cam[i][61]))
        {
            OLED_P6x8Str(50,4,"sta235");
            return 0;
        }
    }

    for(i=2;i<14;i++)
    {
        if(imagin_cam[i][70]||imagin_cam[i][83]|| imagin_cam[i][75]|| imagin_cam[i][62]||imagin_cam[i][90]) break;
    }
    for(i=i+1;i<14;i++)
    {
        if(imagin_cam[i][70]||imagin_cam[i][83]|| imagin_cam[i][75]|| imagin_cam[i][62]||imagin_cam[i][90]) break;
    }
    if(i>12)
    {
        OLED_P6x8Str(50,4,"sta236");
        return 0;
    }

    if(ThreeupRsidewhi<20)
    {
        OLED_P6x8Str(50,4,"sta237");
        return 0;
    }
    for(i=2;i<14;i++)
    {
        if(imagin_cam[i][70]||imagin_cam[i][83]|| imagin_cam[i][75]|| imagin_cam[i][62]||imagin_cam[i][90]) break;
    }
    for(i=i+1;i<14;i++)
    {
        if(imagin_cam[i][70]||imagin_cam[i][83]|| imagin_cam[i][75]|| imagin_cam[i][62]||imagin_cam[i][90]) break;
    }

    if(i>12)
    {
        OLED_P6x8Str(50,4,"sta238");
        return 0;
    }

    for(i=40,j=92;i>10;i--)
    {
        if(!imagin_cam[i][91] && !imagin_cam[i][92])
        {
            break;
        }
    }
    if(i>12)
    {
        while(j>55 && i>3)
        {
            if(!imagin_cam[i][j-1])
            {
                j--;
            }
            else if(!imagin_cam[i-1][j])
            {
                i--;
            }
            else break;
        }
        if(j<58)
        {
            OLED_P6x8Str(50,4,"sta239");
            return 0;
        }
        if(i<6)
        {
            OLED_P6x8Str(50,4,"sta240");
            return 0;
        }
        if(!(imagin_cam[i-3][j+1] && imagin_cam[i-3][j+2] && imagin_cam[i-2][j+1])) //TODO
        {
            OLED_P6x8Str(50,4,"sta241");
            return 0;
        }

        if(Threerightinupl>34)
        {
            OLED_P6x8Str(50,4,"sta242");
            return 0;
        }
        return DFS_UPblack_out();
    }
    OLED_P6x8Str(50,4,"sta243");
    return 0;
}


unsigned char DFS_UPblack_out(void)
{
 //   char txt[20];
    unsigned char i=0,j=0;
    char maxbl=0;
    char maxblj=0;
    memset(IsVup,0,sizeof(IsVup));
    blackpnum=0;
    mowpup=-1;

    memset(Exist_Three_left,0,sizeof(Exist_Three_left));
    memset(Exist_Three_right,0,sizeof(Exist_Three_right));

    if(!imagin_cam[2][47])
    {
        blackpnum++;
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 47;
        IsVup[2][47] = 1;
    }
    if(!imagin_cam[2][46])
    {
        blackpnum++;
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 46;
    }
    if(!imagin_cam[2][44])
    {
        blackpnum++;
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 44;

    }
    if(!imagin_cam[2][49])
    {
        blackpnum++;
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 49;
    }
    if(!imagin_cam[2][30])
    {
        blackpnum++;
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 30;

    }
    if(!imagin_cam[2][37])
    {
        blackpnum++;
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 37;
    }
    if(!imagin_cam[2][63])
    {
        blackpnum++;
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 63;
    }
    if(!imagin_cam[2][54])
    {
        blackpnum++;
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 54;
    }
    if(mowpup<0)
    {
        OLED_P6x8Str(6,6,"sta252");
        return 0;
    }
    kk_tupl=0;
    kk_tupr=0;

    while(mowpup>=0)
    {
        i = npxyup[0][mowpup];
        j = npxyup[1][mowpup];
        mowpup--;

        if((i<2)||(i>48)||(j<1)||(j>89))
        {
            continue;
        }
        if (!IsVup[i][j - 1])
        {
            if(!imagin_cam[i][j - 1])
            {
                mowpup++;
                IsVup[i][j - 1] = 1;
                npxyup[0][mowpup] = i;
                npxyup[1][mowpup] = j - 1;
                blackpnum++;
            }
            else
            {
                if(imagin_cam[i][j-3] && imagin_cam[i][j-2] && !imagin_cam[i][j]&&(imagin_cam[i+1][j-1]||imagin_cam[i-1][j-1]) &&imagin_cam[i+4][j-1]&& imagin_cam[i+2][j-1] && j<64)
                {
                    IsVup[i][j - 1] = 1;
                    Exist_Three_left[i]=1;
                    Point_Three_left[i]=j;
                }
            }
        }

        if (!IsVup[i][j + 1])
        {
            if(!imagin_cam[i][j + 1])
            {
                mowpup++;
                IsVup[i][j + 1] = 1;
                npxyup[0][mowpup] = i;
                npxyup[1][mowpup] = j + 1;
                blackpnum++;
            }
            else
            {
                if(imagin_cam[i][j+2] && !imagin_cam[i][j] &&imagin_cam[i][j+3] && (imagin_cam[i+1][j+1]||imagin_cam[i-1][j+1])&& imagin_cam[i+4][j+1]&& imagin_cam[i+2][j+1]  && j>30)
                {
                    IsVup[i][j + 1] = 1;
                    Exist_Three_right[i]=1;
                    Point_Three_right[i]=j;
                }
            }
        }

        if (!IsVup[i + 1][j] && !imagin_cam[i + 1][j])
        {
            IsVup[i + 1][j] = 1;
            mowpup++;
            npxyup[0][mowpup] = i + 1;
            npxyup[1][mowpup] = j;
            blackpnum++;
            if(i+1>maxbl)
            {
                maxbl=i+1;
                maxblj=j;
            }

        }
        if(maxbl>30)
        {
            OLED_P6x8Str(6,6,"sta223");
            return 0;
        }

    }

    if(blackpnum<10)
    {
        OLED_P6x8Str(6,6,"sta224");
        return 0;
    }
    for(i=30,numbup=0,kk_tupl=0,Sumup_X=Sumup_XX=Sumup_XY=Sumup_Y=0;i>2;i--)
    {
        if(Exist_Three_left[i])
        {
            Sumup_X+=i;
            Sumup_XX+=i*i;
            Sumup_XY+=i*Point_Three_left[i];
            Sumup_Y+=Point_Three_left[i];
            numbup++;
        }
    }
    if(numbup>1)
    {
        kk_tupl= (Sumup_XY*numbup - Sumup_X * Sumup_Y  ) /(Sumup_XX*numbup - Sumup_X * Sumup_X );
    }
    else if(numbup>0 && Sumup_Y>25 && Sumup_Y<50)
    {
       kk_tupl=2.2;
    }
    for(i=30,numbup=0,kk_tupr=0,Sumup_X=Sumup_XX=Sumup_XY=Sumup_Y=0;i>2;i--)
    {
        if(Exist_Three_right[i])
        {
            Sumup_X+=i;
            Sumup_XX+=i*i;
            Sumup_XY+=i*Point_Three_right[i];
            Sumup_Y+=Point_Three_right[i];
            numbup++;
        }
    }
    if(numbup>1)
    {
        kk_tupr = ( Sumup_XY * numbup - Sumup_X * Sumup_Y  ) /( Sumup_XX *numbup - Sumup_X * Sumup_X );
    }
    else if(numbup>0 && Sumup_Y<72 && Sumup_Y>42)
    {
       kk_tupr=-2.2;
    }

 //   sprintf(txt,"l:%05.2f  r:%05.2f ",kk_tupl,kk_tupr);

 //   OLED_P6x8Str(5,6,txt);

    if(blackpnum>540)
    {
        OLED_P6x8Str(6,6,"sta225");
        return 0;
    }
    if(kk_tupl>2 && kk_tupr<-2)
    {
        return 1;
    }
    else if(kk_tupl>2.4  && maxbl>1 && maxbl <14 && maxblj<86 && maxblj>3  )
    {
//        for(numbup=0,i=40;i>7;i--)
//        {
//            if(imagin_cam[i][2] ||imagin_cam[i][3])
//            {
//                numbup++;
//            }
//        }
//        if(numbup>8)
//        {
//            for(i=30;i>10;i--)
//            {
//                if(imagin_cam[i][87] && imagin_cam[i][47] && imagin_cam[i][42])
//                {
//                    break;
//                }
//            }
//            if(i>12)
//            {
//                if(maxblj<86 && maxblj>8) return 1;
//            }
//        }
        return 1;
    }
    else if(kk_tupr<-2.4  && maxbl>1 && maxbl <14  && maxblj<86 && maxblj>3 )
    {
//        for(numbup=0,i=40;i>7;i--)
//        {
//            if(imagin_cam[i][90] || imagin_cam[i][89])
//            {
//                numbup++;
//            }
//        }
//        if(numbup>8)
//        {
//            for(i=30;i>10;i--)
//            {
//                if(imagin_cam[i][52] && imagin_cam[i][47] && imagin_cam[i][5])
//                {
//                    break;
//                }
//            }
//            if(i>12)
//            {
//                if(maxblj<86 && maxblj>8) return 1;
//            }
//        }
        return 1;
    }
    OLED_P6x8Str(6,6,"sta226");
    return 0;
}


//void cam_Three_out_fall(void)
//{
//    uint8 i,j;
//    for(i=30;i>13;i--)
//    {
//        if(!imagin_cam[i][1])
//        {
//            return;
//        }
//    }
//}

void DFS_UPblack_side_step1(void)
{
    mowpup=-1;
    blackpnum=0;
    if(!imagin_cam[2][46])
    {
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 46;
        IsVup[2][46] = 1;
        blackpnum++;
    }

    if(!imagin_cam[2][44])
    {
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 44;
        IsVup[2][44] = 1;
        blackpnum++;
    }

    if(!imagin_cam[2][49])
    {
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 49;
        IsVup[2][49] = 1;
    }

    if(!imagin_cam[2][36])
    {
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] =36;
        IsVup[2][36] = 1;
        blackpnum++;
    }

    if(!imagin_cam[2][33])
    {
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 33;
        IsVup[2][33] = 1;
        blackpnum++;
    }

    if(!imagin_cam[2][62])
    {
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 62;
        IsVup[2][62] = 1;
        blackpnum++;
    }

    if(!imagin_cam[2][60])
    {
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 83;
        IsVup[2][60] = 1;
        blackpnum++;
    }

    if(!imagin_cam[2][47])
    {
        mowpup++;
        npxyup[0][mowpup] = 2;
        npxyup[1][mowpup] = 47;
        IsVup[2][47] = 1;
        blackpnum++;
    }
    return ;

}

void ISVUP_init(void)
{
    unsigned char i=0;//j=0;
    memset(IsVup,0,sizeof(IsVup));
    for(i=0;i<50;i++)
    {
        Exist_Three_left[i]=0;
        Exist_Three_right[i]=0;
    }
    return;
}

void DFS_UPblack_side_step2(void)
{
    unsigned char i=0,j=0;

    while(mowpup>=0)
    {
        i = npxyup[0][mowpup];
        j = npxyup[1][mowpup];
        mowpup--;

        if((i<2)||(i>47)||(j<2)||(j>91))
        {
            continue;
        }

        if (!IsVup[i][j - 1])
        {
            IsVup[i][j - 1] = 1;
            if(!imagin_cam[i][j - 1])
            {
                mowpup++;
                npxyup[0][mowpup] = i;
                npxyup[1][mowpup] = j - 1;
                blackpnum++;
            }
            else if(imagin_cam[i][j-3] && imagin_cam[i][j-2] && (!imagin_cam[i][j])&&imagin_cam[i+1][j-1]&&imagin_cam[i+3][j-1]&& imagin_cam[i+2][j-1]&&imagin_cam[i+4][j-1] && j<60)
            {
                Exist_Three_left[i]=1;
                Point_Three_left[i]=j-1;
            }
        }

        if (!IsVup[i][j + 1])
        {
            IsVup[i][j + 1] = 1;

            if(!imagin_cam[i][j + 1])
            {
                mowpup++;
                npxyup[0][mowpup] = i;
                npxyup[1][mowpup] = j + 1;
                blackpnum++;
            }
            else if(imagin_cam[i][j+2] && (!imagin_cam[i][j]) && imagin_cam[i][j+3] &&(imagin_cam[i+1][j+1]&&imagin_cam[i+3][j+1])&& imagin_cam[i+2][j+1]&&imagin_cam[i+4][j+1]  && j>30)
            {
                Exist_Three_right[i]=1;
                Point_Three_right[i]=j+1;
            }

        }

        if (!IsVup[i + 1][j] && !imagin_cam[i + 1][j])
        {
            IsVup[i + 1][j] = 1;
            mowpup++;
            npxyup[0][mowpup] = i + 1;
            npxyup[1][mowpup] = j;
            blackpnum++;
        }
    }
    return;
}

unsigned char Three_outjudge(void)
{
    if(cam_Three_judge_outl()==1 || cam_Three_judge_outr()==1)
    {
        return 1;
    }
    else if(cam_judge_threemid()==1)
    {
       return DFS_UPblack_out();
    }
    return 0;
}

void  DFS_UPblack_side_step3(void)
{
    unsigned char i;
    for(i=2;i<30;i++)
    {
        if(Exist_Three_left[i] && Exist_Three_right[i])
        {
            if(Point_Three_left[i]>Point_Three_right[i])
            {
                Exist_Three_left[i]=0;
                Exist_Three_right[i]=0;
            }
        }
    }
    return;
}


unsigned char Three_Leftin(void)
{
    unsigned char i,j,k;


    if(ThreeupLsidewhi<27) //TODO ´ý²â
    {
        OLED_P6x8Str(45,5,".");
                OLED_P6x8Str(2,4," ");
                OLED_P6x8Str(50,5,"sta135");
        return 0;
    }

    for(i=2;i<14;i++)
    {
        if(imagin_cam[i][10]||imagin_cam[i][25]|| imagin_cam[i][20]|| imagin_cam[i][15]||imagin_cam[i][5]) break;
    }
    for(i=i+1;i<15;i++)
    {
        if(imagin_cam[i][10]||imagin_cam[i][25]|| imagin_cam[i][20]|| imagin_cam[i][15]||imagin_cam[i][5]) break;
    }
    if(i>12)
    {
        OLED_P6x8Str(45,5,".");
        OLED_P6x8Str(2,4," ");
        OLED_P6x8Str(50,5,"sta136");
        return 0;
    }

    for(i=40,j=1;i>10;i--)
    {
        if(!imagin_cam[i][0] && !imagin_cam[i][1])
        {
            break;
        }
    }
    if(i>12)
    {
        while(j<38 && i>3)
        {
            if(!imagin_cam[i][j+1])
            {
                j++;
            }
            else if(!imagin_cam[i-1][j])
            {
                i--;
            }
            else break;
        }
        if(j>33)
        {
            OLED_P6x8Str(45,5,".");
            OLED_P6x8Str(2,4," ");
            OLED_P6x8Str(50,5,"sta137");
            return 0;
        }
        if(i<6)
        {
            OLED_P6x8Str(45,5,".");
            OLED_P6x8Str(2,4," ");
            OLED_P6x8Str(50,5,"sta138");
            return 0;
        }


        if(!(imagin_cam[i-3][j-1] && imagin_cam[i-3][j-2] && imagin_cam[i-2][j-1])) //TODO
        {
            OLED_P6x8Str(45,5,".");
            OLED_P6x8Str(2,4," ");
            OLED_P6x8Str(50,5,"sta139");
            return 0;
        }

        for(k=0,i--;k<30;k--)
        {
            if(!imagin_cam[i][j+k])
            {
                OLED_P6x8Str(45,5,".");
                                OLED_P6x8Str(2,4," ");
                                OLED_P6x8Str(50,5,"sta140");
                return 0;
            }
        }

        if(Threeleftinupr>20)
        {
            OLED_P6x8Str(45,5,".");
            OLED_P6x8Str(2,4," ");
            OLED_P6x8Str(50,5,"sta141");
            return 0;
        }

        for(k=40;k>23;k--)  //TODO
        {
            if(!imagin_cam[k][87] && !imagin_cam[k][86])
            {
                return 0;
            }
        }

        cam_Three_Leftin=1;
        return 1;
    }
    OLED_P6x8Str(45,5,".");
    OLED_P6x8Str(2,4," ");
    OLED_P6x8Str(50,5,"sta142");
        return 0;
}


unsigned char Three_Rightin(void)
{
    unsigned char i,j,k;

    if(ThreeupRsidewhi<27)
    {
        OLED_P6x8Str(8,5," sta134");
        OLED_P6x8Str(2,5," ");
        OLED_P6x8Str(2,4,".");
        return 0;
    }

    for(i=2;i<14;i++)
    {
        if(imagin_cam[i][70]||imagin_cam[i][83]|| imagin_cam[i][75]|| imagin_cam[i][62]||imagin_cam[i][90]) break;
    }
    for(i=i+1;i<14;i++)
    {
        if(imagin_cam[i][70]||imagin_cam[i][83]|| imagin_cam[i][75]|| imagin_cam[i][62]||imagin_cam[i][90]) break;
    }
    if(i>12)
    {
        OLED_P6x8Str(2,5,".");
        OLED_P6x8Str(2,4," ");
        OLED_P6x8Str(8,5,"sta127");
        return 0;
    }

    for(i=40,j=92;i>10;i--)
    {
        if(!imagin_cam[i][91] && !imagin_cam[i][92])
        {
            break;
        }
    }
    if(i>12)
    {
        while(j>55 && i>3)
        {
            if(!imagin_cam[i][j-1])
            {
                j--;
            }
            else if(!imagin_cam[i-1][j])
            {
                i--;
            }
            else break;
        }
        if(j<60)
        {
            OLED_P6x8Str(2,5,".");
            OLED_P6x8Str(2,4," ");
            OLED_P6x8Str(8,5,"sta128");
            return 0;
        }
        if(i<6)
        {
            OLED_P6x8Str(2,5,".");
                    OLED_P6x8Str(2,4," ");
                    OLED_P6x8Str(8,5,"sta129");
            return 0;
        }


        if(!(imagin_cam[i-3][j+1] && imagin_cam[i-3][j+2] && imagin_cam[i-2][j+1]))
        {
            OLED_P6x8Str(2,5,".");
                    OLED_P6x8Str(2,4," ");
                    OLED_P6x8Str(8,5,"sta130");
            return 0;
        }

        for(k=0,i--;k<20;k--)
        {
            if(!imagin_cam[i][j-k])
            {
                OLED_P6x8Str(2,5,".");
                        OLED_P6x8Str(2,4," ");
                        OLED_P6x8Str(8,5,"sta131");
                return 0;
            }
        }

        if(Threerightinupl>27)
        {
            OLED_P6x8Str(2,5,".");
                    OLED_P6x8Str(2,4," ");
                    OLED_P6x8Str(8,5,"sta132");
            return 0;
        }
        cam_Three_Rightin=1;

        for(k=40;k>23;k--)  //TODO
        {
            if(!imagin_cam[k][7] && !imagin_cam[k][6])
            {
                return 0;
            }
        }

        return 1;
    }
    OLED_P6x8Str(2,5,".");
            OLED_P6x8Str(2,4," ");
            OLED_P6x8Str(8,5,"sta133");
    return 0;

}



