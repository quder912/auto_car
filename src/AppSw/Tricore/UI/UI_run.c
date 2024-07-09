#include "UI_run.h"
#include "ui_hanshu.h"
#include "Came.h"
#include "get_ima01.h"
#include"LQ_MT9V034.h"
#include"LQ_CAMERA.h"
#include <LQ_DMA.h>
#include <LQ_GPIO.h>
#include "cam_findmid.h"
#include <LQ_GTM.h>
#include "W25Q.h"
#include "forreport.h"
#include "string.h"
#include "allinit.h"
#include "cam_IN.h"

unsigned char txpr[1005];
unsigned char rxpr[1005];

uint8 ui_first_line=1;
uint8 ui_page=1;
uint8 ui_floor=1;
uint8 ui_report_line=1;
uint8 ui_report_floor=1;
_Bool ui_anjian=1;
uint8 ui_var_line=1;
uint8 ui_In_floor=1;
uint8 ui_changeplan_line=1;

//_Bool testdiangan;

short PWM_test6=0;
volatile unsigned int cam_jishi=0;

cam_jurun runstate;
cam_judge rungo;
CAM_Inductance Ind;

_Bool runbeginflag=0;

CAM_var plans[4];
unsigned char plansnum;




void Toplancvar(void)
{
  plans[0].k_nihe=1.79; //未显示 //1.9
  plans[0].b_nihe=5.4; // 6.1

  plans[0].Dst.p=73;
  plans[0].Dst.i=0;
  plans[0].Dst.d=410;
  plans[0].errcamcust=85;
  plans[0].Dcup=86;
  plans[0].Dcud=520;
  plans[0].Dian.p=3000;
  plans[0].Dian.i=300;
  plans[0].Dian.d=0;


  plans[0].speedslow=164;
  plans[0].speedadd=0.00;
  plans[0].speedD=0.00;
  plans[0].speederrp=0.08;
  plans[0].speederrd=0.12;
  plans[0].bangbangplus=50;
  plans[0].bangbangminus=58;


  plans[0].bangpluspwm=9500;
  plans[0].bangminuspwm=3300;

  plans[0].errlinebegin=6;
  plans[0].D_max=1484;
  plans[0].D_min=1500;
  plans[0].D_middle=1440;
  Duo_limitplus=305;
  Duo_limitminus=285;

  plans[0].Ring_sumIN=170;
  plans[0].Ring_turnover=140;
  plans[0].Ring_inmax=160;
  plans[0].Ring_over=160;
  plans[0].Ring_speedin=160;
  plans[0].speed_Ring=180;

  plans[0].befringindelay[0]=6;
  plans[0].befringindelay[1]=6;
  plans[0].befringindelay[2]=6;
  plans[0].befringindelay[3]=6;
  plans[0].Ring_direction[0]=1;
  plans[0].Ring_direction[1]=1;
  plans[0].Ring_direction[2]=1;
  plans[0].Ring_direction[3]=1;
  plans[0].angle_Ring[0]=240;
  plans[0].angle_Ring[1]=240;
  plans[0].angle_Ring[2]=240;
  plans[0].angle_Ring[3]=240;


  plans[0].Ramp_delay=109;
  plans[0].speed_camp=145;
  plans[0].Ist.p=14;
  plans[0].Ist.i=0;
  plans[0].Ist.d=45;
  plans[0].err_istcu=55;
  plans[0].Icu.p=35;
  plans[0].Icu.i=0;
  plans[0].Icu.d=40;

  plans[0].Outga_direction=1;
  plans[0].delay_outgarmid=15;
  plans[0].delay_Out=25;
  plans[0].angle_Outgar=285;
  plans[0].delay_In=5;
  plans[0].angle_Ingar=285;
  plans[0].speed_finishin=70;
  plans[0].speed_finished=50;


  plans[0].delay_threein0=1;
  plans[0].Three_speedin=160;
  plans[0].delay_threeout0=0;
  plans[0].Three_speedout=145;
  plans[0].delay_threeall=181;

  plans[0].Three_direction[0]=0;
  plans[0].Three_direction[1]=1;
  plans[0].angle_Threeout[0]=201;
  plans[0].angle_Threeout[1]=200;
  plans[0].angle_Threein[0]=230;
  plans[0].angle_Threein[1]=200;

  plans[0].delay_threein1=2;
  plans[0].delay_threeout1=0;

  plans[0].finished_time=124;
  plans[0].stop_sum=16;
  plans[0].Ring_sum8=0;
  plans[0].stop_times=124;


  plans[1].k_nihe=1.79; //未显示 //1.9
  plans[1].b_nihe=5.4; // 6.1

  plans[1].Dst.p=90;
  plans[1].Dst.i=0;
  plans[1].Dst.d=460;
  plans[1].errcamcust=98;
  plans[1].Dcup=90;
  plans[1].Dcud=460;
  plans[1].Dian.p=1700;
  plans[1].Dian.i=170;
  plans[1].Dian.d=0;


  plans[1].speedslow=75;
  plans[1].speedadd=0;
  plans[1].speedD=0;
  plans[1].speederrp=0;
  plans[1].speederrd=0.03;
  plans[1].bangbangplus=50;
  plans[1].bangbangminus=58;


  plans[1].bangpluspwm=9500;
  plans[1].bangminuspwm=3300;

  plans[1].errlinebegin=7;
  plans[1].D_max=1750;
  plans[1].D_min=1500;
  plans[1].D_middle=1440;

  plans[1].Ring_sumIN=170;
  plans[1].Ring_turnover=125;
  plans[1].Ring_inmax=150;
  plans[1].Ring_over=155;
  plans[1].Ring_speedin=75;
  plans[1].speed_Ring=75;


  plans[1].befringindelay[0]=19;
  plans[1].befringindelay[1]=19;
  plans[1].befringindelay[2]=19;
  plans[1].befringindelay[3]=8;

  plans[1].Ring_direction[0]=1;
  plans[1].Ring_direction[1]=1;
  plans[1].Ring_direction[2]=0;
  plans[1].Ring_direction[3]=0;

  plans[1].angle_Ring[0]=240;
  plans[1].angle_Ring[1]=240;
  plans[1].angle_Ring[2]=275;
  plans[1].angle_Ring[3]=275;

  plans[1].Ramp_delay=160;
  plans[1].speed_camp=75;
  plans[1].Ist.p=14;
  plans[1].Ist.i=0;
  plans[1].Ist.d=30;
  plans[1].err_istcu=64;
  plans[1].Icu.p=42;
  plans[1].Icu.i=0;
  plans[1].Icu.d=26;

  plans[1].Outga_direction=0;
  plans[1].delay_outgarmid=30;

  plans[1].delay_Out=25;
  plans[1].angle_Outgar=285;
  plans[1].delay_In=4;
  plans[1].angle_Ingar=285;
  plans[1].speed_finishin=75;
  plans[1].speed_finished=35;

  plans[1].delay_threein0=3;
  plans[1].Three_speedin=65;
  plans[1].delay_threeout0=1;
  plans[1].Three_speedout=65;
  plans[1].delay_threeall=275;

  plans[1].Three_direction[0]=0;
  plans[1].Three_direction[1]=1;
  plans[1].angle_Threeout[0]=245;
  plans[1].angle_Threeout[1]=245;


  plans[1].angle_Threein[0]=230;
  plans[1].angle_Threein[1]=230;

  plans[1].finished_time=16;
  plans[1].stop_sum=27;
  plans[1].Ring_sum8=20;



  plans[2].k_nihe=1.79; //未显示 //1.9
  plans[2].b_nihe=5.4; // 6.1

  plans[2].Dst.p=110;
  plans[2].Dst.i=0;
  plans[2].Dst.d=750;
  plans[2].errcamcust=48;
  plans[2].Dcup=110;
  plans[2].Dcud=800;
  plans[2].Dian.p=3000;
  plans[2].Dian.i=300;
  plans[2].Dian.d=0;


  plans[2].speedslow=190;
  plans[2].speedadd=0.1;
  plans[2].speedD=0.08;
  plans[2].speederrp=0.30;
  plans[2].speederrd=0.08;
  plans[2].bangbangplus=53;
  plans[2].bangbangminus=50;


  plans[2].bangpluspwm=9500;
  plans[2].bangminuspwm=3300;

  plans[2].errlinebegin=4;
  plans[2].D_max=1484;
  plans[2].D_min=1500;
  plans[2].D_middle=1440;

  plans[2].Ring_sumIN=170;
  plans[2].Ring_turnover=130;
  plans[2].Ring_inmax=160;
  plans[2].Ring_over=165;
  plans[2].Ring_speedin=156;
  plans[2].speed_Ring=215;

  plans[2].befringindelay[0]=5;
  plans[2].befringindelay[1]=5;
  plans[2].befringindelay[2]=5;
  plans[2].befringindelay[3]=5;
  plans[2].Ring_direction[0]=1;
  plans[2].Ring_direction[1]=1;
  plans[2].Ring_direction[2]=1;
  plans[2].Ring_direction[3]=1;
  plans[2].angle_Ring[0]=245;
  plans[2].angle_Ring[1]=245;
  plans[2].angle_Ring[2]=245;
  plans[2].angle_Ring[3]=245;


  plans[2].Ramp_delay=70;
  plans[2].speed_camp=137;
  plans[2].Ist.p=13;
  plans[2].Ist.i=0;
  plans[2].Ist.d=45;
  plans[2].err_istcu=55;
  plans[2].Icu.p=45;
  plans[2].Icu.i=0;
  plans[2].Icu.d=67;

  plans[2].Outga_direction=1;
  plans[2].delay_outgarmid=15;
  plans[2].delay_Out=25;
  plans[2].angle_Outgar=285;
  plans[2].delay_In=3;
  plans[2].angle_Ingar=285;
  plans[2].speed_finishin=80;
  plans[2].speed_finished=75;


  plans[2].delay_threein0=0;
  plans[2].Three_speedin=165;
  plans[2].delay_threeout0=0;
  plans[2].Three_speedout=160;
  plans[2].delay_threeall=275;

  plans[2].Three_direction[0]=0;
  plans[2].Three_direction[1]=1;
  plans[2].angle_Threeout[0]=237;
  plans[2].angle_Threeout[1]=202;
  plans[2].angle_Threein[0]=243;
  plans[2].angle_Threein[1]=218;

  plans[2].delay_threein1=1;
  plans[2].delay_threeout1=3;


  plans[2].finished_time=25;
  plans[2].stop_sum=20;
  plans[2].Ring_sum8=0;

  plans[2].stop_times=123;

  plans[3].k_nihe=1.79; //未显示 //1.9
  plans[3].b_nihe=5.4; // 6.1

  plans[3].Dst.p=92;
  plans[3].Dst.i=0;
  plans[3].Dst.d=565;
  plans[3].errcamcust=98;
  plans[3].Dcup=110;
  plans[3].Dcud=710;
  plans[3].Dian.p=3515;
  plans[3].Dian.i=350;
  plans[3].Dian.d=2;


  plans[3].speedslow=95;
  plans[3].speedadd=0.36;
  plans[3].speedD=0.7;
  plans[3].speederrp=0.12;
  plans[3].speederrd=0.65;
  plans[3].bangbangplus=25;
  plans[3].bangbangminus=30;


  plans[3].bangpluspwm=9500;
  plans[3].bangminuspwm=3400;

  plans[3].errlinebegin=5;
  plans[3].D_max=1750;
  plans[3].D_min=1500;
  plans[3].D_middle=1440;

  plans[3].Ring_sumIN=160;
  plans[3].Ring_turnover=125;
  plans[3].Ring_inmax=150;
  plans[3].Ring_over=160;
  plans[3].Ring_speedin=85;
  plans[3].speed_Ring=105;

  plans[3].befringindelay[0]=8;
  plans[3].befringindelay[1]=8;
  plans[3].befringindelay[2]=8;
  plans[3].befringindelay[3]=8;
  plans[3].Ring_direction[0]=1;
  plans[3].Ring_direction[1]=1;
  plans[3].Ring_direction[2]=0;
  plans[3].Ring_direction[3]=0;
  plans[3].angle_Ring[0]=235;
  plans[3].angle_Ring[1]=235;
  plans[3].angle_Ring[2]=235;
  plans[3].angle_Ring[3]=235;


  plans[3].Ramp_delay=110;
  plans[3].speed_camp=80;
  plans[3].Ist.p=14;
  plans[3].Ist.i=0;
  plans[3].Ist.d=31;
  plans[3].err_istcu=64;
  plans[3].Icu.p=42;
  plans[3].Icu.i=0;
  plans[3].Icu.d=40;

  plans[3].Outga_direction=0;
  plans[3].delay_outgarmid=40;
  plans[3].delay_Out=30;
  plans[3].angle_Outgar=285;
  plans[3].delay_In=2;
  plans[3].angle_Ingar=285;
  plans[3].speed_finishin=35;
  plans[3].speed_finished=35;


  plans[3].delay_threein0=0;
  plans[3].Three_speedin=75;
  plans[3].delay_threeout0=0;
  plans[3].Three_speedout=65;
  plans[3].delay_threeall=275;

  plans[3].Three_direction[0]=0;
  plans[3].Three_direction[1]=1;
  plans[3].angle_Threeout[0]=260;
  plans[3].angle_Threeout[1]=260;
  plans[3].angle_Threein[0]=225;
  plans[3].angle_Threein[1]=225;

  plans[3].finished_time=7;
  plans[3].stop_sum=27;
  plans[3].Ring_sum8=1;

}



void UI_begin(void)
{
  ui_anjian=1;
  while(ui_floor<=3)
  {

    /*
                            _ooOoo_
                         o8888888o
                         88" . "88
                         (| -_- |)
                         O\ = /O
                        ____/`---'\____
                     .' \\|     |// `.
                    / \\||| : |||// \
                    / _||||| -:- |||||- \
                    |    | \\\ - /// |    |
                    | \_| ''\---/'' |    |
                    \ .-\__ `-` ___/-. /
                 ___`. .' /--.--\ `. . __
             ."" '< `.___\_<|>_/___.' >'"".
             | | : `- \`.;`\ _ /`;.`/ - ` : | |
             \ \ `-.    \_ __\ /__ _/    .-` / /
        ======`-.____`-.___\_____/___.-`____.-'======
                            `=---='
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          佛祖保佑    永无BUG
    */

    UI_First(key_scan(0XFF));
  }

}


void UI_First(uint8 KEY)
{
  char txt[21];

  if(ui_floor==1 && ui_anjian)
  {
    OLED_Write68(2,0," 1_  _Change  var   ",(ui_first_line!=1));
    OLED_Write68(2,1," 2_  _Report  cam   ",(ui_first_line!=2));
    OLED_Write68(2,2," 3_  _Run     car   ",(ui_first_line!=3));
    OLED_Write68(2,3," 4_  _Test    Induct",(ui_first_line!=4));
    OLED_Write68(2,4," 5_  _Just    Induct",(ui_first_line!=5));
    OLED_Write68(2,5," 6_  _Test    Machi ",(ui_first_line!=6));
    sprintf(txt," 7_  _Chplan  %03d   ",plansnum);
    OLED_Write68(2,6,txt,(ui_first_line!=7));
    OLED_P6x8Str(2,7,"     UESTC V4.2      ");
    ui_anjian=0;
  }
  else if(ui_floor==2)
  {
    switch(ui_first_line)
    {
      case 1:
        UI_Changevar(KEY);
        return;
        break;
      case 2:
        UI_Reportcam(KEY);
        return;
        break;
      case 3:
        ui_floor=4;
        return;
        break;
      case 4:
        cam_Test_Inductance(KEY);
        return;
        break;
      case 5:
        camrunIN(&Ind,&planc);
        break;
      case 6:
        camtestmach(KEY);
        break;
      case 7:
        change_plan(KEY);
        break;

    }
    return;
  }

  if((KEY&key_up)==key_up)
  {
    ui_anjian=1;
    if(ui_first_line<=1)
    {
      ui_first_line=1;
    }
    else
    {
      ui_first_line--;
    }
  }
  else if((KEY&key_down)==key_down)
  {
    ui_anjian=1;
    if(ui_first_line>=7)
    {
      ui_first_line=7;
    }
    else
    {
      ui_first_line++;
    }
  }
  else if((KEY&key_in)==key_in)
  {
    ui_anjian=1;
    ui_floor++;
  }
  return;
}



void UI_Changevar(uint8 KEY)
{
  char txt[22];
  if(ui_var_line<=7)
  {
    OLED_Write68(2,0,"         MULU       ",1);

    OLED_Write68(2,1," 8   cam_pid        ",ui_var_line!=1);
    OLED_Write68(2,2," 16  normal speed   ",ui_var_line!=2);
    OLED_Write68(2,3," 26  machine        ",ui_var_line!=3);
    OLED_Write68(2,4," 29  Ring           ",ui_var_line!=4);
    OLED_Write68(2,5," 47  Ramp           ",ui_var_line!=5);
    OLED_Write68(2,6," 54  Finished       ",ui_var_line!=6);
    OLED_Write68(2,7," 64  Three          ",ui_var_line!=7);

  }
  else if (ui_var_line>=8 && ui_var_line<=15)
  {
    sprintf(txt," p_str :    %05d   ",planc.Dst.p);
    OLED_Write68(2,0,txt,ui_var_line!=8);

    sprintf(txt," d_str :    %05d   ",planc.Dst.d);
    OLED_Write68(2,1,txt,ui_var_line!=9);

    sprintf(txt,"  errcs0 :   %05d  ",planc.errcamcust);
    OLED_Write68(2,2,txt,ui_var_line!=10);

    sprintf(txt," p_cur  :   %05d   ",planc.Dcup);
    OLED_Write68(2,3,txt,ui_var_line!=11);

    sprintf(txt," d_cur  :   %05d   ",planc.Dcud);
    OLED_Write68(2,4,txt,ui_var_line!=12);

    sprintf(txt,"   p_enc :   %05d ",planc.Dian.p);
    OLED_Write68(2,5,txt,ui_var_line!=13);

    sprintf(txt,"   i_enc :   %05d ",planc.Dian.i);
    OLED_Write68(2,6,txt,ui_var_line!=14);

    sprintf(txt,"   d_enc :   %05d ",planc.Dian.d);
    OLED_Write68(2,7,txt,ui_var_line!=15);

  }
  else if (ui_var_line>=16 && ui_var_line<=22)
  {
    sprintf(txt," speedslow : %05d ",planc.speedslow);
    OLED_Write68(2,0,txt,ui_var_line!=16);

    sprintf(txt," speedadd : %06.2f ",planc.speedadd);
    OLED_Write68(2,1,txt,ui_var_line!=17);

    sprintf(txt," speedD  :  %05.2f ",planc.speedD);
    OLED_Write68(2,2,txt,ui_var_line!=18);

    sprintf(txt," speerp  :  %05.2f ",planc.speederrp);
    OLED_Write68(2,3,txt,ui_var_line!=19);

    sprintf(txt," speerd  :  %05.2f ",planc.speederrd);
    OLED_Write68(2,4,txt,ui_var_line!=20);

    sprintf(txt," Bang_dif+ : %05d ",planc.bangbangplus);
    OLED_Write68(2,5,txt,ui_var_line!=21);

    sprintf(txt," Bang_dif- : %05d ",planc.bangbangminus);
    OLED_Write68(2,6,txt,ui_var_line!=22);

    OLED_Write68(2,7,"                    ",1);


  }
  else if(ui_var_line>=23 && ui_var_line<=28)
  {

    sprintf(txt," Bangb+pwm : %05d ",planc.bangpluspwm);
    OLED_Write68(2,0,txt,ui_var_line!=23);
    sprintf(txt," Bangb-pwm : %05d ",planc.bangminuspwm);
    OLED_Write68(2,1,txt,ui_var_line!=24);

    OLED_Write68(2,2,"                    ",1);

    sprintf(txt," errlineb  :  %03d  ",planc.errlinebegin);
    OLED_Write68(2,3,txt,ui_var_line!=25);

    sprintf(txt," Du_mid:   %05d  ",planc.D_middle);
    OLED_Write68(2,4,txt,ui_var_line!=26);

    sprintf(txt," limitplus:  %05d ",Duo_limitplus);
    OLED_Write68(2,5,txt,ui_var_line!=27);

    sprintf(txt," limitminus: %05d ",Duo_limitminus);
    OLED_Write68(2,6,txt,ui_var_line!=28);

    OLED_Write68(2,7,"                    ",1);

  }
  else if(ui_var_line>=29 && ui_var_line<=34)
  {

    sprintf(txt," Ring_sum :  %05d ",planc.Ring_sumIN);
    OLED_Write68(2,0,txt,ui_var_line!=29);

    sprintf(txt," Ring_trov : %05d ",planc.Ring_turnover);
    OLED_Write68(2,1,txt,ui_var_line!=30);

    sprintf(txt," Ring_inmax: %05d ",planc.Ring_inmax);
    OLED_Write68(2,2,txt,ui_var_line!=31);

    sprintf(txt," Ring_over : %05d ",planc.Ring_over);
    OLED_Write68(2,3,txt,ui_var_line!=32);

    sprintf(txt," Ring_spein: %05d ",planc.Ring_speedin);
    OLED_Write68(2,4,txt,ui_var_line!=33);

    sprintf(txt," speedRing: %05d ",planc.speed_Ring);
    OLED_Write68(2,5,txt,ui_var_line!=34);

    OLED_Write68(2,6,"                    ",1);
    OLED_Write68(2,7,"                    ",1);

  }
  else if(ui_var_line>=35 && ui_var_line<=40)
  {
    sprintf(txt," Ring0_indel: %03d ",planc.befringindelay[0]);
    OLED_Write68(2,0,txt,ui_var_line!=35);

    sprintf(txt," Ring_1    :   %d  ",planc.Ring_direction[0]);
    OLED_Write68(2,1,txt,ui_var_line!=36);

    sprintf(txt," angle_Ring0: %05d ",planc.angle_Ring[0]);
    OLED_Write68(2,2,txt,ui_var_line!=37);
    OLED_Write68(2,3,"                    ",1);

    sprintf(txt," Ring1_indel: %03d ",planc.befringindelay[1]);
    OLED_Write68(2,4,txt,ui_var_line!=38);

    sprintf(txt," Ring_2   :   %d  ",planc.Ring_direction[1]);
    OLED_Write68(2,5,txt,ui_var_line!=39);

    sprintf(txt," angle_Ring1 : %05d ",planc.angle_Ring[1]);
    OLED_Write68(2,6,txt,ui_var_line!=40);

    OLED_Write68(2,7,"                    ",1);

  }
  else if(ui_var_line>=41 && ui_var_line<=46)
  {

    sprintf(txt," Ring2_indel : %03d ",planc.befringindelay[2]);
    OLED_Write68(2,0,txt,ui_var_line!=41);
    sprintf(txt," Ring_3    :    %d   ",planc.Ring_direction[2]);
    OLED_Write68(2,1,txt,ui_var_line!=42);
    sprintf(txt," angle_Ring2: %05d ",planc.angle_Ring[2]);
    OLED_Write68(2,2,txt,ui_var_line!=43);

    OLED_Write68(2,3,"                    ",1);

    sprintf(txt," Ring3_indel : %03d ",planc.befringindelay[3]);
    OLED_Write68(2,4,txt,ui_var_line!=44);

    sprintf(txt," Ring_4    :   %d   ",planc.Ring_direction[3]);
    OLED_Write68(2,5,txt,ui_var_line!=45);

    sprintf(txt," angle_Ring3: %05d ",planc.angle_Ring[3]);
    OLED_Write68(2,6,txt,ui_var_line!=46);

    OLED_Write68(2,7,"                    ",1);


  }
  else if(ui_var_line>=47 && ui_var_line<=53)
  {
      OLED_Write68(2,0,"                    ",1);

    sprintf(txt," Ramp_dela : %05d ",planc.Ramp_delay);
    OLED_Write68(2,1,txt,ui_var_line!=47);


    sprintf(txt," speed_camp :  %03d  ",planc.speed_camp);
    OLED_Write68(2,2,txt,ui_var_line!=48);

    sprintf(txt," p_ist  :    %03d   ",planc.Ist.p);
    OLED_Write68(2,3,txt,ui_var_line!=49);

    sprintf(txt," d_ist  :    %03d   ",planc.Ist.d);
    OLED_Write68(2,4,txt,ui_var_line!=50);

    sprintf(txt,"  err_istcu: %05d ",planc.err_istcu);
    OLED_Write68(2,5,txt,ui_var_line!=51);

    sprintf(txt," p_icu  :    %03d    ",planc.Icu.p);
    OLED_Write68(2,6,txt,ui_var_line!=52);

    sprintf(txt," d_icu  :    %03d    ",planc.Icu.d);
    OLED_Write68(2,7,txt,ui_var_line!=53);

  }
  else if(ui_var_line>=54 && ui_var_line<=61)
  {
    sprintf(txt," Out_direct :   %d ",planc.Outga_direction);
    OLED_Write68(2,0,txt,ui_var_line!=54);

    sprintf(txt," Out_delaymid :  %d ",planc.delay_outgarmid);
    OLED_Write68(2,1,txt,ui_var_line!=55);

    sprintf(txt," Ou_turnde :  %03d   ",planc.delay_Out);
    OLED_Write68(2,2,txt,ui_var_line!=56);

    sprintf(txt," angle_Outga: %05d ",planc.angle_Outgar);
    OLED_Write68(2,3,txt,ui_var_line!=57);

    sprintf(txt," In_midddel : %03d ",planc.delay_In);
    OLED_Write68(2,4,txt,ui_var_line!=58);

    sprintf(txt," angle_Ingar: %05d ",planc.angle_Ingar);
    OLED_Write68(2,5,txt,ui_var_line!=59);

    sprintf(txt," speedinmid : %05d ",planc.speed_finishin);
    OLED_Write68(2,6,txt,ui_var_line!=60);

    sprintf(txt,"speedinturn : %05d ",planc.speed_finished);
    OLED_Write68(2,7,txt,ui_var_line!=61);


  }
  else if(ui_var_line>=61 && ui_var_line<=69)
  {

    sprintf(txt," Thr_delayin0: %03d ",planc.delay_threein0);
    OLED_Write68(2,0,txt,ui_var_line!=62);

    sprintf(txt," Three_spein : %05d ",planc.Three_speedin);
    OLED_Write68(2,1,txt,ui_var_line!=63);

    sprintf(txt," Thr_delayout0: %03d ",planc.delay_threeout0);
    OLED_Write68(2,2,txt,ui_var_line!=64);

    sprintf(txt," Three_spout : %05d ",planc.Three_speedout);
    OLED_Write68(2,3,txt,ui_var_line!=65);

    sprintf(txt," Th_delall  : %05d ",planc.delay_threeall);
    OLED_Write68(2,4,txt,ui_var_line!=66);

    sprintf(txt," Th_direct0 :    %d   ",planc.Three_direction[0]);
    OLED_Write68(2,5,txt,ui_var_line!=67);

    sprintf(txt," angle_Thout0 :%05d ",planc.angle_Threeout[0]);
    OLED_Write68(2,6,txt,ui_var_line!=68);

    sprintf(txt," angle_Thrin0 :%05d ",planc.angle_Threein[0]);
    OLED_Write68(2,7,txt,ui_var_line!=69);

  }
  else if(ui_var_line>=70 && ui_var_line<=77)
  {

    sprintf(txt," Th_direct1  :    %d  ",planc.Three_direction[1]);
    OLED_Write68(2,0,txt,ui_var_line!=70);

    sprintf(txt," angle_Thout1 :%05d ",planc.angle_Threeout[1]);
    OLED_Write68(2,1,txt,ui_var_line!=71);

    sprintf(txt," angle_Thrin1 :%05d ",planc.angle_Threein[1]);
    OLED_Write68(2,2,txt,ui_var_line!=72);

    sprintf(txt," Thr_delayin1: %03d ",planc.delay_threein1);
    OLED_Write68(2,3,txt,ui_var_line!=73);

    sprintf(txt," Thr_delayout1: %03d ",planc.delay_threeout1);
    OLED_Write68(2,4,txt,ui_var_line!=74);


//    OLED_Write68(2,5,"                    ",1);
//    sprintf(txt," finisturndel : %03d ",planc.finished_time);
//    OLED_Write68(2,5,txt,ui_var_line!=75);
    sprintf(txt," stopsum :    %03d   ",planc.stop_sum);
    OLED_Write68(2,5,txt,ui_var_line!=75);
    sprintf(txt," Ringsum8 :   %03d   ",planc.Ring_sum8);
    OLED_Write68(2,6,txt,ui_var_line!=76);

    sprintf(txt," Stop_time :  %03d   ",planc.stop_times);
    OLED_Write68(2,7,txt,ui_var_line!=77);

////    OLED_Write68(2,,"                    ",1);
//    OLED_Write68(2,6,"                    ",1);
//    OLED_Write68(2,7,"                    ",1);

  }
//  else if(ui_var_line>=70 && ui_var_line<=77)
//  {
//      OLED_Write68(2,6,"                    ",1);
//      OLED_Write68(2,7,"                    ",1);
//
//  }


  if((KEY&key_up)==key_up)
  {
    if(ui_var_line<2)
    {
      ui_var_line=77;
    }
    else
    {
      ui_var_line--;
    }
  }
  else if((KEY&key_down)==key_down)
  {
    if(ui_var_line>76)
    {
      ui_var_line=1;
    }
    else
    {
      ui_var_line++;
    }
  }
  else if((KEY&key_left)==key_left)
  {
    switch(ui_var_line)
    {
      case 1:
              ui_var_line=8;
              break;

            case 2:
              ui_var_line=16;
              break;

            case 3:
              ui_var_line=26;
              break;

            case 4:
              ui_var_line=29;
              break;

            case 5:
              ui_var_line=47;
              break;


            case 6:
              ui_var_line=54;
              break;

            case 7:
              ui_var_line=64;
              break;


            case 8:
              if(planc.Dst.p>0)
              {
                planc.Dst.p--;
              }
              break;

            case 9:
              if(planc.Dst.d>0)
              {
                planc.Dst.d--;
              }
              break;

            case 10:
              if(planc.errcamcust>0)
              {
                planc.errcamcust--;
              }
              break;


            case 11:
              if(planc.Dcup>0)
              {
                planc.Dcup--;
              }
              break;

            case 12:
              if(planc.Dcud>0)
              {
                planc.Dcud--;
              }
              break;

            case 13:
              if(planc.Dian.p>0)
              {
                planc.Dian.p--;
              }
              break;

            case 14:
              if(planc.Dian.i>0)
              {
                planc.Dian.i--;
              }
              break;

            case 15:
              if(planc.Dian.d>0)
              {
                planc.Dian.d--;
              }
              break;


            case 16:
              if(planc.speedslow>0)
              {
                planc.speedslow--;
              }break;

            case 17:
              if(planc.speedadd>0.00f)
              {
                planc.speedadd-=0.01f;
              }break;

            case 18:
              if(planc.speedD>0.00f)
              {
                planc.speedD-=0.01f;
              }break;

            case 19:
               if(planc.speederrp>0.00f)
               {
                 planc.speederrp-=0.01f;
               }
               break;
            case 20:
              if(planc.speederrd>0.00f)
              {
                planc.speederrd-=0.01f;
              }
              break;

            case 21:
              if(planc.bangbangplus>0)
              {
                planc.bangbangplus--;
              }
              break;
            case 22:
              if(planc.bangbangminus>0)
              {
                planc.bangbangminus--;
              }
              break;

            case 23:
              if(planc.bangpluspwm>10)
              {
                planc.bangpluspwm-=10;
              }
              break;
            case 24:
              if(planc.bangminuspwm>10)
              {
                planc.bangminuspwm-=10;
              }
              break;

            case 25:
              if(planc.errlinebegin>3)
              {
                planc.errlinebegin--;
              }
              else
              {
                planc.errlinebegin=3;
              }
              break;

            case 26:
              if(planc.D_middle>1100)
              {
                planc.D_middle--;
                ATOM_PWM_SetDuty(STEER_IO,planc.D_middle, 100);
              }
              break;

            case 27:
              if(Duo_limitplus>0)
              {
                Duo_limitplus--;
              }
              break;

            case 28:
              if(Duo_limitminus>0)
              {
                Duo_limitminus--;
              }
              break;

            case 29:
              if(planc.Ring_sumIN>0)
              {
                planc.Ring_sumIN--;
              }break;


            case 30:
              if(planc.Ring_turnover>=0)
              {
                planc.Ring_turnover--;
              }
              break;
            case 31:
               if(planc.Ring_inmax>=0)
               {
                 planc.Ring_inmax--;
               }
               break;
            case 32:
              if(planc.Ring_over>=0)
              {
                planc.Ring_over--;
              }
              break;
            case 33:
              if(planc.Ring_speedin>0)
              {
                planc.Ring_speedin--;
              }
              break;

            case 34:
              if(planc.speed_Ring>0)
              {
                planc.speed_Ring--;
              }
              break;
            case 35:
              if(planc.befringindelay[0]>0)
              {
                planc.befringindelay[0]--;
              }break;

            case 36:
              planc.Ring_direction[0]=0;
              break;

            case 37:
              if(planc.angle_Ring[0]>0)
              {
                planc.angle_Ring[0]--;
              }
              break;


            case 38:
              if(planc.befringindelay[1]>0)
              {
                planc.befringindelay[1]--;
              }
              break;

            case 39:
              planc.Ring_direction[1]=0;
              break;


            case 40:
              if(planc.angle_Ring[1]>0)
              {
                planc.angle_Ring[1]--;
              }
              break;


            case 41:
              if(planc.befringindelay[2]>0)
              {
                planc.befringindelay[2]--;
              }
              break;

            case 42:
              planc.Ring_direction[2]=0;
              break;

            case 43:
              if(planc.angle_Ring[2]>0)
              {
                planc.angle_Ring[2]--;
              }
              break;

            case 44:
              if(planc.befringindelay[3]>0)
              {
                planc.befringindelay[3]--;
              }
              break;

            case 45:
              planc.Ring_direction[3]=0;
              break;

            case 46:
              if(planc.angle_Ring[3]>0)
              {
                planc.angle_Ring[3]--;
              }
              break;

            case 47:
              if(planc.Ramp_delay>0)
              {
                planc.Ramp_delay--;
              }
              break;

            case 48:
              if(planc.speed_camp>0)
              {
                planc.speed_camp--;
              }
              break;



            case 49:
              if(planc.Ist.p>0)
              {
                planc.Ist.p--;
              }
              break;

            case 50:
              if(planc.Ist.d>0)
              {
                planc.Ist.d--;
              }
              break;

            case 51:
              if(planc.err_istcu>0)
              {
                planc.err_istcu--;
              }
              break;

            case 52:
              if(planc.Icu.p>0)
              {
                planc.Icu.p--;
              }
              break;

            case 53:
              if(planc.Icu.d>0)
              {
                planc.Icu.d--;
              }
              break;

            case 54:
              planc.Outga_direction=0;
              break;

            case 55:
              if(planc.delay_outgarmid>0)
              {
                planc.delay_outgarmid--;
              }
              break;


            case 56:
              if(planc.delay_Out>0)
              {
                planc.delay_Out--;
              }
              break;

            case 57:
               if(planc.angle_Outgar>0)
               {
                 planc.angle_Outgar--;
               }
               break;

            case 58:
              if(planc.delay_In>0)
              {
                planc.delay_In--;
              }
              break;

            case 59:
               if(planc.angle_Ingar>0)
               {
                 planc.angle_Ingar--;
               }
               break;

            case 60:
              if(planc.speed_finishin>0)
              {
                planc.speed_finishin--;
              }
              break;
            case 61:
              if(planc.speed_finished>0)
              {
                planc.speed_finished--;
              }
              break;

            case 62:
              if(planc.delay_threein0>0)
              {
                planc.delay_threein0--;
              }
              break;

            case 63:
             if(planc.Three_speedin>0)
             {
               planc.Three_speedin--;
             }
             break;

            case 64:
              if(planc.delay_threeout0>0)
              {
                planc.delay_threeout0--;
              }
              break;

            case 65:
              if(planc.Three_speedout>0)
              {
                planc.Three_speedout--;
              }
              break;

           case 66:
             if(planc.delay_threeall>0)
             {
               planc.delay_threeall--;
             }
             break;

           case 67:
             planc.Three_direction[0]=0;
             break;

           case 68:
              if(planc.angle_Threeout[0]>0)
              {
                planc.angle_Threeout[0]--;
              }
              break;
           case 69:
              if(planc.angle_Threein[0]>0)
              {
                planc.angle_Threein[0]--;
              }
              break;


           case 70:
             planc.Three_direction[1]=0;
             break;

           case 71:
              if(planc.angle_Threeout[1]>0)
              {
                planc.angle_Threeout[1]--;
              }
              break;
           case 72:
              if(planc.angle_Threein[1]>0)
              {
                planc.angle_Threein[1]--;
              }
              break;

           case 73:
             if(planc.delay_threein1>0)
             {
               planc.delay_threein1--;
             }
             break;

           case 74:
            if(planc.delay_threeout1>0)
            {
              planc.delay_threeout1--;
            }
            break;

//           case 73:
//
//               if(planc.finished_time>0)
//           planc.finished_time--;
//
//           break;

           case 75:
               if(planc.stop_sum>0)
           planc.stop_sum--;
           break;

           case 76:
              if(planc.Ring_sum8>0)
              planc.Ring_sum8--;
              break;

           case 77:
               if(planc.stop_times>0)
               planc.stop_times--;
               break;

    }
   }
  else if((KEY&key_right)==key_right)
  {
    switch(ui_var_line)
    {
      case 1:
      ui_var_line=8;
      break;

      case 2:
      ui_var_line=16;
      break;

      case 3:
      ui_var_line=26;
      break;

      case 4:
      ui_var_line=29;
      break;

      case 5:
      ui_var_line=47;
      break;


      case 6:
      ui_var_line=54;
      break;

      case 7:
      ui_var_line=64;
      break;


      case 8:

      planc.Dst.p++;

      break;

      case 9:

      planc.Dst.d++;

      break;

      case 10:

      planc.errcamcust++;

      break;


      case 11:

      planc.Dcup++;

      break;

      case 12:

      planc.Dcud++;

      break;

      case 13:

      planc.Dian.p++;

      break;

      case 14:

      planc.Dian.i++;

      break;

      case 15:
      planc.Dian.d++;

      break;

      case 16:
      planc.speedslow++;
      break;

      case 17:
      planc.speedadd+=0.01f;
      break;

      case 18:
      planc.speedD+=0.01f;
      break;

      case 19:
      planc.speederrp+=0.01f;
      break;
      case 20:
      planc.speederrd+=0.01f;
      break;

      case 21:
      planc.bangbangplus++;

      break;
      case 22:
      planc.bangbangminus++;

      break;

      case 23:
      if(planc.bangpluspwm<10000)
      {
      planc.bangpluspwm+=10;
      }
      break;
      case 24:
      if(planc.bangminuspwm<10000)
      {
      planc.bangminuspwm+=10;
      }
      break;

      case 25:
      if(planc.errlinebegin<20)
      {
      planc.errlinebegin++;
      }
      else
      {
      planc.errlinebegin=20;
      }
      break;

      case 26:
      if(planc.D_middle<1900)
      {
      planc.D_middle++;
      ATOM_PWM_SetDuty(STEER_IO,planc.D_middle, 100);
      }
      break;

      case 27:

      Duo_limitplus++;

      break;

      case 28:

      Duo_limitminus++;

      break;

      case 29:

      planc.Ring_sumIN++;
      break;


      case 30:

      planc.Ring_turnover++;

      break;
      case 31:

      planc.Ring_inmax++;

      break;
      case 32:

      planc.Ring_over++;

      break;
      case 33:

      planc.Ring_speedin++;

      break;

      case 34:

      planc.speed_Ring++;

      break;

      case 35:

      planc.befringindelay[0]++;
      break;
      case 36:
      planc.Ring_direction[0]=1;
      break;

      case 37:

      planc.angle_Ring[0]++;

      break;


      case 38:

      planc.befringindelay[1]++;

      break;

      case 39:
      planc.Ring_direction[1]=1;
      break;


      case 40:

      planc.angle_Ring[1]++;

      break;


      case 41:

      planc.befringindelay[2]++;

      break;

      case 42:
      planc.Ring_direction[2]=1;
      break;

      case 43:

      planc.angle_Ring[2]++;

      break;

      case 44:

      planc.befringindelay[3]++;

      break;

      case 45:
      planc.Ring_direction[3]=1;
      break;

      case 46:

      planc.angle_Ring[3]++;

      break;

      case 47:

      planc.Ramp_delay++;

      break;

      case 48:

      planc.speed_camp++;

      break;

      case 49:

      planc.Ist.p++;

      break;

      case 50:

      planc.Ist.d++;

      break;

      case 51:

      planc.err_istcu++;

      break;

      case 52:

      planc.Icu.p++;

      break;

      case 53:

      planc.Icu.d++;

      break;

      case 54:
      planc.Outga_direction=1;
      break;

      case 55:

      planc.delay_outgarmid++;

      break;


      case 56:

      planc.delay_Out++;

      break;

      case 57:

      planc.angle_Outgar++;

      break;

      case 58:

      planc.delay_In++;

      break;

      case 59:

      planc.angle_Ingar++;

      break;

      case 60:

      planc.speed_finishin++;

      break;
      case 61:

      planc.speed_finished++;

      break;

      case 62:

      planc.delay_threein0++;

      break;

      case 63:

      planc.Three_speedin++;

      break;

      case 64:

      planc.delay_threeout0++;

      break;

      case 65:

      planc.Three_speedout++;

      break;

      case 66:

      planc.delay_threeall++;

      break;

      case 67:
      planc.Three_direction[0]=1;
      break;

      case 68:
      planc.angle_Threeout[0]++;

      break;
      case 69:

      planc.angle_Threein[0]++;

      break;


      case 70:
      planc.Three_direction[1]=1;
      break;

      case 71:


      planc.angle_Threeout[1]++;

      break;
      case 72:
      planc.angle_Threein[1]++;
      break;

      case 73:
          planc.delay_threein1++;


//      case 73:
//
//
//      planc.finished_time++;

      break;

      case 74:
          planc.delay_threeout1++;
          break;


      case 75:
      planc.stop_sum++;
      break;

      case 76:
     planc.Ring_sum8++;
     break;

      case 77:
          planc.stop_times++;
          break;

    }
   }
  else if((KEY&key_in)==key_in)
  {
    ui_floor=4;
  //  main_run(&planc,&runc,&line_fit,&Ind);
  }
  else if((KEY&key_out)==key_out)
  {
    ui_floor--;
    ui_anjian=1;
    ui_var_line=1;
  }
}

void UI_Reportcam(uint8 KEY)
{
  OLED_Write68(2,0,"UP:    120_188 ",1);
  OLED_Write68(2,1,"DOWN:   grey94  ",1);
  OLED_Write68(2,2,"LEFT:   black01 ",1);
  OLED_Write68(2,3,"RIGHT:  LINE_b  ",1);
  OLED_Write68(2,4,"IN:    LINE_a  ",1);
  OLED_Write68(2,5,"          ",1);

  switch(KEY)
  {
    case 0X04:
      OLED_Write68(2,0,"UP:    120_188 ",0);
      while(1)
      {
        if (Camera_Flag == 2)
        {
          CAMERA_Reprot_switch(0);
          Camera_Flag = 0;
        }
      }break;

    case 0X08:
      OLED_Write68(2,1,"DOWN:   grey94  ",0);
      while(1)
      {
        if (Camera_Flag == 2)
        {
          Get_Image_grey();
          CAMERA_Reprot_switch(1);
          Camera_Flag=0;
        }
      }break;

    case 0X10:
      OLED_Write68(2,2,"LEFT:   black01 ",0);
      while(1)
      {
        if (Camera_Flag == 2)
        {
          Get_Image_grey();
          Get_Image01(0);
          CAMERA_Reprot_switch (2);
          Camera_Flag=0;
        }
      }break;

    case 0X20:
      OLED_Write68(2,3,"RIGHT:   LINE_b ",0);
      while(1)
      {
        if (Camera_Flag == 2)
        {
          for(uint8 i=0;i<60;i++)
          {
            Point_Right[i]=0;
            Point_Left[i]=0;
          }
          Get_Image_grey();
          Get_Image01(0);
          ImageDfs();
          CAMERA_Reprot_switch(3);
          Camera_Flag=0;
        }
      }break;

    case 0X01:
      OLED_Write68(2,4,"IN:    LINE_a ",0);
      while(1)
      {
        if (Camera_Flag == 2)
        {
          Get_Cam_Process_Val(&planc,&line_fit,&Ind);
          CAMERA_Reprot_switch(4);
          cam_jishi+=20;
          Camera_Flag=0;
          runbeginflag=1;
        }
      }break;

    case 0X02:
      ui_floor--;
      ui_anjian=1;
      ui_var_line=1;
    break;


    default:break;
  }

}

void main_run(CAM_var *plan, CAM_run *speed , Fit_line *line,CAM_Inductance *indu)
{
  char txt[21];
  OLED_CLS();
  memcpy(&(plans[plansnum]),&(planc),sizeof(planc));
  UI_writeinflash();
  OLED_Write68(30,1,"_rc_",0);
  cam_jishi=0;
  runbeginflag=1;
  while(1)
  {
    if(Camera_Flag == 2)
    {
      Get_Cam_Process_Val(plan,line,indu);
  //     cam_getspeed(plan,speed);
      Camera_Flag =0;
      cam_jishi++;
      sprintf(txt,"time: %d ",cam_jishi);
      OLED_Write68(10,0,txt,1);
    }
  }
  return;
}


void UI_init(void)
{


  planc.Dst.p= 78 ;
  planc.Dst.d= 4 ;

  planc.D_middle=1550;

  planc.Dian.p=17;

  planc.k_nihe=1.9;
  planc.b_nihe=6;

  OLED_Init();
  delayms(30);
  OLED_CLS();
  delayms(10);
  rungo.judge_ramp=Ramp_judge;
  rungo.judge_ring=cam_Ring_judge;
  rungo.judge_finish=Finish_judge;
  rungo.judge_three=cam_three;
  rungo.judge_ringout=cam_Ringout_judge;
  runinfo.getimagin=getdajinima;
  runinfo.getinducrance=camgetIN;
  runstate.ring=cam_Ring_run;
  runstate.camp=camIN2pidcamp;
  runstate.Three=cam_Three_run;
  Three_dec.part_1=cam_judge_threemid;
  Three_dec.part_2=DFS_UPblack;
  Three_dec.part_3=cam_Three_judge_side;
  Three_dec.part_4=ISVUP_init;
  Three_dec.part_13=DFS_UPblack_side_step1;
  Three_dec.part_6=DFS_UPblack_side_step2;
  Three_dec.part_7=DFS_UPblack_side_step3;
  Three_dec.part_8=DFS_UPblack_side;
  Three_dec.part_9=threeDFSside;
  Three_dec.part_12=Three_outjudge;
  FMid_dec.part_4=ImageDfs;
  FMid_dec.part_5=DoFilter;
  FMid_dec.part_10=RepairBottomLine_left;
  FMid_dec.part_11=RepairBottomLine_right;
}



void UI_writeinflash(void)
{
    txpr[0]=0XFC;
    txpr[1]=0XAA;

    txpr[2]=(uint8)(plans[0].Dst.p>>8);
    txpr[3]=(uint8)(plans[0].Dst.p);
    txpr[4]=(uint8)(plans[0].Dst.d>>8);
    txpr[5]=(uint8)(plans[0].Dst.d);


    txpr[8]=plans[0].delay_Out;
    txpr[9]=plans[0].delay_In;
    txpr[10]=(uint8)plans[0].Outga_direction;

    memcpy(&(txpr[11]),&(plans[0].k_nihe),sizeof(float));
    memcpy(&(txpr[16]),&(plans[0].b_nihe),sizeof(float));

    txpr[21]=(uint8)(plans[0].D_max>>8);
    txpr[22]=(uint8)(plans[0].D_max);
    txpr[23]=(uint8)(plans[0].D_min>>8);
    txpr[24]=(uint8)(plans[0].D_min);
    txpr[25]=(uint8)(plans[0].D_middle>>8);
    txpr[26]=(uint8)(plans[0].D_middle);

    txpr[27]=(uint8)(plans[0].Ring_direction[0]);
    txpr[28]=(uint8)(plans[0].Ring_direction[1]);
    txpr[29]=(uint8)(plans[0].Ring_direction[2]);
    txpr[30]=(uint8)(plans[0].Ring_direction[3]);

    txpr[31]=(uint8)(plans[0].speedslow>>8);
    txpr[32]=(uint8)(plans[0].speedslow);

    memcpy(&(txpr[33]),&(plans[0].speedadd),sizeof(float));

    memcpy(&(txpr[38]),&(plans[0].speedD),sizeof(float));

    txpr[43]=(uint8)(plans[0].Ring_sumIN>>8);
    txpr[44]=(uint8)(plans[0].Ring_sumIN);

    txpr[45]=(uint8)(plans[0].Ramp_delay>>8);
    txpr[46]=(uint8)(plans[0].Ramp_delay);

    txpr[47]=plans[0].Ist.p;
    txpr[48]=plans[0].Ist.i;
    txpr[49]=plans[0].Ist.d;

    txpr[50]=(uint8)(Ind.Inleftmax>>8);
    txpr[51]=(uint8)(Ind.Inleftmax);
            txpr[52]=(uint8)(Ind.Inrightmax>>8);
            txpr[53]=(uint8)(Ind.Inrightmax);

            txpr[54]=plans[0].Icu.p;
            txpr[55]=plans[0].Icu.i;
            txpr[56]=plans[0].Icu.d;

            txpr[57]=(uint8)(plans[0].err_istcu>>8);
            txpr[58]=(uint8)(plans[0].err_istcu);

            txpr[59]=plans[0].befringindelay[0];
            txpr[60]=plans[0].befringindelay[1];
            txpr[61]=plans[0].befringindelay[2];
            txpr[62]=plans[0].befringindelay[3];

            txpr[63]=(uint8)(plans[0].bangbangplus>>8);
            txpr[64]=(uint8)(plans[0].bangbangplus);
            txpr[65]=(uint8)(plans[0].bangbangminus>>8);
            txpr[66]=(uint8)(plans[0].bangbangminus);

            txpr[67]=(uint8)(plans[0].bangpluspwm >>8);
            txpr[68]=(uint8)(plans[0].bangpluspwm);

            txpr[69]=(uint8)(plans[0].bangminuspwm >>8);
            txpr[70]=(uint8)(plans[0].bangminuspwm);

            txpr[71]=(uint8)(Ind.Inlevellmax>>8);
            txpr[72]=(uint8)(Ind.Inlevellmax);
            txpr[73]=(uint8)(Ind.Inlevelrmax>>8);
            txpr[74]=(uint8)(Ind.Inlevelrmax);

            txpr[75]=(uint8)(plans[0].Dian.p>>8);
            txpr[76]=(uint8)(plans[0].Dian.p);
            txpr[77]=(uint8)(plans[0].Dian.i>>8);
            txpr[78]=(uint8)(plans[0].Dian.i);
            txpr[79]=(uint8)(plans[0].Dian.d>>8);
            txpr[80]=(uint8)(plans[0].Dian.d);

            txpr[81]=(uint8)(plans[0].speed_camp>>8);
            txpr[82]=(uint8)(plans[0].speed_camp);

            txpr[83]=(uint8)(plans[0].delay_threeall>>8);
            txpr[84]=(uint8)(plans[0].delay_threeall);

            txpr[85]=(uint8)(plans[0].Three_direction[0]);
            txpr[86]=(uint8)(plans[0].Three_direction[1]);

            txpr[87]=(uint8)(plans[0].delay_threein0);
            txpr[88]=(uint8)(plans[0].delay_threeout0);

            txpr[89]=(uint8)(plans[0].Outga_direction);
            txpr[90]=(uint8)(plans[0].delay_outgarmid);

            txpr[91]=(uint8)(plans[0].errlinebegin);

            txpr[92]=(uint8)(plans[0].Ring_turnover>>8);
            txpr[93]=(uint8)(plans[0].Ring_turnover);

            txpr[94]=(uint8)(plans[0].Ring_speedin>>8);
            txpr[95]=(uint8)(plans[0].Ring_speedin);

            txpr[96]=(uint8)(plans[0].Ring_inmax>>8);
            txpr[97]=(uint8)(plans[0].Ring_inmax);

            txpr[98]=(uint8)(plans[0].Ring_over>>8);
            txpr[99]=(uint8)(plans[0].Ring_over);

            txpr[100]=(uint8)(plans[0].Three_speedin>>8);
           txpr[101]=(uint8)(plans[0].Three_speedin);

           memcpy(&(txpr[102]),&(plans[0].speederrp),sizeof(float));

           memcpy(&(txpr[107]),&(plans[0].speederrd),sizeof(float));

           txpr[113]=(uint8)(plans[0].Dcup>>8);
           txpr[114]=(uint8)(plans[0].Dcup);
           txpr[115]=(uint8)(plans[0].Dcud>>8);
           txpr[116]=(uint8)(plans[0].Dcud);

           txpr[117]=(uint8)(plans[0].errcamcust>>8);
           txpr[118]=(uint8)(plans[0].errcamcust);

           txpr[119]=(uint8)(plans[0].angle_Ring[0]>>8);
            txpr[120]=(uint8)(plans[0].angle_Ring[0]);

            txpr[121]=(uint8)(plans[0].angle_Ring[1]>>8);
           txpr[122]=(uint8)(plans[0].angle_Ring[1]);

           txpr[123]=(uint8)(plans[0].angle_Ring[2]>>8);
           txpr[124]=(uint8)(plans[0].angle_Ring[2]);

           txpr[125]=(uint8)(plans[0].angle_Ring[3]>>8);
          txpr[126]=(uint8)(plans[0].angle_Ring[3]);

          txpr[127]=(uint8)(plans[0].angle_Threein[0]>>8);
          txpr[128]=(uint8)(plans[0].angle_Threein[0]);

          txpr[129]=(uint8)(plans[0].angle_Threein[1]>>8);
          txpr[130]=(uint8)(plans[0].angle_Threein[1]);

          txpr[131]=(uint8)(plans[0].angle_Outgar>>8);
          txpr[132]=(uint8)(plans[0].angle_Outgar);

          txpr[133]=(uint8)(plans[0].angle_Ingar>>8);
          txpr[134]=(uint8)(plans[0].angle_Ingar);

          txpr[135]=(uint8)(Duo_limitplus>>8);
          txpr[136]=(uint8)(Duo_limitplus);

          txpr[137]=(uint8)(Duo_limitminus>>8);
         txpr[138]=(uint8)(Duo_limitminus);

         txpr[139]=(uint8)(plans[0].angle_Threeout[0]>>8);
         txpr[140]=(uint8)(plans[0].angle_Threeout[0]);

         txpr[141]=(uint8)(plans[0].angle_Threeout[1]>>8);
         txpr[142]=(uint8)(plans[0].angle_Threeout[1]);

         txpr[143]=(uint8)(plans[0].speed_Ring>>8);
          txpr[144]=(uint8)(plans[0].speed_Ring);

          txpr[145]=(uint8)(plans[0].speed_finishin);
          txpr[146]=(uint8)(plans[0].speed_finished);

          txpr[147]=(uint8)(plans[0].Three_speedout>>8);
          txpr[148]=(uint8)(plans[0].Three_speedout);

          txpr[149]=(uint8)plans[0].finished_time;
          txpr[150]=(uint8)plans[0].stop_sum;

          txpr[157]=(uint8)(plans[0].delay_threein1);
          txpr[158]=(uint8)(plans[0].delay_threeout1);
          txpr[159]=(uint8)(plans[0].stop_times);

          txpr[202]=(uint8)(plans[1].Dst.p>>8);
          txpr[203]=(uint8)(plans[1].Dst.p);
          txpr[204]=(uint8)(plans[1].Dst.d>>8);
          txpr[205]=(uint8)(plans[1].Dst.d);


          txpr[208]=plans[1].delay_Out;
          txpr[209]=plans[1].delay_In;
          txpr[210]=(uint8)plans[1].Outga_direction;

          memcpy(&(txpr[211]),&(plans[1].k_nihe),sizeof(float));
          memcpy(&(txpr[216]),&(plans[1].b_nihe),sizeof(float));

          txpr[221]=(uint8)(plans[1].D_max>>8);
          txpr[222]=(uint8)(plans[1].D_max);
          txpr[223]=(uint8)(plans[1].D_min>>8);
          txpr[224]=(uint8)(plans[1].D_min);
          txpr[225]=(uint8)(plans[1].D_middle>>8);
          txpr[226]=(uint8)(plans[1].D_middle);

          txpr[227]=(uint8)(plans[1].Ring_direction[0]);
          txpr[228]=(uint8)(plans[1].Ring_direction[1]);
          txpr[229]=(uint8)(plans[1].Ring_direction[2]);
          txpr[230]=(uint8)(plans[1].Ring_direction[3]);

          txpr[231]=(uint8)(plans[1].speedslow>>8);
          txpr[232]=(uint8)(plans[1].speedslow);

          memcpy(&(txpr[233]),&(plans[1].speedadd),sizeof(float));

          memcpy(&(txpr[238]),&(plans[1].speedD),sizeof(float));

          txpr[243]=(uint8)(plans[1].Ring_sumIN>>8);
          txpr[244]=(uint8)(plans[1].Ring_sumIN);

          txpr[245]=(uint8)(plans[1].Ramp_delay>>8);
          txpr[246]=(uint8)(plans[1].Ramp_delay);

          txpr[247]=plans[1].Ist.p;
          txpr[248]=plans[1].Ist.i;
          txpr[249]=plans[1].Ist.d;


          txpr[254]=plans[1].Icu.p;
          txpr[255]=plans[1].Icu.i;
          txpr[256]=plans[1].Icu.d;

          txpr[257]=(uint8)(plans[1].err_istcu>>8);
          txpr[258]=(uint8)(plans[1].err_istcu);

          txpr[259]=plans[1].befringindelay[0];
          txpr[260]=plans[1].befringindelay[1];
          txpr[261]=plans[1].befringindelay[2];
          txpr[262]=plans[1].befringindelay[3];

          txpr[263]=(uint8)(plans[1].bangbangplus>>8);
          txpr[264]=(uint8)(plans[1].bangbangplus);
          txpr[265]=(uint8)(plans[1].bangbangminus>>8);
          txpr[266]=(uint8)(plans[1].bangbangminus);

          txpr[267]=(uint8)(plans[1].bangpluspwm >>8);
          txpr[268]=(uint8)(plans[1].bangpluspwm);

          txpr[269]=(uint8)(plans[1].bangminuspwm >>8);
          txpr[270]=(uint8)(plans[1].bangminuspwm);

          txpr[275]=(uint8)(plans[1].Dian.p>>8);
          txpr[276]=(uint8)(plans[1].Dian.p);
          txpr[277]=(uint8)(plans[1].Dian.i>>8);
          txpr[278]=(uint8)(plans[1].Dian.i);
          txpr[279]=(uint8)(plans[1].Dian.d>>8);
          txpr[280]=(uint8)(plans[1].Dian.d);

          txpr[281]=(uint8)(plans[1].speed_camp>>8);
          txpr[282]=(uint8)(plans[1].speed_camp);

          txpr[283]=(uint8)(plans[1].delay_threeall>>8);
          txpr[284]=(uint8)(plans[1].delay_threeall);

          txpr[285]=(uint8)(plans[1].Three_direction[0]);
          txpr[286]=(uint8)(plans[1].Three_direction[1]);

          txpr[287]=(uint8)(plans[1].delay_threein0);
          txpr[288]=(uint8)(plans[1].delay_threeout0);

          txpr[289]=(uint8)(plans[1].Outga_direction);
          txpr[290]=(uint8)(plans[1].delay_outgarmid);

          txpr[291]=(uint8)(plans[1].errlinebegin);

          txpr[292]=(uint8)(plans[1].Ring_turnover>>8);
          txpr[293]=(uint8)(plans[1].Ring_turnover);

          txpr[294]=(uint8)(plans[1].Ring_speedin>>8);
          txpr[295]=(uint8)(plans[1].Ring_speedin);

          txpr[296]=(uint8)(plans[1].Ring_inmax>>8);
          txpr[297]=(uint8)(plans[1].Ring_inmax);

          txpr[298]=(uint8)(plans[1].Ring_over>>8);
          txpr[299]=(uint8)(plans[1].Ring_over);

          txpr[300]=(uint8)(plans[1].Three_speedin>>8);
          txpr[301]=(uint8)(plans[1].Three_speedin);

          memcpy(&(txpr[302]),&(plans[1].speederrp),sizeof(float));

          memcpy(&(txpr[307]),&(plans[1].speederrd),sizeof(float));

          txpr[313]=(uint8)(plans[1].Dcup>>8);
         txpr[314]=(uint8)(plans[1].Dcup);
         txpr[315]=(uint8)(plans[1].Dcud>>8);
         txpr[316]=(uint8)(plans[1].Dcud);

         txpr[317]=(uint8)(plans[1].errcamcust>>8);
          txpr[318]=(uint8)(plans[1].errcamcust);

          txpr[319]=(uint8)(plans[1].angle_Ring[0]>>8);
          txpr[320]=(uint8)(plans[1].angle_Ring[0]);

          txpr[321]=(uint8)(plans[1].angle_Ring[1]>>8);
          txpr[322]=(uint8)(plans[1].angle_Ring[1]);

          txpr[323]=(uint8)(plans[1].angle_Ring[2]>>8);
         txpr[324]=(uint8)(plans[1].angle_Ring[2]);

         txpr[325]=(uint8)(plans[1].angle_Ring[3]>>8);
         txpr[326]=(uint8)(plans[1].angle_Ring[3]);

         txpr[327]=(uint8)(plans[1].angle_Threein[0]>>8);
         txpr[328]=(uint8)(plans[1].angle_Threein[0]);

         txpr[329]=(uint8)(plans[1].angle_Threein[1]>>8);
         txpr[330]=(uint8)(plans[1].angle_Threein[1]);

         txpr[331]=(uint8)(plans[1].angle_Outgar>>8);
         txpr[332]=(uint8)(plans[1].angle_Outgar);

         txpr[333]=(uint8)(plans[1].angle_Ingar>>8);
        txpr[334]=(uint8)(plans[1].angle_Ingar);



        txpr[339]=(uint8)(plans[1].angle_Threeout[0]>>8);
        txpr[340]=(uint8)(plans[1].angle_Threeout[0]);

        txpr[341]=(uint8)(plans[1].angle_Threeout[1]>>8);
        txpr[342]=(uint8)(plans[1].angle_Threeout[1]);

        txpr[343]=(uint8)(plans[1].speed_Ring>>8);
         txpr[344]=(uint8)(plans[1].speed_Ring);

         txpr[345]=(uint8)(plans[1].speed_finishin);
         txpr[346]=(uint8)(plans[1].speed_finished);


         txpr[347]=(uint8)(plans[1].Three_speedout>>8);
         txpr[348]=(uint8)(plans[1].Three_speedout);


         txpr[349]=(uint8)plans[1].finished_time;
         txpr[350]=(uint8)plans[1].stop_sum;

         txpr[357]=(uint8)(plans[1].delay_threein1);
         txpr[358]=(uint8)(plans[1].delay_threeout1);
         txpr[359]=(uint8)(plans[1].stop_times);


         txpr[402]=(uint8)(plans[2].Dst.p>>8);
         txpr[403]=(uint8)(plans[2].Dst.p);
         txpr[404]=(uint8)(plans[2].Dst.d>>8);
         txpr[405]=(uint8)(plans[2].Dst.d);


         txpr[408]=plans[2].delay_Out;
         txpr[409]=plans[2].delay_In;
         txpr[410]=(uint8)plans[2].Outga_direction;

         memcpy(&(txpr[411]),&(plans[2].k_nihe),sizeof(float));
         memcpy(&(txpr[416]),&(plans[2].b_nihe),sizeof(float));

         txpr[421]=(uint8)(plans[2].D_max>>8);
         txpr[422]=(uint8)(plans[2].D_max);
         txpr[423]=(uint8)(plans[2].D_min>>8);
         txpr[424]=(uint8)(plans[2].D_min);
         txpr[425]=(uint8)(plans[2].D_middle>>8);
         txpr[426]=(uint8)(plans[2].D_middle);

         txpr[427]=(uint8)(plans[2].Ring_direction[0]);
         txpr[428]=(uint8)(plans[2].Ring_direction[1]);
         txpr[429]=(uint8)(plans[2].Ring_direction[2]);
         txpr[430]=(uint8)(plans[2].Ring_direction[3]);

         txpr[431]=(uint8)(plans[2].speedslow>>8);
         txpr[432]=(uint8)(plans[2].speedslow);

         memcpy(&(txpr[433]),&(plans[2].speedadd),sizeof(float));

         memcpy(&(txpr[438]),&(plans[2].speedD),sizeof(float));

         txpr[443]=(uint8)(plans[2].Ring_sumIN>>8);
         txpr[444]=(uint8)(plans[2].Ring_sumIN);

         txpr[445]=(uint8)(plans[2].Ramp_delay>>8);
         txpr[446]=(uint8)(plans[2].Ramp_delay);

         txpr[447]=plans[2].Ist.p;
         txpr[448]=plans[2].Ist.i;
         txpr[449]=plans[2].Ist.d;


         txpr[454]=plans[2].Icu.p;
         txpr[455]=plans[2].Icu.i;
         txpr[456]=plans[2].Icu.d;

         txpr[457]=(uint8)(plans[2].err_istcu>>8);
         txpr[458]=(uint8)(plans[2].err_istcu);

         txpr[459]=plans[2].befringindelay[0];
         txpr[460]=plans[2].befringindelay[1];
         txpr[461]=plans[2].befringindelay[2];
         txpr[462]=plans[2].befringindelay[3];

         txpr[463]=(uint8)(plans[2].bangbangplus>>8);
         txpr[464]=(uint8)(plans[2].bangbangplus);
         txpr[465]=(uint8)(plans[2].bangbangminus>>8);
         txpr[466]=(uint8)(plans[2].bangbangminus);

         txpr[467]=(uint8)(plans[2].bangpluspwm >>8);
         txpr[468]=(uint8)(plans[2].bangpluspwm);

         txpr[469]=(uint8)(plans[2].bangminuspwm >>8);
         txpr[470]=(uint8)(plans[2].bangminuspwm);


         txpr[475]=(uint8)(plans[2].Dian.p>>8);
         txpr[476]=(uint8)(plans[2].Dian.p);
         txpr[477]=(uint8)(plans[2].Dian.i>>8);
         txpr[478]=(uint8)(plans[2].Dian.i);
         txpr[479]=(uint8)(plans[2].Dian.d>>8);
         txpr[480]=(uint8)(plans[2].Dian.d);

         txpr[481]=(uint8)(plans[2].speed_camp>>8);
         txpr[482]=(uint8)(plans[2].speed_camp);

         txpr[483]=(uint8)(plans[2].delay_threeall>>8);
         txpr[484]=(uint8)(plans[2].delay_threeall);

         txpr[485]=(uint8)(plans[2].Three_direction[0]);
         txpr[486]=(uint8)(plans[2].Three_direction[1]);

         txpr[487]=(uint8)(plans[2].delay_threein0);
         txpr[488]=(uint8)(plans[2].delay_threeout0);

         txpr[489]=(uint8)(plans[2].Outga_direction);
         txpr[490]=(uint8)(plans[2].delay_outgarmid);

         txpr[491]=(uint8)(plans[2].errlinebegin);

         txpr[492]=(uint8)(plans[2].Ring_turnover>>8);
         txpr[493]=(uint8)(plans[2].Ring_turnover);

         txpr[494]=(uint8)(plans[2].Ring_speedin>>8);
         txpr[495]=(uint8)(plans[2].Ring_speedin);

         txpr[496]=(uint8)(plans[2].Ring_inmax>>8);
         txpr[497]=(uint8)(plans[2].Ring_inmax);

         txpr[498]=(uint8)(plans[2].Ring_over>>8);
         txpr[499]=(uint8)(plans[2].Ring_over);

         txpr[500]=(uint8)(plans[2].Three_speedin>>8);
        txpr[501]=(uint8)(plans[2].Three_speedin);

        memcpy(&(txpr[502]),&(plans[2].speederrp),sizeof(float));

        memcpy(&(txpr[507]),&(plans[2].speederrd),sizeof(float));

        txpr[513]=(uint8)(plans[2].Dcup>>8);
        txpr[514]=(uint8)(plans[2].Dcup);
        txpr[515]=(uint8)(plans[2].Dcud>>8);
        txpr[516]=(uint8)(plans[2].Dcud);

        txpr[517]=(uint8)(plans[2].errcamcust>>8);
        txpr[518]=(uint8)(plans[2].errcamcust);

        txpr[519]=(uint8)(plans[2].angle_Ring[0]>>8);
         txpr[520]=(uint8)(plans[2].angle_Ring[0]);

         txpr[521]=(uint8)(plans[2].angle_Ring[1]>>8);
        txpr[522]=(uint8)(plans[2].angle_Ring[1]);

        txpr[523]=(uint8)(plans[2].angle_Ring[2]>>8);
        txpr[524]=(uint8)(plans[2].angle_Ring[2]);

        txpr[525]=(uint8)(plans[2].angle_Ring[3]>>8);
        txpr[526]=(uint8)(plans[2].angle_Ring[3]);

        txpr[527]=(uint8)(plans[2].angle_Threein[0]>>8);
        txpr[528]=(uint8)(plans[2].angle_Threein[0]);

        txpr[529]=(uint8)(plans[2].angle_Threein[1]>>8);
        txpr[530]=(uint8)(plans[2].angle_Threein[1]);

        txpr[531]=(uint8)(plans[2].angle_Outgar>>8);
        txpr[532]=(uint8)(plans[2].angle_Outgar);

        txpr[533]=(uint8)(plans[2].angle_Ingar>>8);
        txpr[534]=(uint8)(plans[2].angle_Ingar);


        txpr[539]=(uint8)(plans[2].angle_Threeout[0]>>8);
        txpr[540]=(uint8)(plans[2].angle_Threeout[0]);

        txpr[541]=(uint8)(plans[2].angle_Threeout[1]>>8);
        txpr[542]=(uint8)(plans[2].angle_Threeout[1]);

        txpr[543]=(uint8)(plans[2].speed_Ring>>8);
        txpr[544]=(uint8)(plans[2].speed_Ring);

        txpr[545]=(uint8)(plans[2].speed_finishin);
        txpr[546]=(uint8)(plans[2].speed_finished);

        txpr[547]=(uint8)(plans[2].Three_speedout>>8);
        txpr[548]=(uint8)(plans[2].Three_speedout);
        txpr[549]=(uint8)plans[2].finished_time;
                txpr[550]=(uint8)plans[2].stop_sum;

                txpr[557]=(uint8)(plans[2].delay_threein1);
          txpr[558]=(uint8)(plans[2].delay_threeout1);
          txpr[559]=(uint8)(plans[2].stop_times);


        txpr[602]=(uint8)(plans[3].Dst.p>>8);
        txpr[603]=(uint8)(plans[3].Dst.p);
        txpr[604]=(uint8)(plans[3].Dst.d>>8);
        txpr[605]=(uint8)(plans[3].Dst.d);


        txpr[608]=plans[3].delay_Out;
        txpr[609]=plans[3].delay_In;
        txpr[610]=(uint8)plans[3].Outga_direction;

        memcpy(&(txpr[611]),&(plans[3].k_nihe),sizeof(float));
        memcpy(&(txpr[616]),&(plans[3].b_nihe),sizeof(float));

        txpr[621]=(uint8)(plans[3].D_max>>8);
        txpr[622]=(uint8)(plans[3].D_max);
        txpr[623]=(uint8)(plans[3].D_min>>8);
        txpr[624]=(uint8)(plans[3].D_min);
        txpr[625]=(uint8)(plans[3].D_middle>>8);
        txpr[626]=(uint8)(plans[3].D_middle);

        txpr[627]=(uint8)(plans[3].Ring_direction[0]);
        txpr[628]=(uint8)(plans[3].Ring_direction[1]);
        txpr[629]=(uint8)(plans[3].Ring_direction[2]);
        txpr[630]=(uint8)(plans[3].Ring_direction[3]);

        txpr[631]=(uint8)(plans[3].speedslow>>8);
        txpr[632]=(uint8)(plans[3].speedslow);

        memcpy(&(txpr[633]),&(plans[3].speedadd),sizeof(float));

        memcpy(&(txpr[638]),&(plans[3].speedD),sizeof(float));

        txpr[643]=(uint8)(plans[3].Ring_sumIN>>8);
        txpr[644]=(uint8)(plans[3].Ring_sumIN);

        txpr[645]=(uint8)(plans[3].Ramp_delay>>8);
        txpr[646]=(uint8)(plans[3].Ramp_delay);

        txpr[647]=plans[3].Ist.p;
        txpr[648]=plans[3].Ist.i;
        txpr[649]=plans[3].Ist.d;

        txpr[654]=plans[3].Icu.p;
        txpr[655]=plans[3].Icu.i;
        txpr[656]=plans[3].Icu.d;

        txpr[657]=(uint8)(plans[3].err_istcu>>8);
        txpr[658]=(uint8)(plans[3].err_istcu);

        txpr[659]=plans[3].befringindelay[0];
        txpr[660]=plans[3].befringindelay[1];
        txpr[661]=plans[3].befringindelay[2];
        txpr[662]=plans[3].befringindelay[3];

        txpr[663]=(uint8)(plans[3].bangbangplus>>8);
        txpr[664]=(uint8)(plans[3].bangbangplus);
        txpr[665]=(uint8)(plans[3].bangbangminus>>8);
        txpr[666]=(uint8)(plans[3].bangbangminus);

        txpr[667]=(uint8)(plans[3].bangpluspwm >>8);
        txpr[668]=(uint8)(plans[3].bangpluspwm);

        txpr[669]=(uint8)(plans[3].bangminuspwm >>8);
        txpr[670]=(uint8)(plans[3].bangminuspwm);


        txpr[675]=(uint8)(plans[3].Dian.p>>8);
        txpr[676]=(uint8)(plans[3].Dian.p);
        txpr[677]=(uint8)(plans[3].Dian.i>>8);
        txpr[678]=(uint8)(plans[3].Dian.i);
        txpr[679]=(uint8)(plans[3].Dian.d>>8);
        txpr[680]=(uint8)(plans[3].Dian.d);

        txpr[681]=(uint8)(plans[3].speed_camp>>8);
        txpr[682]=(uint8)(plans[3].speed_camp);

        txpr[683]=(uint8)(plans[3].delay_threeall>>8);
        txpr[684]=(uint8)(plans[3].delay_threeall);

        txpr[685]=(uint8)(plans[3].Three_direction[0]);
        txpr[686]=(uint8)(plans[3].Three_direction[1]);

        txpr[687]=(uint8)(plans[3].delay_threein0);
        txpr[688]=(uint8)(plans[3].delay_threeout0);

        txpr[689]=(uint8)(plans[3].Outga_direction);
        txpr[690]=(uint8)(plans[3].delay_outgarmid);

        txpr[691]=(uint8)(plans[3].errlinebegin);

        txpr[692]=(uint8)(plans[3].Ring_turnover>>8);
        txpr[693]=(uint8)(plans[3].Ring_turnover);

        txpr[694]=(uint8)(plans[3].Ring_speedin>>8);
        txpr[695]=(uint8)(plans[3].Ring_speedin);

        txpr[696]=(uint8)(plans[3].Ring_inmax>>8);
        txpr[697]=(uint8)(plans[3].Ring_inmax);

        txpr[698]=(uint8)(plans[3].Ring_over>>8);
        txpr[699]=(uint8)(plans[3].Ring_over);

        txpr[700]=(uint8)(plans[3].Three_speedin>>8);
        txpr[701]=(uint8)(plans[3].Three_speedin);

        memcpy(&(txpr[702]),&(plans[3].speederrp),sizeof(float));

        memcpy(&(txpr[707]),&(plans[3].speederrd),sizeof(float));

        txpr[713]=(uint8)(plans[3].Dcup>>8);
        txpr[714]=(uint8)(plans[3].Dcup);
        txpr[715]=(uint8)(plans[3].Dcud>>8);
        txpr[716]=(uint8)(plans[3].Dcud);

        txpr[717]=(uint8)(plans[3].errcamcust>>8);
        txpr[718]=(uint8)(plans[3].errcamcust);

        txpr[719]=(uint8)(plans[3].angle_Ring[0]>>8);
        txpr[720]=(uint8)(plans[3].angle_Ring[0]);

        txpr[721]=(uint8)(plans[3].angle_Ring[1]>>8);
        txpr[722]=(uint8)(plans[3].angle_Ring[1]);

        txpr[723]=(uint8)(plans[3].angle_Ring[2]>>8);
        txpr[724]=(uint8)(plans[3].angle_Ring[2]);

        txpr[725]=(uint8)(plans[3].angle_Ring[3]>>8);
        txpr[726]=(uint8)(plans[3].angle_Ring[3]);

        txpr[727]=(uint8)(plans[3].angle_Threein[0]>>8);
        txpr[728]=(uint8)(plans[3].angle_Threein[0]);

        txpr[729]=(uint8)(plans[3].angle_Threein[1]>>8);
        txpr[730]=(uint8)(plans[3].angle_Threein[1]);

        txpr[731]=(uint8)(plans[3].angle_Outgar>>8);
        txpr[732]=(uint8)(plans[3].angle_Outgar);

        txpr[733]=(uint8)(plans[3].angle_Ingar>>8);
        txpr[734]=(uint8)(plans[3].angle_Ingar);



        txpr[739]=(uint8)(plans[3].angle_Threeout[0]>>8);
        txpr[740]=(uint8)(plans[3].angle_Threeout[0]);

        txpr[741]=(uint8)(plans[3].angle_Threeout[1]>>8);
        txpr[742]=(uint8)(plans[3].angle_Threeout[1]);

        txpr[743]=(uint8)(plans[3].speed_Ring>>8);
        txpr[744]=(uint8)(plans[3].speed_Ring);

        txpr[745]=(uint8)(plans[3].speed_finishin);
        txpr[746]=(uint8)(plans[3].speed_finished);

        txpr[747]=(uint8)(plans[3].Three_speedout>>8);
        txpr[748]=(uint8)(plans[3].Three_speedout);

        txpr[749]=(uint8)plans[3].finished_time;
                txpr[750]=(uint8)plans[3].stop_sum;

        txpr[757]=(uint8)(plans[3].delay_threein1);
        txpr[758]=(uint8)(plans[3].delay_threeout1);
        txpr[759]=(uint8)(plans[3].stop_times);

        txpr[900]=0XAA;


          W25QXX_Write(txpr,1048576,1000);


        }


        void cam_Test_Inductance(uint8 KEY)
        {
          char txt[22];

          if((KEY&key_out)==key_out)
          {
            ui_floor--;
            ui_anjian=1;
            ui_var_line=1;
            ui_In_floor=1;
          }
          else if(((KEY&key_in)==key_in)||ui_In_floor==2)
          {
            if(ui_In_floor!=2)
            {
  Ind.Inleftmax =10;
  Ind.Inlevellmax=10;
  Ind.Inlevelrmax=10;
  Ind.Inrightmax=10;

            }
            ui_In_floor=2;
            Ind.Inleft= ADC_ReadAverage(Inleft_IO,10);
            Ind.Inlevell=ADC_ReadAverage(Inlevell_IO,10);
            Ind.Inlevelr=ADC_ReadAverage(Inlevelr_IO,10);
            Ind.Inright=ADC_ReadAverage(Inright_IO,10);

            Ind.Inleft = (Ind.Inleft>1 )?Ind.Inleft :1;
            Ind.Inlevell= (Ind.Inlevell>1)?Ind.Inlevell :1 ;
            Ind.Inlevelr= (Ind.Inlevelr>1)?Ind.Inlevelr :1 ;
            Ind.Inright= (Ind.Inright>1)?Ind.Inright :1 ;

            Ind.Inleftmax =(Ind.Inleft>Ind.Inleftmax) ? Ind.Inleft:Ind.Inleftmax;
            Ind.Inlevellmax=(Ind.Inlevell>Ind.Inlevellmax)?Ind.Inlevell:Ind.Inlevellmax;
            Ind.Inlevelrmax=(Ind.Inlevelr>Ind.Inlevelrmax)?Ind.Inlevelr:Ind.Inlevelrmax;
            Ind.Inrightmax=(Ind.Inright>Ind.Inrightmax)?Ind.Inright:Ind.Inrightmax;


            sprintf(txt," left: %04d %04d ",Ind.Inleft,Ind.Inleftmax);
            OLED_P6x8Str(2,0,txt);
            sprintf(txt," Levl: %04d %04d ",Ind.Inlevell,Ind.Inlevellmax);
            OLED_P6x8Str(2,1,txt);
            sprintf(txt," Levr: %04d %04d ",Ind.Inlevelr,Ind.Inlevelrmax);
            OLED_P6x8Str(2,2,txt);
            sprintf(txt," righ: %04d %04d ",Ind.Inright,Ind.Inrightmax);
            OLED_P6x8Str(2,3,txt);
            OLED_P6x8Str(2,4,"          ");
            OLED_P6x8Str(2,5," Getting Induct max ");

  }
  else
  {
    Ind.Inleft= ADC_ReadAverage(Inleft_IO,8);
    Ind.Inlevell=ADC_ReadAverage(Inlevell_IO,8);
    Ind.Inlevelr=ADC_ReadAverage(Inlevelr_IO,8);
    Ind.Inright=ADC_ReadAverage(Inright_IO,8);
    Ind.Invertical=ADC_ReadAverage(Invertic_IO,8);

    Ind.Inleft = (Ind.Inleft>1 )?Ind.Inleft :1;
    Ind.Inlevell= (Ind.Inlevell>1)?Ind.Inlevell :1 ;
    Ind.Inlevelr= (Ind.Inlevelr>1)?Ind.Inlevelr :1 ;
    Ind.Inright= (Ind.Inright>1)?Ind.Inright :1 ;
    Ind.Invertical=(Ind.Invertical>1)?Ind.Invertical:1;

    sprintf(txt," left :  %04d  ",Ind.Inleft);
    OLED_P6x8Str(2,0,txt);
    sprintf(txt," Levl :   %04d  ",Ind.Inlevell);
    OLED_P6x8Str(2,1,txt);
    sprintf(txt," Levr :   %04d  ",Ind.Inlevelr);
    OLED_P6x8Str(2,3,txt);
    sprintf(txt," right :  %04d  ",Ind.Inright);
    OLED_P6x8Str(2,4,txt);
    sprintf(txt," Vertic:  %04d  ",Ind.Invertical);
    OLED_P6x8Str(2,2,txt);

 //   OLED_P6x8Str(2,4,"          ");
    OLED_P6x8Str(2,5," in: renew in's max");
    OLED_P6x8Str(2,6,"          ");

  }
}



void Flash_data_change(void)
{

  ////如果Flash_BREAK的话，就把0改成1再下一次，然后等屏幕显示finished后(等待时间会有一点长) //TODO TODO
  ////再把1改成0 再下一次就欧克了 //TODO TODO

  if(0)
  {
    W25QXX_Erase_Chip();
    Toplancvar();
    UI_writeinflash();
  }
  else
  {
    UI_readfromflash();
  }
}

void UI_readfromflash(void)
{
  W25QXX_Read(rxpr,1048576,1000);

  if(rxpr[0]!=0XFC || rxpr[1]!=0XAA || rxpr[900]!=0XAA)
  {
    OLED_CLS();
    OLED_P6x8Str(6,2," FLASH_BREAK ");
    OLED_P6x8Str(2,3,"IN:DOWNLOAD_before");

    while(1)
    {
      if(PIN_Read(key_in_IO)==0) break;
    }
    W25QXX_Erase_Chip();
    Toplancvar();
    UI_writeinflash();
  }

  else
  {
    W25QXX_Write(rxpr,1048576,1000);

    plans[0].Dst.p=((unsigned short)rxpr[2]<<8)+rxpr[3];
    plans[0].Dst.d=((unsigned short)rxpr[4]<<8)+rxpr[5];

    plans[0].delay_Out=rxpr[8];
    plans[0].delay_In=rxpr[9];
    plans[0].Outga_direction=rxpr[10];


    memcpy(&plans[0].k_nihe,&(rxpr[11]),sizeof(float));
    memcpy(&plans[0].b_nihe,&(rxpr[16]),sizeof(float));

    plans[0].D_max=(((unsigned short)rxpr[21])<<8)+rxpr[22];
    plans[0].D_min=(((unsigned short)rxpr[23])<<8)+rxpr[24];
    plans[0].D_middle=(((unsigned short)rxpr[25])<<8)+rxpr[26];


    plans[0].Ring_direction[0]=rxpr[27];
    plans[0].Ring_direction[1]=rxpr[28];
    plans[0].Ring_direction[2]=rxpr[29];
    plans[0].Ring_direction[3]=rxpr[30];

    plans[0].speedslow=(((unsigned short)rxpr[31])<<8)+rxpr[32];
    memcpy(&(plans[0].speedadd),&(rxpr[33]),sizeof(float));

    memcpy(&(plans[0].speedD),&(rxpr[38]),sizeof(float));

    plans[0].Ring_sumIN=((unsigned short)rxpr[43]<<8)+rxpr[44];

    plans[0].Ramp_delay=((unsigned short)rxpr[45]<<8)+rxpr[46];

    plans[0].Ist.p=rxpr[47];
    plans[0].Ist.i=rxpr[48];
    plans[0].Ist.d=rxpr[49];

    Ind.Inleftmax=((unsigned short)rxpr[50]<<8)+rxpr[51];
    Ind.Inrightmax=((unsigned short)rxpr[52]<<8)+rxpr[53];

    plans[0].Icu.p=rxpr[54];
    plans[0].Icu.i=rxpr[55];
    plans[0].Icu.d=rxpr[56];

    plans[0].err_istcu=((unsigned short)rxpr[57]<<8)+rxpr[58];

    plans[0].befringindelay[0]=rxpr[59];
    plans[0].befringindelay[1]=rxpr[60];
    plans[0].befringindelay[2]=rxpr[61];
    plans[0].befringindelay[3]=rxpr[62];

    plans[0].bangbangplus=((unsigned short)rxpr[63]<<8)+rxpr[64];
    plans[0].bangbangminus=((unsigned short)rxpr[65]<<8)+rxpr[66];

    plans[0].bangpluspwm=((unsigned short)rxpr[67]<<8)+rxpr[68];
    plans[0].bangminuspwm=((unsigned short)rxpr[69]<<8)+rxpr[70];

    Ind.Inlevellmax=((unsigned short)rxpr[71]<<8)+rxpr[72];
    Ind.Inlevelrmax=((unsigned short)rxpr[73]<<8)+rxpr[74];

    plans[0].Dian.p=((unsigned short)rxpr[75]<<8)+rxpr[76];
    plans[0].Dian.i=((unsigned short)rxpr[77]<<8)+rxpr[78];
    plans[0].Dian.d=((unsigned short)rxpr[79]<<8)+rxpr[80];

    plans[0].speed_camp=((unsigned short)rxpr[81]<<8)+rxpr[82];

    plans[0].delay_threeall=((unsigned short)rxpr[83]<<8)+rxpr[84];

    plans[0].Three_direction[0]=rxpr[85];
    plans[0].Three_direction[1]=rxpr[86];

    plans[0].delay_threein0=rxpr[87];

    plans[0].delay_threeout0=rxpr[88];

    plans[0].Outga_direction=rxpr[89];

    plans[0].delay_outgarmid=rxpr[90];

    plans[0].errlinebegin=rxpr[91];

    plans[0].Ring_turnover=((unsigned short)rxpr[92]<<8)+rxpr[93];

    plans[0].Ring_speedin=((unsigned short)rxpr[94]<<8)+rxpr[95];

    plans[0].Ring_inmax=((unsigned short)rxpr[96]<<8)+rxpr[97];

    plans[0].Ring_over=((unsigned short)rxpr[98]<<8)+rxpr[99];

    plans[0].Three_speedin=((unsigned short)rxpr[100]<<8)+rxpr[101];

    memcpy(&(plans[0].speederrp) ,&(rxpr[102]),sizeof(float));
    memcpy(&(plans[0].speederrd) ,&(rxpr[107]),sizeof(float));

    plans[0].Dcup=((unsigned short)rxpr[113]<<8)+rxpr[114];
    plans[0].Dcud=((unsigned short)rxpr[115]<<8)+rxpr[116];

    plans[0].errcamcust=((unsigned short)rxpr[117]<<8)+rxpr[118];

    plans[0].angle_Ring[0]=((unsigned short)rxpr[119]<<8)+rxpr[120];
    plans[0].angle_Ring[1]=((unsigned short)rxpr[121]<<8)+rxpr[122];
    plans[0].angle_Ring[2]=((unsigned short)rxpr[123]<<8)+rxpr[124];
    plans[0].angle_Ring[3]=((unsigned short)rxpr[125]<<8)+rxpr[126];

    plans[0].angle_Threein[0]=((unsigned short)rxpr[127]<<8)+rxpr[128];
    plans[0].angle_Threein[1]=((unsigned short)rxpr[129]<<8)+rxpr[130];

    plans[0].angle_Outgar=((unsigned short)rxpr[131]<<8)+rxpr[132];
    plans[0].angle_Ingar=((unsigned short)rxpr[133]<<8)+rxpr[134];

    Duo_limitplus=((unsigned short)rxpr[135]<<8)+rxpr[136];
    Duo_limitminus=((unsigned short)rxpr[137]<<8)+rxpr[138];

    plans[0].angle_Threeout[0]=((unsigned short)rxpr[139]<<8)+rxpr[140];
    plans[0].angle_Threeout[1]=((unsigned short)rxpr[141]<<8)+rxpr[142];

    plans[0].speed_Ring=((unsigned short)rxpr[143]<<8)+rxpr[144];

    plans[0].speed_finishin=rxpr[145];
    plans[0].speed_finished=rxpr[146];

    plans[0].Three_speedout=((unsigned short)rxpr[147]<<8)+rxpr[148];

    plans[0].finished_time=rxpr[149];
    plans[0].stop_sum=rxpr[150];

    plans[0].delay_threein1=rxpr[157];
    plans[0].delay_threeout1=rxpr[158];
    plans[0].stop_times=rxpr[159];

        plans[1].Dst.p=((unsigned short)rxpr[202]<<8)+rxpr[203];
        plans[1].Dst.d=((unsigned short)rxpr[204]<<8)+rxpr[205];

        plans[1].delay_Out=rxpr[208];
        plans[1].delay_In=rxpr[209];
        plans[1].Outga_direction=rxpr[210];


        memcpy(&plans[1].k_nihe,&(rxpr[211]),sizeof(float));
        memcpy(&plans[1].b_nihe,&(rxpr[216]),sizeof(float));

        plans[1].D_max=(((unsigned short)rxpr[221])<<8)+rxpr[222];
        plans[1].D_min=(((unsigned short)rxpr[223])<<8)+rxpr[224];
        plans[1].D_middle=(((unsigned short)rxpr[225])<<8)+rxpr[226];


        plans[1].Ring_direction[0]=rxpr[227];
        plans[1].Ring_direction[1]=rxpr[228];
        plans[1].Ring_direction[2]=rxpr[229];
        plans[1].Ring_direction[3]=rxpr[230];

        plans[1].speedslow=(((unsigned short)rxpr[231])<<8)+rxpr[232];
        memcpy(&(plans[1].speedadd),&(rxpr[233]),sizeof(float));

        memcpy(&(plans[1].speedD),&(rxpr[238]),sizeof(float));

        plans[1].Ring_sumIN=((unsigned short)rxpr[243]<<8)+rxpr[244];

        plans[1].Ramp_delay=((unsigned short)rxpr[245]<<8)+rxpr[246];

        plans[1].Ist.p=rxpr[247];
        plans[1].Ist.i=rxpr[248];
        plans[1].Ist.d=rxpr[249];


        plans[1].Icu.p=rxpr[254];
        plans[1].Icu.i=rxpr[255];
        plans[1].Icu.d=rxpr[256];

        plans[1].err_istcu=((unsigned short)rxpr[257]<<8)+rxpr[258];

        plans[1].befringindelay[0]=rxpr[259];
        plans[1].befringindelay[1]=rxpr[260];
        plans[1].befringindelay[2]=rxpr[261];
        plans[1].befringindelay[3]=rxpr[262];

        plans[1].bangbangplus=((unsigned short)rxpr[263]<<8)+rxpr[264];
        plans[1].bangbangminus=((unsigned short)rxpr[265]<<8)+rxpr[266];

        plans[1].bangpluspwm=((unsigned short)rxpr[267]<<8)+rxpr[268];
        plans[1].bangminuspwm=((unsigned short)rxpr[269]<<8)+rxpr[270];



        plans[1].Dian.p=((unsigned short)rxpr[275]<<8)+rxpr[276];
        plans[1].Dian.i=((unsigned short)rxpr[277]<<8)+rxpr[278];
        plans[1].Dian.d=((unsigned short)rxpr[279]<<8)+rxpr[280];

        plans[1].speed_camp=((unsigned short)rxpr[281]<<8)+rxpr[282];

        plans[1].delay_threeall=((unsigned short)rxpr[283]<<8)+rxpr[284];

        plans[1].Three_direction[0]=rxpr[285];
        plans[1].Three_direction[1]=rxpr[286];

        plans[1].delay_threein0=rxpr[287];

        plans[1].delay_threeout0=rxpr[288];

        plans[1].Outga_direction=rxpr[289];

        plans[1].delay_outgarmid=rxpr[290];

        plans[1].errlinebegin=rxpr[291];

        plans[1].Ring_turnover=((unsigned short)rxpr[292]<<8)+rxpr[293];

        plans[1].Ring_speedin=((unsigned short)rxpr[294]<<8)+rxpr[295];

        plans[1].Ring_inmax=((unsigned short)rxpr[296]<<8)+rxpr[297];

        plans[1].Ring_over=((unsigned short)rxpr[298]<<8)+rxpr[299];

        plans[1].Three_speedin=((unsigned short)rxpr[300]<<8)+rxpr[301];

        memcpy(&(plans[1].speederrp) ,&(rxpr[302]),sizeof(float));
        memcpy(&(plans[1].speederrd) ,&(rxpr[307]),sizeof(float));

        plans[1].Dcup=((unsigned short)rxpr[313]<<8)+rxpr[314];
        plans[1].Dcud=((unsigned short)rxpr[315]<<8)+rxpr[316];

        plans[1].errcamcust=((unsigned short)rxpr[317]<<8)+rxpr[318];

        plans[1].angle_Ring[0]=((unsigned short)rxpr[319]<<8)+rxpr[320];
        plans[1].angle_Ring[1]=((unsigned short)rxpr[321]<<8)+rxpr[322];
        plans[1].angle_Ring[2]=((unsigned short)rxpr[323]<<8)+rxpr[324];
        plans[1].angle_Ring[3]=((unsigned short)rxpr[325]<<8)+rxpr[326];

        plans[1].angle_Threein[0]=((unsigned short)rxpr[327]<<8)+rxpr[328];
        plans[1].angle_Threein[1]=((unsigned short)rxpr[329]<<8)+rxpr[330];

        plans[1].angle_Outgar=((unsigned short)rxpr[331]<<8)+rxpr[332];
        plans[1].angle_Ingar=((unsigned short)rxpr[333]<<8)+rxpr[334];

        plans[1].angle_Threeout[0]=((unsigned short)rxpr[339]<<8)+rxpr[340];
        plans[1].angle_Threeout[1]=((unsigned short)rxpr[341]<<8)+rxpr[342];

        plans[1].speed_Ring=((unsigned short)rxpr[343]<<8)+rxpr[344];

        plans[1].speed_finishin=rxpr[345];
        plans[1].speed_finished=rxpr[346];

        plans[1].Three_speedout=((unsigned short)rxpr[347]<<8)+rxpr[348];

        plans[1].finished_time=rxpr[349];
         plans[1].stop_sum=rxpr[350];

         plans[1].delay_threein1=rxpr[357];
         plans[1].delay_threeout1=rxpr[358];
         plans[1].stop_times=rxpr[359];




        plans[2].Dst.p=((unsigned short)rxpr[402]<<8)+rxpr[403];
        plans[2].Dst.d=((unsigned short)rxpr[404]<<8)+rxpr[405];

        plans[2].delay_Out=rxpr[408];
        plans[2].delay_In=rxpr[409];
        plans[2].Outga_direction=rxpr[410];


        memcpy(&plans[2].k_nihe,&(rxpr[411]),sizeof(float));
        memcpy(&plans[2].b_nihe,&(rxpr[416]),sizeof(float));

        plans[2].D_max=(((unsigned short)rxpr[421])<<8)+rxpr[422];
        plans[2].D_min=(((unsigned short)rxpr[423])<<8)+rxpr[424];
        plans[2].D_middle=(((unsigned short)rxpr[425])<<8)+rxpr[426];


        plans[2].Ring_direction[0]=rxpr[427];
        plans[2].Ring_direction[1]=rxpr[428];
        plans[2].Ring_direction[2]=rxpr[429];
        plans[2].Ring_direction[3]=rxpr[430];

        plans[2].speedslow=(((unsigned short)rxpr[431])<<8)+rxpr[432];
        memcpy(&(plans[2].speedadd),&(rxpr[433]),sizeof(float));

        memcpy(&(plans[2].speedD),&(rxpr[438]),sizeof(float));

        plans[2].Ring_sumIN=((unsigned short)rxpr[443]<<8)+rxpr[444];

        plans[2].Ramp_delay=((unsigned short)rxpr[445]<<8)+rxpr[446];

        plans[2].Ist.p=rxpr[447];
        plans[2].Ist.i=rxpr[448];
        plans[2].Ist.d=rxpr[449];


        plans[2].Icu.p=rxpr[454];
        plans[2].Icu.i=rxpr[455];
        plans[2].Icu.d=rxpr[456];

        plans[2].err_istcu=((unsigned short)rxpr[457]<<8)+rxpr[458];

        plans[2].befringindelay[0]=rxpr[459];
        plans[2].befringindelay[1]=rxpr[460];
        plans[2].befringindelay[2]=rxpr[461];
        plans[2].befringindelay[3]=rxpr[462];

        plans[2].bangbangplus=((unsigned short)rxpr[463]<<8)+rxpr[464];
        plans[2].bangbangminus=((unsigned short)rxpr[465]<<8)+rxpr[466];

        plans[2].bangpluspwm=((unsigned short)rxpr[467]<<8)+rxpr[468];
        plans[2].bangminuspwm=((unsigned short)rxpr[469]<<8)+rxpr[470];

        plans[2].Dian.p=((unsigned short)rxpr[475]<<8)+rxpr[476];
        plans[2].Dian.i=((unsigned short)rxpr[477]<<8)+rxpr[478];
        plans[2].Dian.d=((unsigned short)rxpr[479]<<8)+rxpr[480];

        plans[2].speed_camp=((unsigned short)rxpr[481]<<8)+rxpr[482];

        plans[2].delay_threeall=((unsigned short)rxpr[483]<<8)+rxpr[484];

        plans[2].Three_direction[0]=rxpr[485];
        plans[2].Three_direction[1]=rxpr[486];

        plans[2].delay_threein0=rxpr[487];

        plans[2].delay_threeout0=rxpr[488];

        plans[2].Outga_direction=rxpr[489];

        plans[2].delay_outgarmid=rxpr[490];

        plans[2].errlinebegin=rxpr[491];

        plans[2].Ring_turnover=((unsigned short)rxpr[492]<<8)+rxpr[493];

        plans[2].Ring_speedin=((unsigned short)rxpr[494]<<8)+rxpr[495];

        plans[2].Ring_inmax=((unsigned short)rxpr[496]<<8)+rxpr[497];

        plans[2].Ring_over=((unsigned short)rxpr[498]<<8)+rxpr[499];

        plans[2].Three_speedin=((unsigned short)rxpr[500]<<8)+rxpr[501];

        memcpy(&(plans[2].speederrp) ,&(rxpr[502]),sizeof(float));
        memcpy(&(plans[2].speederrd) ,&(rxpr[507]),sizeof(float));

        plans[2].Dcup=((unsigned short)rxpr[513]<<8)+rxpr[514];
        plans[2].Dcud=((unsigned short)rxpr[515]<<8)+rxpr[516];

        plans[2].errcamcust=((unsigned short)rxpr[517]<<8)+rxpr[518];

        plans[2].angle_Ring[0]=((unsigned short)rxpr[519]<<8)+rxpr[520];
        plans[2].angle_Ring[1]=((unsigned short)rxpr[521]<<8)+rxpr[522];
        plans[2].angle_Ring[2]=((unsigned short)rxpr[523]<<8)+rxpr[524];
        plans[2].angle_Ring[3]=((unsigned short)rxpr[525]<<8)+rxpr[526];

        plans[2].angle_Threein[0]=((unsigned short)rxpr[527]<<8)+rxpr[528];
        plans[2].angle_Threein[1]=((unsigned short)rxpr[529]<<8)+rxpr[530];

        plans[2].angle_Outgar=((unsigned short)rxpr[531]<<8)+rxpr[532];
        plans[2].angle_Ingar=((unsigned short)rxpr[533]<<8)+rxpr[534];

        plans[2].angle_Threeout[0]=((unsigned short)rxpr[539]<<8)+rxpr[540];
        plans[2].angle_Threeout[1]=((unsigned short)rxpr[541]<<8)+rxpr[542];

        plans[2].speed_Ring=((unsigned short)rxpr[543]<<8)+rxpr[544];

        plans[2].speed_finishin=rxpr[545];
        plans[2].speed_finished=rxpr[546];

        plans[2].Three_speedout=((unsigned short)rxpr[547]<<8)+rxpr[548];

        plans[2].finished_time=rxpr[549];
         plans[2].stop_sum=rxpr[550];

         plans[2].delay_threein1=rxpr[557];
         plans[2].delay_threeout1=rxpr[558];
         plans[2].stop_times=rxpr[559];


        plans[3].Dst.p=((unsigned short)rxpr[602]<<8)+rxpr[603];
        plans[3].Dst.d=((unsigned short)rxpr[604]<<8)+rxpr[605];

        plans[3].delay_Out=rxpr[608];
        plans[3].delay_In=rxpr[609];
        plans[3].Outga_direction=rxpr[610];


        memcpy(&plans[3].k_nihe,&(rxpr[611]),sizeof(float));
        memcpy(&plans[3].b_nihe,&(rxpr[616]),sizeof(float));

        plans[3].D_max=(((unsigned short)rxpr[621])<<8)+rxpr[622];
        plans[3].D_min=(((unsigned short)rxpr[623])<<8)+rxpr[624];
        plans[3].D_middle=(((unsigned short)rxpr[625])<<8)+rxpr[626];


        plans[3].Ring_direction[0]=rxpr[627];
        plans[3].Ring_direction[1]=rxpr[628];
        plans[3].Ring_direction[2]=rxpr[629];
        plans[3].Ring_direction[3]=rxpr[630];

        plans[3].speedslow=(((unsigned short)rxpr[631])<<8)+rxpr[632];
        memcpy(&(plans[3].speedadd),&(rxpr[633]),sizeof(float));

        memcpy(&(plans[3].speedD),&(rxpr[638]),sizeof(float));

        plans[3].Ring_sumIN=((unsigned short)rxpr[643]<<8)+rxpr[644];

        plans[3].Ramp_delay=((unsigned short)rxpr[645]<<8)+rxpr[646];

        plans[3].Ist.p=rxpr[647];
        plans[3].Ist.i=rxpr[648];
        plans[3].Ist.d=rxpr[649];

        plans[3].Icu.p=rxpr[654];
        plans[3].Icu.i=rxpr[655];
        plans[3].Icu.d=rxpr[656];

        plans[3].err_istcu=((unsigned short)rxpr[657]<<8)+rxpr[658];

        plans[3].befringindelay[0]=rxpr[659];
        plans[3].befringindelay[1]=rxpr[660];
        plans[3].befringindelay[2]=rxpr[661];
        plans[3].befringindelay[3]=rxpr[662];

        plans[3].bangbangplus=((unsigned short)rxpr[663]<<8)+rxpr[664];
        plans[3].bangbangminus=((unsigned short)rxpr[665]<<8)+rxpr[666];

        plans[3].bangpluspwm=((unsigned short)rxpr[667]<<8)+rxpr[668];
        plans[3].bangminuspwm=((unsigned short)rxpr[669]<<8)+rxpr[670];


        plans[3].Dian.p=((unsigned short)rxpr[675]<<8)+rxpr[676];
        plans[3].Dian.i=((unsigned short)rxpr[677]<<8)+rxpr[678];
        plans[3].Dian.d=((unsigned short)rxpr[679]<<8)+rxpr[680];

        plans[3].speed_camp=((unsigned short)rxpr[681]<<8)+rxpr[682];

        plans[3].delay_threeall=((unsigned short)rxpr[683]<<8)+rxpr[684];

        plans[3].Three_direction[0]=rxpr[685];
        plans[3].Three_direction[1]=rxpr[686];

        plans[3].delay_threein0=rxpr[687];

        plans[3].delay_threeout0=rxpr[688];

        plans[3].Outga_direction=rxpr[689];

        plans[3].delay_outgarmid=rxpr[690];

        plans[3].errlinebegin=rxpr[691];

        plans[3].Ring_turnover=((unsigned short)rxpr[692]<<8)+rxpr[693];

        plans[3].Ring_speedin=((unsigned short)rxpr[694]<<8)+rxpr[695];

        plans[3].Ring_inmax=((unsigned short)rxpr[696]<<8)+rxpr[697];

        plans[3].Ring_over=((unsigned short)rxpr[698]<<8)+rxpr[699];

        plans[3].Three_speedin=((unsigned short)rxpr[700]<<8)+rxpr[701];

        memcpy(&(plans[3].speederrp) ,&(rxpr[702]),sizeof(float));
        memcpy(&(plans[3].speederrd) ,&(rxpr[707]),sizeof(float));

        plans[3].Dcup=((unsigned short)rxpr[713]<<8)+rxpr[714];
        plans[3].Dcud=((unsigned short)rxpr[715]<<8)+rxpr[716];

        plans[3].errcamcust=((unsigned short)rxpr[717]<<8)+rxpr[718];

        plans[3].angle_Ring[0]=((unsigned short)rxpr[719]<<8)+rxpr[720];
        plans[3].angle_Ring[1]=((unsigned short)rxpr[721]<<8)+rxpr[722];
        plans[3].angle_Ring[2]=((unsigned short)rxpr[723]<<8)+rxpr[724];
        plans[3].angle_Ring[3]=((unsigned short)rxpr[725]<<8)+rxpr[726];

        plans[3].angle_Threein[0]=((unsigned short)rxpr[727]<<8)+rxpr[728];
        plans[3].angle_Threein[1]=((unsigned short)rxpr[729]<<8)+rxpr[730];

        plans[3].angle_Outgar=((unsigned short)rxpr[731]<<8)+rxpr[732];
        plans[3].angle_Ingar=((unsigned short)rxpr[733]<<8)+rxpr[734];

        plans[3].angle_Threeout[0]=((unsigned short)rxpr[739]<<8)+rxpr[740];
        plans[3].angle_Threeout[1]=((unsigned short)rxpr[741]<<8)+rxpr[742];

        plans[3].speed_Ring=((unsigned short)rxpr[743]<<8)+rxpr[744];

        plans[3].speed_finishin=rxpr[745];
        plans[3].speed_finished=rxpr[746];
        plans[3].Three_speedout=((unsigned short)rxpr[747]<<8)+rxpr[748];

        plans[3].finished_time=rxpr[749];
         plans[3].stop_sum=rxpr[750];

         plans[3].delay_threein1=rxpr[757];
          plans[3].delay_threeout1=rxpr[758];
          plans[3].stop_times=rxpr[759];


        memcpy(&(planc),&(plans[0]),sizeof(planc));//TODO
        plansnum=0;

    OLED_P6x8Str(6,2," FLASH_finished ");

  }

}

void camtestmach(uint8 KEY)
{
  char txt[20];
  if((KEY&key_left)==key_left)
  {
    if(PWM_test6>-9600)
    PWM_test6-=100;
  }
  else if((KEY&key_right)==key_right)
  {
    if(PWM_test6<9600)
    PWM_test6+=100;
  }
  else if((KEY&key_out)==key_out)
  {
    ui_floor--;
    ui_anjian=1;
    ui_var_line=1;
    PWM_test6=0;
    ui_In_floor=1;
    ATOM_PWM_SetDuty(MOTOR_Backward_IO, 0, 17000);
    ATOM_PWM_SetDuty(MOTOR_Forward_IO, 0, 17000);
  }
  sprintf(txt," PWM:  %05d  ",PWM_test6);
  OLED_P6x8Str(2,0,"          ");
  OLED_P6x8Str(2,1,"          ");
  OLED_P6x8Str(2,2,"          ");
  OLED_P6x8Str(2,3,"          ");
  OLED_P6x8Str(2,4,"          ");
  OLED_P6x8Str(2,5,txt);

  if(PWM_test6>=0)
  {
    ATOM_PWM_SetDuty(MOTOR_Backward_IO, 0, 17000);
    delayus(3);
    ATOM_PWM_SetDuty(MOTOR_Forward_IO, PWM_test6, 17000);
  }
  else
  {
    ATOM_PWM_SetDuty(MOTOR_Forward_IO, 0, 17000);
    delayus(3);
    ATOM_PWM_SetDuty(MOTOR_Backward_IO, -PWM_test6, 17000);
  }
}


void change_plan(uint8 KEY)
{

  char txt[22];

  OLED_Write68(2,2,"    plan0          ",ui_changeplan_line!=1);
  OLED_Write68(2,3,"     plan1         ",ui_changeplan_line!=2);
  OLED_Write68(2,4,"      plan2        ",ui_changeplan_line!=3);
  OLED_Write68(2,5,"       plan3       ",ui_changeplan_line!=4);
  OLED_Write68(2,6,"  restart all plan ",ui_changeplan_line!=5);
  OLED_Write68(2,1,"         ",1);

  sprintf(txt,"  now_plan:   %03d  ",plansnum);
  OLED_Write68(5,0,txt,1);

  if(ui_changeplan_line>5)
  {
    ui_changeplan_line=5;
  }
  else if(ui_changeplan_line<1)
  {
    ui_changeplan_line=1;
  }

  if((KEY&key_up)==key_up)
  {
    if(ui_changeplan_line!=1)
    {
      ui_changeplan_line--;
    }
  }
  else if((KEY&key_down)==key_down)
  {
    if(ui_changeplan_line!=5)
    {
      ui_changeplan_line++;
    }
  }
  else if((KEY&key_in)==key_in)
  {
    if(ui_changeplan_line>0 && ui_changeplan_line<5)
    {
      memcpy(&(plans[plansnum]),&(planc),sizeof(planc));
      memcpy(&(planc),&(plans[ui_changeplan_line-1]),sizeof(planc));
      plansnum=ui_changeplan_line-1;
    }
    else if(ui_changeplan_line==5)
    {
      Toplancvar();
      memcpy(&(planc),&(plans[0]),sizeof(planc));
      plansnum=0;
      OLED_Write68(2,7," success restart",1);
    }

  }
  else if((KEY&key_out)==key_out)
  {
    ui_floor--;
    ui_anjian=1;
    ui_var_line=1;
    ui_In_floor=1;
  }
}



