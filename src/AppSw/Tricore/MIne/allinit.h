#ifndef _ALL_INIT_
#define _ALL_INIT_


#include "LQ_ADC.h"
#include "LQ_GPIO.h"
#include "LQ_GTM.h"
#include "LQ_STM.h"
#include "hanshu.h"
#include "LQ_OLED096.h"
#include "Platform_Types.h"
#include "LQ_GPT12_ENC.h"
#include "All_Struct.h"



//void UI_init(void);
void ADC_init(void);
void GPIO_init(void);
void PWM_init(void);
void tim_init(void);
void PID_init(PID_I *Str , PID_E *enc);
void IN_Varinit(void);
void init_all(void);



extern PID_I now_pid;
extern PID_E now_enc;
extern Dian now_speed;



extern volatile unsigned long time_run;


#define MOTOR_Forward_IO IfxGtm_ATOM0_3_TOUT49_P22_2_OUT
#define MOTOR_Backward_IO  IfxGtm_ATOM0_2N_TOUT56_P21_5_OUT

#define STEER_IO IfxGtm_ATOM0_1_TOUT31_P33_9_OUT








#endif

