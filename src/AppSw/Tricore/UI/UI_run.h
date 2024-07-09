#ifndef _UI_RUN_H_
#define _UI_RUN_H_



#include "LQ_GPIO.h"
#include "LQ_STM.h"
#include "math.h"
#include "LQ_OLED096.h"
#include "stdbool.h"
#include "stdio.h"
#include "LQ_ADC.h"
#include "UI_init.h"

extern uint8 ui_first_line;
extern uint8 ui_page;
extern uint8 ui_floor;
extern uint8 ui_report_line;
extern uint8 ui_report_floor;
extern cam_judge rungo;
extern cam_jurun runstate;
extern CAM_Inductance Ind;

extern _Bool runbeginflag;

extern volatile unsigned int cam_jishi;

void UI_init(void);

void UI_begin(void);

void UI_First(uint8 KEY);

void UI_Changevar(uint8 KEY);

void UI_Reportcam(uint8 KEY);

void main_run(CAM_var *plan, CAM_run *speed , Fit_line *line,CAM_Inductance *indu);

void Toplancvar(void);
void UI_writeinflash(void);
void UI_readfromflash(void);
void Flash_data_change(void);

void cam_Test_Inductance(uint8 KEY);
void camtestmach(uint8 KEY);
void change_plan(uint8 KEY);

////新车//
#define Inleft_IO  ADC35
#define Inlevelr_IO   ADC38
#define Inlevell_IO ADC37
#define Inright_IO ADC36

//////老车//////
//#define Inleft_IO  ADC35
//#define Inlevelr_IO ADC37
//#define Inlevell_IO ADC36
//#define Inright_IO ADC38


#define Invertic_IO ADC44



#endif
