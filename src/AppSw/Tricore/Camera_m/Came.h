#ifndef _CAME_HH_
#define _CAME_HH_

#include"LQ_MT9V034.h"
#include"LQ_CAMERA.h"
#include "stdbool.h"
#include "string.h"
#include "UI_init.h"

extern CAM_var planc;
extern CAM_run runc;
extern Fit_line line_fit;
extern Subs_proc Three_dec;
extern Subs_proc FMid_dec;
extern volatile _Bool cam_Ramp_flag;
extern volatile _Bool cam_finish_flag;
extern  int cam_Ringin_bit;
extern int cam_Three_quar;
extern volatile _Bool cam_Ring_flag;
extern _Bool cam_Ringin_flag;
extern _Bool cam_Ringout_flag;
extern volatile int jishi_quanju;
extern volatile _Bool cam_Stop_flag;
extern volatile _Bool cam_Ring_in;
extern volatile _Bool finished_stop;
extern unsigned short Duo_limitplus;
extern unsigned short Duo_limitminus;



#define Iwidth   0X3C
#define Ilength  0X5E
#define Iwidhalf 0X1E
#define Ilenghalf 0X2F




extern volatile uint8 imagin_init[120][188];

extern volatile _Bool imagin_cam[Iwidth][Ilength];//


extern int Cam_Process_Result;
extern unsigned char cam_finished_time;
extern _Bool Lost_Center;
extern _Bool Lost_Left;
extern volatile _Bool onefinished;
#define Differto(x) ((x==0)?1:0)
extern _Bool Lost_Right;
extern unsigned char cam_Ring_num;
extern int cam_finished_gar;
extern _Bool cam_three_flag;
extern volatile _Bool cam_Outga;
unsigned char cam_Three_judge_side(void);
unsigned char DFS_UPblack_side(void);
void Get_Cam_Process_Val(CAM_var *pla, Fit_line *lin,CAM_Inductance *indu);
void getdajinima(void);
unsigned char Three_outjudge(void);
unsigned char threeDFSside(void);
void cam_Three_run(CAM_var * pla,CAM_Inductance *indu , Fit_line *linec);
unsigned char cam_Three_judge_outr(void);
unsigned char cam_Three_judge_outl(void);
unsigned char DFS_UPblack_out(void);
unsigned char Three_Leftin(void);
unsigned char Three_Rightin(void);
void cam_Camprun(CAM_var *pla,CAM_Inductance *indu);
void make_cam_err(CAM_var *plan,CAM_Inductance *ind);
void cam_Three_judge(void);
unsigned char Ramp_judge(void);
void ISVUP_init(void);
unsigned char DFS_UPblack_out(void);
unsigned char cam_judge_threemid(void);
void  DFS_UPblack_side_step3(void);
void DFS_UPblack_side_step2(void);
void DFS_UPblack_side_step1(void);
unsigned char Finish_judge(void);
void cam_getspeed(CAM_var *pla,CAM_run *ru);
void cam_Ring_run(CAM_var * pla ,CAM_Inductance *indu , Fit_line *linec);
unsigned char DFS_UPblack(void);
unsigned char cam_Ringout_judge(void);
unsigned char cam_Threeoutfinished(CAM_Inductance *indu);
unsigned char cam_three(void);

#endif
