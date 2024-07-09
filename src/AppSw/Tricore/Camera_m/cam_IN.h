#ifndef _CAM_INS_H_
#define _CAM_INS_H_

#include "UI_init.h"



void camgetIN(CAM_Inductance *ind);
unsigned char camIN2pidcamp(CAM_Inductance *ind,CAM_var *pla);
unsigned char camIN2pidcamp1(CAM_Inductance *ind,CAM_var *pla);
void camIN2pid(CAM_Inductance *ind,CAM_var *pla);
unsigned char camIN2pidcampout(CAM_Inductance *ind,CAM_var *pla);
void camrunIN(CAM_Inductance *indu,CAM_var *plan);
void cam_Outgarage(CAM_var *pla);
unsigned char cam_Ring_judge(CAM_Inductance *indu);

#endif
