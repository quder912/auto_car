#ifndef _CAM_MID_H_
#define _CAM_MID_H_

#include "Came.h"

extern volatile short Point_Left[Iwidth];
extern volatile short Point_Right[Iwidth];
extern volatile short Point_Center[Iwidth];
extern  _Bool Exist_Left[Iwidth];
extern  _Bool Exist_Right[Iwidth];
extern volatile  _Bool Exist_Center[Iwidth];
extern unsigned short cross09bit;

/*********************************/
extern unsigned short ThreeupLsidewhi;
extern unsigned short ThreeupRsidewhi;

extern unsigned short Threeleftinupr;
extern unsigned short Threerightinupl;


#define CROSS_EDGE_LEN 4                    //TODO　拟合十字路口两段直线时所需的最少点数

void Find_Center_Line(Fit_line *li,CAM_var *pla);

/***************shendusoubian******************/

void ImageDfs(void);

_Bool IsLeftPoint(uint8 i, uint8 j);
_Bool IsRightPoint(uint8 i, uint8 j);



/***************guolv************************/

void DoFilter(CAM_var *pla);

void LostLineJudge(void);
void CrossLeftLineJudge(void);
void CrossRightLineJudge(void);

void clean_side_lostr(void);
void clean_side_lostl(void);

/****************xiufu*************************/


void RepairBottomLine_left(Fit_line *line_f);
void RepairBottomLine_right(Fit_line *line_f);
void Fit_linemulsumr(Fit_line *line_f);
void Fit_linemulsuml(Fit_line *line_f);
void Fit_linemulsumc(Fit_line *line_f ,unsigned char Startli );

/*************jiugong********************/

void RepairTopLine(void);

_Bool IsEdgePoint(int i, int j);

/**********xiufuzhongxian***************/

unsigned char RepairbetweenCen(void);

void RepairWholeLine(Fit_line *line_f);
void Fit_getbefakb(Fit_line *line_fi);

void  GetCenter(Fit_line *line_f);

/*************jiaoyan****************************/

void CenterLine_Validation(void);


#endif


