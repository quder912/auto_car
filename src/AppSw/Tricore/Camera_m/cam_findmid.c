
#include "Came.h"
#include "cam_findmid.h"
#include <LQ_UART.h>
#include "stdio.h"
#include "string.h"


volatile short Point_Left[Iwidth];
volatile short Point_Right[Iwidth];
volatile short Point_Center[Iwidth];

_Bool  Exist_Left[Iwidth];
_Bool  Exist_Right[Iwidth];
volatile _Bool  Exist_Center[Iwidth];


float ercinihe=0.2; //0-1

unsigned short cross09bit=0;


/***********堆栈溢出___暂时放这里***********/



short baohuzhi;
_Bool Left_Valid = 0, Right_Valid = 0;
uint8 Left_Start = 0;
uint8 Right_Start = 0;
short MAX_Start=0;

unsigned char Dfstopline=3;

/************************************************/

/*********************三岔************************/

unsigned short ThreeupLsidewhi;
unsigned short ThreeupRsidewhi;

unsigned short Threeleftinupr;
unsigned short Threerightinupl;



/***************************************************/


void Find_Center_Line(Fit_line *li,CAM_var *pla)
{
    FMid_dec.part_5(pla);
    FMid_dec.part_10(li);
    FMid_dec.part_11(li);
//    RepairTopLine();
}



/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/






//找到所有的边界点 边界点均为白0
void ImageDfs(void)
{
    static _Bool IsVisited[Iwidth][Ilength]={{0}};
    static uint8 nowpointxy[2][4000];
    short nowpoint;
    uint8 i=0, j=0;
    nowpoint=-1;
    cross09bit=0;
    ThreeupLsidewhi=0;
    ThreeupRsidewhi=0;
    Threeleftinupr=0;
    Threerightinupl=0;


    memset(IsVisited,0,sizeof(IsVisited));
    memset(nowpointxy,0,sizeof(nowpointxy));
    memset(Exist_Left,0,sizeof(Exist_Left));
    memset(Exist_Right,0,sizeof(Exist_Right));

    if (imagin_cam[57][Ilenghalf])
    {
        nowpoint++;
        nowpointxy[0][nowpoint] = 57;
        nowpointxy[1][nowpoint] = Ilength >> 1;
        IsVisited[57][Ilenghalf] = 1;
    }
    else
    {
        for (j = 2; j < Ilenghalf-3; j++)
        {
            if ( imagin_cam[57][Ilenghalf+j - 1] && imagin_cam[57][Ilenghalf+j] && imagin_cam[57][Ilenghalf+j + 1] )
            {
                nowpoint++;
                nowpointxy[0][nowpoint] = 57;
                nowpointxy[1][nowpoint] = Ilenghalf+j;
                IsVisited[57][Ilenghalf+j] = 1;
                break;
            }

            else if( imagin_cam[57][Ilenghalf-j - 1] &&imagin_cam[57][Ilenghalf-j] && imagin_cam[57][Ilenghalf-j + 1] )
            {
                nowpoint++;
                nowpointxy[0][nowpoint] = 57;
                nowpointxy[1][nowpoint] = Ilenghalf-j;
                IsVisited[57][Ilenghalf-j] = 1;
                break;
            }
        }
    }
    while (nowpoint >= 0)
    {
        i = nowpointxy[0][nowpoint];
        j = nowpointxy[1][nowpoint];
        nowpoint--;

        if ((i < 1) ||  (j < 2) || (j > Ilength - 3))
        {
            continue;
        }

        if (!IsVisited[i][j - 1])
        {
            IsVisited[i][j - 1] = 1;
            if (imagin_cam[i][j - 1])
            {
                nowpoint++;
                nowpointxy[0][nowpoint] = i;
                nowpointxy[1][nowpoint] = j - 1;
                if(i<5 && j<65 && j>28)
                {
                    cross09bit++;
                }
                if(i<12)
                {
                    if(j<28)
                    {
                        ThreeupLsidewhi++;
                    }
                    else if(j>65)
                    {
                        ThreeupRsidewhi++;
                    }
                }
                if(i<4)
                {
                    if(j>57 && j<83)
                    {
                        Threeleftinupr++;
                    }
                    else if(j>10 && j<37)
                    {
                        Threerightinupl++;
                    }
                }
            }
            else
            {
                if (IsLeftPoint(i, j))
                {
                    Point_Left[i] = j;
                    Exist_Left[i] = 1;
                }
            }
        }
        if (!IsVisited[i][j + 1])
        {
            IsVisited[i][j + 1] = 1;
            if (imagin_cam[i][j + 1])
            {
                nowpoint++;
                nowpointxy[0][nowpoint] = i;
                nowpointxy[1][nowpoint] = j + 1;
                if(i<5 && j<65 && j>28)
                {
                    cross09bit++;
                }
                if(i<14)
                {
                    if(j<28)
                    {
                        ThreeupLsidewhi++;
                    }
                    else if(j>65)
                    {
                        ThreeupRsidewhi++;
                    }
                }
                if(i<4)
                {
                    if(j>57 && j<83)
                    {
                        Threeleftinupr++;
                    }
                    else if(j>10 && j<37)
                    {
                        Threerightinupl++;
                    }
                }
            }
            else
            {
                if (IsRightPoint(i, j))
                {
                    Point_Right[i] = j;
                    Exist_Right[i] = 1;
                }
            }
        }
        if (!IsVisited[i - 1][j] && imagin_cam[i - 1][j])
        {
            IsVisited[i - 1][j] = 1;
            nowpoint++;
            nowpointxy[0][nowpoint] = i - 1;
            nowpointxy[1][nowpoint] = j;
            if(i<5 && j<65 && j>28)
            {
                cross09bit++;
            }
            if(i<14)
            {
                if(j<28)
                {
                    ThreeupLsidewhi++;
                }
                else if(j>65)
                {
                    ThreeupRsidewhi++;
                }
            }
            if(i<4)
            {
                if(j>57 && j<83)
                {
                    Threeleftinupr++;
                }
                else if(j>10 && j<37)
                {
                    Threerightinupl++;
                }
            }

        }
    }
    return;
}




_Bool IsLeftPoint(uint8 i, uint8 j)
{

    if (j < 2 || j > Ilength - 10)
    {
        return 0;
    }

    if ((!imagin_cam[i][j]) || (!imagin_cam[i][j + 1]) || (!imagin_cam[i][j + 2]))
    {
        return 0;
    }

    if ((imagin_cam[i][j - 1]) || (imagin_cam[i][j - 2]))
    {
        return 0;
    }

    if(cam_Ringin_flag && j>70)
    {
        return 0;
    }

    return 1;
}



_Bool IsRightPoint(uint8 i, uint8 j)
{

    if (j < 10 || j > Ilength - 3)
    {
        return 0;
    }

    if ((!imagin_cam[i][j]) || (!imagin_cam[i][j - 1]) || (!imagin_cam[i][j - 2]))
    {
        return 0;
    }

    if ((imagin_cam[i][j + 1]) || (imagin_cam[i][j + 2]))
    {
        return 0;
    }

    if(cam_Ringin_flag && j<20)
    {
        return 0;
    }

    return 1;
}



/***************************************************************************/
/***************************************************************************/
/***************************************************************************/


void DoFilter(CAM_var *pla)
{
    short i, j;
    LostLineJudge();

    for(i=1,j=0;i<48;i++)
    {
        if(Exist_Left[i] && Exist_Right[i])
        {
            if(Point_Right[i] - Point_Left[i]>(3*pla->k_nihe*i +3*pla->b_nihe))
            {
                Exist_Right[i]=0;
                Exist_Left[i]=0;
            }
        }
    }

    if ((!Lost_Left) && (!Lost_Right)) //如果左右不丢线
    {
        for (i = 30; i > 0; i--)
        {
            if (Exist_Left[i] && Exist_Right[i])
            {
                if ((Point_Right[i] - Point_Left[i] <=  i ) && (i < Iwidhalf))
                {
                    Exist_Left[i] = 0;
                    Exist_Right[i] = 0;

                }
            }

            if (Exist_Left[i] && (Point_Left[i] > 84 ))
            {
                Exist_Left[i] = 0;
            }

            if (Exist_Right[i] && (Point_Right[i] < 10))
            {
                Exist_Right[i] = 0;
            }
        }
    }

    clean_side_lostr();
    clean_side_lostl();
 //   CrossLeftLineJudge(); //TODO
 //   CrossRightLineJudge(); //TODO

    char k;
    for (i = 55; i > 0; i--)
    {
        if (Exist_Left[i])
        {
            for (j = i + 1; j < 59; j++)
            {
                if (Exist_Left[j])
                {
                    k = (Point_Left[i] - Point_Left[j]) / (i - j);
                    if ((k >= 3) || ((j - i >= 5) && (k >= 2)))
                    {
                        Exist_Left[i] = 0;

                    }
                    else
                    {
                        k = -k;
                        if (k > 15)
                        {
                            Exist_Left[i] = 0;
                        }
                    }
                    break;
                }
            }
        }
    }

    for (i = 55; i >= 0; i--)
    {
        if (Exist_Right[i])
        {   //向下找第一个有效点
            for (j = i + 1; j < Iwidth; j++)
            {
                if (Exist_Right[j])
                {
                    k =(Point_Right[i] - Point_Right[j]) / (i - j);
                    if ((k <= -3) || ((j - i >= 5) && (k <= -2))) // TODO  滤除斜率
                    {
                        Exist_Right[i] = 0;

                    }
                    else  if (k > 15) //TODO 大弯阈值
                    {
                        Exist_Right[i] = 0;
                    }
                    break;
                }
            }
        }
    }

    LostLineJudge();
}







/*
 * 左右存点判断
 */
void LostLineJudge(void)
{
    short i;
    uint8 ValidCnt1 = 0;
    uint8 ValidCnt2 = 0;

    //计算左右有多少个有效点
    for (i = 1; i < 58; i++)
    {
        if (Exist_Left[i])
            ValidCnt1++;
        if (Exist_Right[i])
            ValidCnt2++;
    }

    //如果左边有效点个数小于阈值，则判定丢左线
    if (ValidCnt1 < 6)
    {
        Lost_Left = 1;
    }
    else
    {
        Lost_Left = 0;
    }

    //如果右边有效点个数小于阈值，则判定丢右线
    if (ValidCnt2 < 6)  //TODO 丢线阈值 调参变量
    {
        Lost_Right = 1;
    }
    else
    {
        Lost_Right = 0;
    }
}



/*
 *
 * 十字丢弃以上
 *
 */
void CrossLeftLineJudge(void)
{
    short i, j;
    //求左线最右点
    short Max_Left_Line_Point = 0;
    for (i = 1; i < Iwidth; i++)
    {
        if (Exist_Left[i] && (Max_Left_Line_Point < Point_Left[i]))
        {
            Max_Left_Line_Point = Point_Left[i];
        }
    }
    //若最右点未超过1/3则不检查
    if (Max_Left_Line_Point < Ilength *0.3 )
    {
        return;
    }

    //定义左拐点1 2
    short Left_Inflection_Point1 = 1;
    short Left_Inflection_Point2 = 1;

    /************************求左线拐点1***********************/
    for (i = Iwidth - 1; i > 1; i--)
    {   //从下往上遍历
        if (Exist_Left[i])
        {   //前提是该行存在左边界点
            for (j = i + 1; j < Iwidth; j++)
            {   //向下找最近有效点
                if (Exist_Left[j])
                {   //找到最近有效点
                    if (Point_Left[j] > Point_Left[i])
                    {   //如果趋势逆转，即下方左边界点在上方左边界点的右边
                        Left_Inflection_Point1 = j;
                    }
                    break;
                }
            }
        }
        //如果已找到拐点1,break
        if (Left_Inflection_Point1 != 1)
        {
            break;
        }
    }

    /************************求左线拐点2***********************/
    for (i = Left_Inflection_Point1 - 1; i > 1; i--)
    {   //从下往上遍历
        if (Exist_Left[i])
        {   //前提是该行存在左边界点
            for (j = i + 1; j < Iwidth; j++)
            {   //向下找最近有效点
                if (Exist_Left[j])
                {   //找到最近有效点
                    if (Point_Left[j] < Point_Left[i])
                    {   //如果趋势逆转，即下方左边界点在上方左边界点的左边
                        Left_Inflection_Point2 = j;
                    }
                    break;
                }
            }
        }
        //如果已找到拐点2,break
        if (Left_Inflection_Point2 != 1)
            break;
    }

    /************************最小二乘法拟合两段直线***********************/
    float K_Left1, B_Left1, K_Left2, B_Left2;
    float Aver_X, Aver_Y;
    float ValidCnt = 0;
    int Sum_X=0,Sum_Y=0,Sum_XX=0,Sum_XY=0;

    //拟合第一段
    for (i = Left_Inflection_Point1; i < Iwidth; i++)
    {
        if (Exist_Left[i])
        {
            Sum_X += i;
            Sum_Y += Point_Left[i];
            Sum_XX += i * i;
            Sum_XY += i * Point_Left[i];

            ValidCnt++;
        }
    }

    if (ValidCnt >= CROSS_EDGE_LEN)
    {
        Aver_X = Sum_X / ValidCnt;
        Aver_Y = Sum_Y / ValidCnt;

        K_Left1 = (Sum_XY - Sum_X * Sum_Y / ValidCnt) / (Sum_XX - Sum_X * Sum_X / ValidCnt);
        B_Left1 = Aver_Y - K_Left1 * Aver_X;
    }
    else
    {
        return;
    }

    //拟合第二段
    Sum_X = Sum_Y = Sum_XX = Sum_XY = 0;
    ValidCnt = 0;

    for (i = Left_Inflection_Point2; i <= Left_Inflection_Point1; i++)
    {
        if (Exist_Left[i])
        {
            Sum_X += i;
            Sum_Y += Point_Left[i];
            Sum_XX += i * i;
            Sum_XY += i * Point_Left[i];

            ValidCnt++;
        }
    }

    if (ValidCnt >= CROSS_EDGE_LEN)
    {
        Aver_X = Sum_X / ValidCnt;
        Aver_Y = Sum_Y / ValidCnt;

        K_Left2 = (Sum_XY - Sum_X * Sum_Y / ValidCnt) / (Sum_XX - Sum_X * Sum_X / ValidCnt);
        B_Left2 = Aver_Y - K_Left2 * Aver_X;
    }
    else
    {
        return;
    }

    //前面未return，说明上下段都拟合成功，下面开始计算拟合误差
    float Fit_Err1 = 0, Fit_Err2 = 0;
    uint8 cnt1 = 0, cnt2 = 0;
    float Fit_Val;
    for (i = Left_Inflection_Point2; i < Iwidth; i++)
    {
        if (Exist_Left[i])
        {
            if (i < Left_Inflection_Point1)
            {   //第二段
                Fit_Val = K_Left2 * i + B_Left2;
                Fit_Err2 += ((Point_Left[i] - Fit_Val) * (Point_Left[i] - Fit_Val));
                cnt2++;
            }
            else
            {   //第一段
                Fit_Val = K_Left1 * i + B_Left1;
                Fit_Err1 += ((Point_Left[i] - Fit_Val) * (Point_Left[i] - Fit_Val));
                cnt1++;
            }
        }
    }

    Fit_Err1 /= cnt1;
    Fit_Err2 /= cnt2;

    //两段都是直线，且段1向右段2向左
    if (
        ((Fit_Err1 <= 4 && Fit_Err2 <= 1) || (Fit_Err1 <= 1 && Fit_Err2 <= 4)) &&
        ((K_Left1 < -2 && K_Left2 > 1) || (K_Left1 < 0 && K_Left2 > 2.5f))      //TODO 需要改4 1 2.5这些
        )
    {
        for (i = 1; i < Left_Inflection_Point1; i++)
        {
            Exist_Left[i] = 0;
        }
        for (i = 1; i < Iwidth; i++)  //TODO 同下
        {
            Exist_Right[i] = 0;
        }
        Lost_Right = 1;
    }
}

void CrossRightLineJudge(void)
{
    //求右线最左点
    uint8 i, j;
    uint8 Min_Right_Line_Point = Ilength - 1;
    for (i = 1; i < Iwidth; i++)
    {
        if (Exist_Right[i] && (Min_Right_Line_Point > Point_Left[i]))
        {
            Min_Right_Line_Point = (uint8)Point_Left[i];
        }
    }
    //若最左点未超过1/3则不检查
    if (Min_Right_Line_Point > (2 * Ilength / 3))
    {
        return;
    }

    //定义右拐点1 2
    uint8 Right_Inflection_Point1 = 1;
    uint8 Right_Inflection_Point2 = 1;

    /************************求右线拐点1***********************/
    for (i = Iwidth - 1; i >= 1; i--)
    {   //从下往上遍历
        if (Exist_Right[i])
        {   //前提是该行存在右边界点
            for (j = i + 1; j < Iwidth; j++)
            {   //向下找最近有效点
                if (Exist_Right[j])
                {   //找到最近有效点
                    if (Point_Right[j] < Point_Right[i])
                    {   //如果趋势逆转，即下方右边界点在上方右边界点的左边
                        Right_Inflection_Point1 = j;
                    }
                    break;
                }
            }
        }
        //如果已找到拐点1,break
        if (Right_Inflection_Point1 != 1)
        {
            break;
        }
    }

    /************************求左线拐点2***********************/
    for (i = Right_Inflection_Point1 - 1; i >= 1; i--)
    {   //从下往上遍历
        if (Exist_Right[i])
        {   //前提是该行存在右边界点
            for (j = i + 1; j < Iwidth; j++)
            {   //向下找最近有效点
                if (Exist_Right[j])
                {   //找到最近有效点
                    if (Point_Right[j] > Point_Right[i])
                    {   //如果趋势逆转，即下方右边界点在上方右边界点的左边
                        Right_Inflection_Point2 = j;
                    }
                    break;
                }
            }
        }
        //如果已找到拐点2,break
        if (Right_Inflection_Point2 != 1)
            break;
    }

    /************************最小二乘法拟合两段直线***********************/
    float K_Right1, B_Right1, K_Right2, B_Right2;
    float Aver_X, Aver_Y;
    float ValidCnt = 0;
    int Sum_X=0,Sum_Y=0,Sum_XX=0,Sum_XY=0;

    //拟合第一段
    for (i = Right_Inflection_Point1; i < Iwidth; i++)
    {
        if (Exist_Right[i])
        {
            Sum_X += i;
            Sum_Y += Point_Right[i];
            Sum_XX += i * i;
            Sum_XY += i * Point_Right[i];

            ValidCnt++;
        }
    }

    if (ValidCnt >= CROSS_EDGE_LEN)
    {
        Aver_X = Sum_X / ValidCnt;
        Aver_Y = Sum_Y / ValidCnt;

        K_Right1 = (Sum_XY - Sum_X * Sum_Y / ValidCnt) / (Sum_XX - Sum_X * Sum_X / ValidCnt);
        B_Right1 = Aver_Y - K_Right1 * Aver_X;
    }
    else
    {
        return;
    }

    //拟合第二段
    Sum_X = Sum_Y = Sum_XX = Sum_XY = 0;
    ValidCnt = 0;

    for (i = Right_Inflection_Point2; i <= Right_Inflection_Point1; i++)
    {
        if (Exist_Right[i])
        {
            Sum_X += i;
            Sum_Y += Point_Right[i];
            Sum_XX += i * i;
            Sum_XY += i * Point_Right[i];

            ValidCnt++;
        }
    }

    if (ValidCnt >= CROSS_EDGE_LEN)
    {
        Aver_X = Sum_X / ValidCnt;
        Aver_Y = Sum_Y / ValidCnt;

        K_Right2 = (Sum_XY - Sum_X * Sum_Y / ValidCnt) / (Sum_XX - Sum_X * Sum_X / ValidCnt);
        B_Right2 = Aver_Y - K_Right2 * Aver_X;
    }
    else
    {
        return;
    }

    //前面未return，说明上下段都拟合成功，下面开始计算拟合误差
    float Fit_Err1 = 0, Fit_Err2 = 0;
    uint8 cnt1 = 0, cnt2 = 0;
    float Fit_Val;
    for (i = Right_Inflection_Point2; i < Iwidth; i++)
    {
        if (Exist_Right[i])
        {
            if (i < Right_Inflection_Point1)
            {   //第二段
                Fit_Val = K_Right2 * i + B_Right2;
                Fit_Err2 += ((Point_Left[i] - Fit_Val) * (Point_Left[i] - Fit_Val));
                cnt2++;
            }
            else
            {   //第一段
                Fit_Val = K_Right1 * i + B_Right1;
                Fit_Err1 += ((Point_Left[i] - Fit_Val) * (Point_Left[i] - Fit_Val));
                cnt1++;
            }
        }
    }

    Fit_Err1 /= cnt1;
    Fit_Err2 /= cnt2;

    if (
        ((Fit_Err1 <= 4 && Fit_Err2 <= 1) || (Fit_Err1 <= 1 && Fit_Err2 <= 4)) &&
        ((K_Right1 < -2 && K_Right2 > 1) || (K_Right1 < 0 && K_Right2 > 2.5f))  //TODO 同上
    )
    {
        for (uint8 i = 1; i < Right_Inflection_Point1; i++)
        {
            Exist_Right[i] = 0;
        }
        for (uint8 i = 1; i < Iwidth; i++) //TODO 待抛弃
        {
            Exist_Left[i] = 0;
        }
        Lost_Left = 1;
    }
}




/***************************************************************/
/***************************************************************/
/***************************************************************/
/***************************************************************/




void RepairBottomLine_left(Fit_line *line_f)
{
    short i;

    Left_Valid = 0 ;
    Left_Start = 0;
    line_f->ValidCnt=MAX_Start =0;

    line_f->A_Left=line_f->K_Left=line_f->B_Left=0;

    line_f->Sum_X=line_f->Sum_Y =line_f->Sum_XY =line_f->Sum_XX=line_f->Sum_XXXX=line_f->Sum_XXX=line_f->Sum_XXY =0;

    Fit_linemulsuml(line_f);


        if(line_f->ValidCnt > 6)
        {
            Fit_getbefakb(line_f);

           line_f->A_Left= (ercinihe * line_f->temp2)/line_f->temp ;
           line_f->K_Left= (ercinihe * line_f->temp1)/line_f->temp + (1.0-ercinihe) * line_f->K_yicimid;
           line_f->B_Left= (ercinihe * line_f->temp0)/line_f->temp + (1.0-ercinihe) * (line_f->Aver_Y - line_f->Aver_X * line_f->K_yicimid);

            for (i = Iwidth - 1; i > Left_Start-30 && i>2; i--)
            {
                if (!Exist_Left[i])
                {
                    baohuzhi=(line_f->A_Left*i*i+line_f->K_Left * i +line_f->B_Left);
                    if(baohuzhi>-45 && baohuzhi<140)
                    {
                        Exist_Left[i] = 1;
                        Point_Left[i] =baohuzhi;
                    }
                }
            }
        }

    return;
}




void Fit_linemulsuml(Fit_line *line_f)
{
    uint8 i;
    for (i = Iwidth - 3; i > 0; i--)
        {
            if (Exist_Left[i])
            {
                Left_Start = i;
                break;
            }
        }

        if (Left_Start > 8)
        {
            MAX_Start=(Left_Start-26>4)?Left_Start-26:4;
            for (i = Left_Start; i >=  MAX_Start; i--)
            {
                if (Exist_Left[i])
                {
                    line_f->Sum_X += i;
                    line_f->Sum_XX += i * i;
                    line_f->Sum_XXX += (int)i*i*i;
                    line_f->Sum_XXXX+=(int)i*i*i*i;
                    line_f->Sum_Y += Point_Left[i];
                    line_f->Sum_XY += i * Point_Left[i];
                    line_f->Sum_XXY +=(int)i*i*Point_Left[i];
                    line_f->ValidCnt++;
                }
                if (line_f->ValidCnt > 16 ||(cam_Ring_flag &&line_f->ValidCnt>9))
                {
                    return;
                }
            }
        }

        return;
}

void Fit_getbefakb(Fit_line *line_f)
{
    line_f->temp=line_f->ValidCnt*(line_f->Sum_XX*line_f->Sum_XXXX-line_f->Sum_XXX*line_f->Sum_XXX)-line_f->Sum_X*(line_f->Sum_X*line_f->Sum_XXXX-line_f->Sum_XX*line_f->Sum_XXX)+line_f->Sum_XX*(line_f->Sum_X*line_f->Sum_XXX-line_f->Sum_XX*line_f->Sum_XX);

    line_f->temp0=line_f->Sum_Y*(line_f->Sum_XX*line_f->Sum_XXXX-line_f->Sum_XXX*line_f->Sum_XXX)-line_f->Sum_XY*(line_f->Sum_X*line_f->Sum_XXXX-line_f->Sum_XX*line_f->Sum_XXX)+line_f->Sum_XXY*(line_f->Sum_X*line_f->Sum_XXX-line_f->Sum_XX*line_f->Sum_XX);

    line_f->temp1=line_f->ValidCnt*(line_f->Sum_XY*line_f->Sum_XXXX-line_f->Sum_XXY*line_f->Sum_XXX)-line_f->Sum_X*(line_f->Sum_Y*line_f->Sum_XXXX-line_f->Sum_XX*line_f->Sum_XXY)+line_f->Sum_XX*(line_f->Sum_Y*line_f->Sum_XXX-line_f->Sum_XY*line_f->Sum_XX);

    line_f->temp2=line_f->ValidCnt*(line_f->Sum_XX*line_f->Sum_XXY-line_f->Sum_XY*line_f->Sum_XXX)-line_f->Sum_X*(line_f->Sum_X*line_f->Sum_XXY-line_f->Sum_Y*line_f->Sum_XXX)+line_f->Sum_XX*(line_f->Sum_X*line_f->Sum_XY-line_f->Sum_Y*line_f->Sum_XX);

    line_f->Aver_X = line_f->Sum_X / line_f->ValidCnt;
    line_f->Aver_Y = line_f->Sum_Y / line_f->ValidCnt;

    line_f->K_yicimid=(line_f->Sum_XY * line_f->ValidCnt - line_f->Sum_X * line_f->Sum_Y ) / (line_f->Sum_XX * line_f->ValidCnt - line_f->Sum_X * line_f->Sum_X );

}



void RepairBottomLine_right(Fit_line *line_f)
{

    short i;
    Right_Valid = 0 ;
    Right_Start = 0;
    MAX_Start=0;

    line_f->A_Right=0;
    line_f->K_Right=0;
    line_f->B_Right=0;


    line_f->ValidCnt = 0;
    line_f->Sum_XY = line_f->Sum_XX=line_f->Sum_XXXX=0;
    line_f->Sum_XXX=line_f->Sum_XXY =0;
    line_f->Sum_X = line_f->Sum_Y = 0;
    line_f->temp=line_f->temp0=line_f->temp1=line_f->temp2=0;

    Fit_linemulsumr(line_f);

    if(line_f->ValidCnt >6)
    {
        Fit_getbefakb(line_f);
        line_f->A_Right = ercinihe * line_f->temp2/line_f->temp ;
        line_f->K_Right = ercinihe * line_f->temp1/line_f->temp + (1-ercinihe) * line_f->K_yicimid;
        line_f->B_Right = ercinihe * line_f->temp0/line_f->temp + (1-ercinihe) * (line_f->Aver_Y - line_f->Aver_X * line_f->K_yicimid);

        for (i = Iwidth - 1; i > Right_Start-30 && i>2; i--)
        {
            if (!Exist_Right[i])
            {
                baohuzhi=(line_f->A_Right*i*i+line_f->K_Right * i + line_f->B_Right);
                if(baohuzhi>-45 && baohuzhi<140)
                {
                    Exist_Right[i] = 1;
                    Point_Right[i] = baohuzhi;
                }
            }
        }

    }

    return;
}

void Fit_linemulsumr(Fit_line *line_f)
{
    uint8 i;
    for (i = Iwidth -3; i > 0; i--)
    {
        if (Exist_Right[i])
        {
            Right_Start = i;
            break;
        }
    }

    if (Right_Start > 8)
    {
        MAX_Start=(Right_Start-26>4)?Right_Start-26:4;
        for (i = Right_Start; i >= MAX_Start; i--)
        {
            if (Exist_Right[i])
            {
                line_f->Sum_X += i;
                line_f->Sum_XX += i * i;
                line_f->Sum_XXX += (int)i*i*i;
                line_f->Sum_XXXX+=(int)i*i*i*i;
                line_f->Sum_Y += Point_Right[i];
                line_f->Sum_XY += i * Point_Right[i];
                line_f->Sum_XXY +=(int)i*i*Point_Right[i];
                line_f->ValidCnt++;
            }
            if (line_f->ValidCnt > 16 || (cam_Ring_flag &&line_f->ValidCnt>9))
            {
                return;
            }
        }
    }

    return;

}






/***************************************************************/
/***************************************************************/
/***************************************************************/
/***************************************************************/
/***************************************************************/




void RepairTopLine(void)
{
    uint8 i;
    _Bool Is_Left_All_To_Right = 1, Is_Right_All_To_Left = 1;
    uint8 Left_ValidCnt = 0, Right_ValidCnt = 0;
    uint8 MinLeft_Row = Iwidth - 1, MinRight_Row = Iwidth - 1;

    for (i = Iwidth - 2; i > 1; i--)
    {
        if (Exist_Left[i])
        {
            Left_ValidCnt++;
            if (Exist_Left[i] && (Point_Left[i] < Point_Left[i + 1]) )
            {   //左线是否一直向右
                Is_Left_All_To_Right = 0;
            }
            MinLeft_Row = i;
        }
        if (Exist_Right[i])
        {
            Right_ValidCnt++;
            if (Exist_Right[i] && (Point_Right[i] > Point_Right[i + 1])) //TODO 同上
            {   //右线是否一直向左
                Is_Right_All_To_Left = 0;
            }
            MinRight_Row = i;
        }
    }
    /************************************补左*****************************************/
    if (Is_Left_All_To_Right && (Left_ValidCnt >= (Iwidth - 1)*0.5 )) //TODO 0.5可试着改小
    {
        //从最末点开始右上(包括右上)dfs,标记为左边点
        //右上>上>右
        //学长这里代码逻辑顺序和注释不同
        short Point_I = MinLeft_Row, Point_J = Point_Left[MinLeft_Row];
        while (1)
        {
            if (IsEdgePoint(Point_I - 1, Point_J + 1))
            {   //右上
                Point_I -= 1;
                Point_J += 1;

                Point_Left[Point_I] = Point_J;
                Exist_Left[Point_I] = 1;
            }
            else if (IsEdgePoint(Point_I - 1, Point_J))
            {   //上
                Point_I -= 1;

                Point_Left[Point_I] = Point_J;
                Exist_Left[Point_I] = 1;
            }
            else if (IsEdgePoint(Point_I, Point_J + 1))
            {   //右
                Point_J += 1;

                Point_Left[Point_I] = Point_J;
                Exist_Left[Point_I] = 1;
            }
            else
            {
                break;
            }
        }
    }

    /************************************补右*****************************************/
    if (Is_Right_All_To_Left && (Right_ValidCnt >= 0.5 * (Iwidth - 1))) // TODO 同上
    {
        //从最末点开始左上(包括左上)dfs,标记为右边点
        //左上>上>左
        //学长这里代码逻辑顺序和注释不同，
        short Point_I = MinRight_Row, Point_J = Point_Right[MinRight_Row];
        while (1)
        {
            if (IsEdgePoint(Point_I - 1, Point_J - 1))
            {   //左上
                Point_I -= 1;
                Point_J -= 1;

                Point_Right[Point_I] = Point_J;
                Exist_Right[Point_I] = 1;
            }
            else if (IsEdgePoint(Point_I - 1, Point_J))
            {   //上
                Point_I -= 1;

                Point_Right[Point_I] = Point_J;
                Exist_Right[Point_I] = 1;
            }
            else if (IsEdgePoint(Point_I, Point_J - 1))
            {   //左
                Point_J -= 1;

                Point_Right[Point_I] = Point_J;
                Exist_Right[Point_I] = 1;
            }
            else
            {
                break;
            }
        }
    }
}




_Bool IsEdgePoint(int i, int j)
{
    //数组越界保护
    if (i < 2 || i >= Iwidth - 3)
    {
        return 0;
    }
    if (j < 2 || j >= Ilength - 3)
    {
        return 0;
    }
    //自身不能是黑点0
    if (!imagin_cam[i][j])
    {
        return 0;
    }
    //上下左右不能全是白的1
    if ((imagin_cam[i - 1][j]) && (imagin_cam[i + 1][j]) &&
            (imagin_cam[i][j - 1]) && (imagin_cam[i][j + 1]))
    {
        return 0;
    }
    return 1;
}



/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/





void RepairWholeLine(Fit_line *line_f)
{
    uint8 i;

    line_f->A_Center=0;
    line_f->K_Center=0;
    line_f->B_Center=0;

    line_f->ValidCnt = 0;
    line_f->Sum_XY = line_f->Sum_XX=line_f->Sum_XXXX=0;
    line_f->Sum_XXX=line_f->Sum_XXY =0;
    line_f->Sum_X = line_f->Sum_Y = 0;
    line_f->temp=line_f->temp0=line_f->temp1=line_f->temp2=0;

    unsigned char Center_Start =RepairbetweenCen();

    Fit_linemulsumc(line_f ,Center_Start);

    if(line_f->ValidCnt>6)
    {
        for (i = Center_Start+15; i > 1; i--)
        {
            if(!Exist_Center[i])
            {
                baohuzhi=line_f->A_Center*i*i+ line_f->K_Center* i + line_f->B_Center;
                if(baohuzhi<=141&&baohuzhi>=-47)
                {
                    Point_Center[i] = baohuzhi;
                    Exist_Center[i] = 1;
                }
            }
            else
            {
                baohuzhi=line_f->A_Center*i*i+ line_f->K_Center* i + line_f->B_Center;
                if(baohuzhi<=104&&baohuzhi>=-10)
                {
                    Point_Center[i] = 0.40*baohuzhi+0.60*Point_Center[i];
                }

            }
        }
    }

   return;
}


void Fit_linemulsumc(Fit_line *line_f ,unsigned char Startli )
{

    uint8 i;
    for (i = Startli+20; i >= Startli && i>3; i--)
    {
        if (Exist_Center[i])
        {
            line_f->Sum_X += i;
            line_f->Sum_XX += i * i;
            line_f->Sum_XXX += (int)i*i*i;
            line_f->Sum_XXXX+=(int)i*i*i*i;
            line_f->Sum_Y += Point_Center[i];
            line_f->Sum_XY += i * Point_Center[i];
            line_f->Sum_XXY += (int)i*i*Point_Center[i];
            line_f->ValidCnt++;
        }
    }

    if (line_f->ValidCnt > 6)
    {
        Fit_getbefakb(line_f);

        line_f->A_Center= (0.22 * line_f->temp2)/line_f->temp ;
        line_f->K_Center= (0.22 * line_f->temp1)/line_f->temp + (0.78) * line_f->K_yicimid;
        line_f->B_Center= (0.22 * line_f->temp0)/line_f->temp + (0.78) * (line_f->Aver_Y - line_f->Aver_X * line_f->K_yicimid);
    }



}


unsigned char RepairbetweenCen(void)
{
    unsigned char Center_Start = 0;
    uint8 i,j,kk;
    float x1,y1,x2,y2;

        for (i = Iwidth - 1; i > 1; i--)
        {
            if (Exist_Center[i])
            {
                Center_Start = i;
                break;
            }
        }
        for (i =Center_Start; i > 1; i--)
        {
            if (!Exist_Center[i])
            {
                x1 = i + 1; y1 = Point_Center[i + 1];
                for (j = i - 1; j >= 1; j--)
                {
                    if (Exist_Center[j])
                    {
                        x2 = j;
                        y2 = Point_Center[j];

                        for (kk = j + 1; kk <= i; kk++) //利用两点间直线公式拟合所缺的点(先乘后除，防止先执行除法使得为0)
                        {
                            Point_Center[kk] = (kk - x1) *  (y2 - y1) / (x2 - x1) + y1;
                            Exist_Center[kk] = 1;
                        }
                        break;
                    }
                }
            }
        }
        for (i = 1; i < Iwidth - 1; i++)
        {
            if (Exist_Center[i])
            {
                Center_Start = i;
                break;
            }
        }

    return Center_Start;
}

/**************************************************/
/**************************************************/
/**************************************************/
/**************************************************/
/**************************************************/

void CenterLine_Validation(void)
{
    uint8 i;
    if (Lost_Right)
    {   //右丢线
        uint8 Left_ValidCnt = 0; //左边界点个数
        short Left_Col_Max = 0;  //表示左边界点横向最远延伸到了哪里
        for (i = 1; i < 30; i++)
        {
            if (Exist_Left[i])
            {
                Left_ValidCnt++;
                if (Point_Left[i] > Left_Col_Max)
                {
                    Left_Col_Max = Point_Left[i];
                }
            }
        }
    }

    if (Lost_Left)
    {   //左丢线
        uint8 Right_ValidCnt = 0; //右边界点个数
        short Right_Col_Min = Ilength - 1;  //表示右边界点横向最远延伸到了哪里
        for (i = 1; i < Iwidth>>1 ; i++)
        {
            if (Exist_Right[i])
            {
                Right_ValidCnt++;
                if (Point_Right[i] < Right_Col_Min)
                {
                    Right_Col_Min = Point_Right[i];
                }
            }
        }
    }

    if ((!Lost_Left) && (!Lost_Right))
    {   //不丢线
        uint8 Road_Len_Of_Image;//图片上拍摄到的路宽
        uint8 Road_Len_Of_Array;//利用Road_Len_K * i + Road_Len_B计算得到的路宽

        for (i = Iwidth >> 1; i < Iwidth; i++)
        {   //如果本行左右边点存在
            if (Exist_Left[i] && Exist_Right[i])
            {
                Road_Len_Of_Image = Point_Right[i] - Point_Left[i];
                Road_Len_Of_Array = planc.b_nihe * i + planc.k_nihe;

                //本行实际路宽应全满或大于roadLen - 10
                if ((Road_Len_Of_Image < Ilength - 5) && (Road_Len_Of_Image < Road_Len_Of_Array - 14)) //TODO 参数
                {

                }
            }
        }
    }
}



void  GetCenter(Fit_line *line_f)
{
    short i;
//    char txt[20];

    for(i=0;i<60;i++)
    {
        Exist_Center[i]=0;
    }
    if (Lost_Left && Lost_Right)
    {
        return;
    }

    Lost_Center = 0;
    if (Lost_Left)
    {
        line_f->ValidCnt = 0;
        float Sum_Err = 0;
        float Std_Point;

        for (i = 58; i > 2; i--)
        {
            if (Exist_Right[i])
            {   //计算中线补偿
                Std_Point = 47 + (planc.k_nihe * i + planc.b_nihe) ;
                if ((Std_Point < 141) && (Point_Right[i] < 141))
                {
                    line_f->ValidCnt++;
                    Sum_Err += (Std_Point - Point_Right[i]);
                }
            }
        }

        //个数足够,总Error在左
        short Center_Fix = 0;
        if ((line_f->ValidCnt >= 4) && (Sum_Err > 0))
        {
            float Aver_Err  = Sum_Err / line_f->ValidCnt;

//            sprintf(txt,"%f\n",Aver_Err);
//            UART_PutStr(UART0,txt);
            if (Aver_Err > 40) //TODO
            {
                Aver_Err = 40;
            }
                Center_Fix = Aver_Err * 0.5;// TODO 调参
        }

        for (i = Iwidth - 1; i > 1; i--)
        {
            if(Exist_Right[i] && Exist_Left[i])
            {
                Point_Center[i]= (Point_Left[i]+Point_Right[i])/4 + ( Point_Right[i] - ((planc.k_nihe+0.3) * i + planc.b_nihe)  - Center_Fix)/2;
                Exist_Center[i] = 1;
            }
            else if (Exist_Right[i])
            {
                Exist_Center[i] = 1;
                Point_Center[i] = Point_Right[i] - ((planc.k_nihe+0.3) * i + planc.b_nihe) - Center_Fix;
            }
        }
        RepairWholeLine(line_f);//修复中线
    }
    else if (Lost_Right)
    {

        line_f->ValidCnt = 0;
        float Sum_Err = 0;
        float Std_Point;

        for (i = 58; i > 2; i--)
        {
            if (Exist_Left[i])
            {   //计算中线补偿
                Std_Point = Ilenghalf - (planc.k_nihe * i + planc.b_nihe) ;
                if ((Std_Point > -47) && (Point_Left[i] > -47))
                {
                    line_f->ValidCnt++;
                    Sum_Err += (Point_Left[i] - Std_Point);
                }
            }
        }

        //个数足够,总Error向右
        float Center_Fix = 0;
        if ((line_f->ValidCnt >= 4) && (Sum_Err > 0))
        {
            float Aver_Err = Sum_Err / line_f->ValidCnt;
//
//            sprintf(txt,"%f\n",Aver_Err);
//            UART_PutStr(UART0,txt);

            if (Aver_Err > 45)
            {
                Aver_Err = 45;
            }
            Center_Fix = Aver_Err * 0.5;
        }

        //补线
        for (i = 55; i > 1; i--)
        {
            if(Exist_Left[i]&&Exist_Right[i])
            {
                Exist_Center[i] = 1;
                Point_Center[i] = (Point_Left[i]+Point_Right[i])/4 + (Point_Left[i] + ((planc.k_nihe+0.3) * i + planc.b_nihe)  + Center_Fix)/2;
            }
            else if (Exist_Left[i])
            {
                Exist_Center[i] = 1;
                Point_Center[i] = Point_Left[i] + ((planc.k_nihe+0.3) * i + planc.b_nihe)  + Center_Fix;

            }
        }
        RepairWholeLine(line_f);//修复中线
    }

    else if ((!Lost_Left) && (!Lost_Right))
    {
        for (i = 55; i > 1; i--)
        {
            if (Exist_Left[i] && Exist_Right[i])
            {
                if (Point_Right[i] - Point_Left[i] <= 4)//TODO
                {
                    continue;
                }
                Point_Center[i] = (Point_Right[i] + Point_Left[i])>>1;
                Exist_Center[i] = 1;
            }

            else if ((Exist_Left[i]) && (i < Iwidth * 0.7)) //左右丢其中一个像素时 用未丢补线
            {
                Point_Center[i] = Point_Left[i] + ((planc.k_nihe+0.18) * i + planc.b_nihe);
                Exist_Center[i] = 1;
            }

            else if ((Exist_Right[i]) && (i < Iwidth *0.7))
            {
                Point_Center[i] = Point_Right[i] - ((planc.k_nihe+0.18) * i + planc.b_nihe) ;
                Exist_Center[i] = 1;
            }
        }
        RepairWholeLine(line_f);//修复中线

    }

 //   CenterLine_Validation();  //TODO

    return;
}

void clean_side_lostl(void)
{
    uint8 i,j,k,l;
    short pom=256;

    if(Lost_Left)
    {
        for(i=8;i<40;i++)
        {
            if(!Exist_Right[i-1] && !Exist_Right[i-2] && !Exist_Right[i-3]  &&
                    imagin_cam[i-1][92] &&   imagin_cam[i-2][91] &&  imagin_cam[i-3][90] )
            {
                for(j=i;j<50;j++)
                {
                    if(Exist_Right[j])
                    {
                        if(pom>Point_Right[j])
                        {
                            l=j;
                            pom=Point_Right[j];
                        }
                    }
                }

//                    for(k=i;k>2;k--)
//                    {
//                        Exist_Right[k]=0;
//                    }
                    for(k=l+2;k>i;k--)
                    {
                        Exist_Right[k]=0;
                    }
                    return;
            }
        }
    }
}

void clean_side_lostr(void)
{
    uint8 i,j,k,l;
    short pom=0;

    if(Lost_Right)
    {
        for(i=8;i<40;i++)
        {
            if(!Exist_Left[i-1] && !Exist_Left[i-2] && !Exist_Left[i-3]  &&
                    imagin_cam[i-1][2] &&   imagin_cam[i-2][3] &&  imagin_cam[i-3][4] )
            {
                for(j=i;j<50;j++)
                {
                    if(Exist_Left[j])
                    {
                        if(pom<Point_Left[j])
                        {
                            l=j;
                            pom=Point_Left[j];
                        }
                    }
                }
//                if(1)
//                {
//                    for(k=i;k>2;k--)
//                    {
//                        Exist_Left[k]=0;
//                    }
                    for(k=l+2;k>i;k--)
                    {
                        Exist_Left[k]=0;
                    }
                    return;
//                }
            }
        }
    }
}

