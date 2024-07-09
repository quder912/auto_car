
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


/***********��ջ���___��ʱ������***********/



short baohuzhi;
_Bool Left_Valid = 0, Right_Valid = 0;
uint8 Left_Start = 0;
uint8 Right_Start = 0;
short MAX_Start=0;

unsigned char Dfstopline=3;

/************************************************/

/*********************����************************/

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






//�ҵ����еı߽�� �߽���Ϊ��0
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

    if ((!Lost_Left) && (!Lost_Right)) //������Ҳ�����
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
        {   //�����ҵ�һ����Ч��
            for (j = i + 1; j < Iwidth; j++)
            {
                if (Exist_Right[j])
                {
                    k =(Point_Right[i] - Point_Right[j]) / (i - j);
                    if ((k <= -3) || ((j - i >= 5) && (k <= -2))) // TODO  �˳�б��
                    {
                        Exist_Right[i] = 0;

                    }
                    else  if (k > 15) //TODO ������ֵ
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
 * ���Ҵ���ж�
 */
void LostLineJudge(void)
{
    short i;
    uint8 ValidCnt1 = 0;
    uint8 ValidCnt2 = 0;

    //���������ж��ٸ���Ч��
    for (i = 1; i < 58; i++)
    {
        if (Exist_Left[i])
            ValidCnt1++;
        if (Exist_Right[i])
            ValidCnt2++;
    }

    //��������Ч�����С����ֵ�����ж�������
    if (ValidCnt1 < 6)
    {
        Lost_Left = 1;
    }
    else
    {
        Lost_Left = 0;
    }

    //����ұ���Ч�����С����ֵ�����ж�������
    if (ValidCnt2 < 6)  //TODO ������ֵ ���α���
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
 * ʮ�ֶ�������
 *
 */
void CrossLeftLineJudge(void)
{
    short i, j;
    //���������ҵ�
    short Max_Left_Line_Point = 0;
    for (i = 1; i < Iwidth; i++)
    {
        if (Exist_Left[i] && (Max_Left_Line_Point < Point_Left[i]))
        {
            Max_Left_Line_Point = Point_Left[i];
        }
    }
    //�����ҵ�δ����1/3�򲻼��
    if (Max_Left_Line_Point < Ilength *0.3 )
    {
        return;
    }

    //������յ�1 2
    short Left_Inflection_Point1 = 1;
    short Left_Inflection_Point2 = 1;

    /************************�����߹յ�1***********************/
    for (i = Iwidth - 1; i > 1; i--)
    {   //�������ϱ���
        if (Exist_Left[i])
        {   //ǰ���Ǹ��д�����߽��
            for (j = i + 1; j < Iwidth; j++)
            {   //�����������Ч��
                if (Exist_Left[j])
                {   //�ҵ������Ч��
                    if (Point_Left[j] > Point_Left[i])
                    {   //���������ת�����·���߽�����Ϸ���߽����ұ�
                        Left_Inflection_Point1 = j;
                    }
                    break;
                }
            }
        }
        //������ҵ��յ�1,break
        if (Left_Inflection_Point1 != 1)
        {
            break;
        }
    }

    /************************�����߹յ�2***********************/
    for (i = Left_Inflection_Point1 - 1; i > 1; i--)
    {   //�������ϱ���
        if (Exist_Left[i])
        {   //ǰ���Ǹ��д�����߽��
            for (j = i + 1; j < Iwidth; j++)
            {   //�����������Ч��
                if (Exist_Left[j])
                {   //�ҵ������Ч��
                    if (Point_Left[j] < Point_Left[i])
                    {   //���������ת�����·���߽�����Ϸ���߽������
                        Left_Inflection_Point2 = j;
                    }
                    break;
                }
            }
        }
        //������ҵ��յ�2,break
        if (Left_Inflection_Point2 != 1)
            break;
    }

    /************************��С���˷��������ֱ��***********************/
    float K_Left1, B_Left1, K_Left2, B_Left2;
    float Aver_X, Aver_Y;
    float ValidCnt = 0;
    int Sum_X=0,Sum_Y=0,Sum_XX=0,Sum_XY=0;

    //��ϵ�һ��
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

    //��ϵڶ���
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

    //ǰ��δreturn��˵�����¶ζ���ϳɹ������濪ʼ����������
    float Fit_Err1 = 0, Fit_Err2 = 0;
    uint8 cnt1 = 0, cnt2 = 0;
    float Fit_Val;
    for (i = Left_Inflection_Point2; i < Iwidth; i++)
    {
        if (Exist_Left[i])
        {
            if (i < Left_Inflection_Point1)
            {   //�ڶ���
                Fit_Val = K_Left2 * i + B_Left2;
                Fit_Err2 += ((Point_Left[i] - Fit_Val) * (Point_Left[i] - Fit_Val));
                cnt2++;
            }
            else
            {   //��һ��
                Fit_Val = K_Left1 * i + B_Left1;
                Fit_Err1 += ((Point_Left[i] - Fit_Val) * (Point_Left[i] - Fit_Val));
                cnt1++;
            }
        }
    }

    Fit_Err1 /= cnt1;
    Fit_Err2 /= cnt2;

    //���ζ���ֱ�ߣ��Ҷ�1���Ҷ�2����
    if (
        ((Fit_Err1 <= 4 && Fit_Err2 <= 1) || (Fit_Err1 <= 1 && Fit_Err2 <= 4)) &&
        ((K_Left1 < -2 && K_Left2 > 1) || (K_Left1 < 0 && K_Left2 > 2.5f))      //TODO ��Ҫ��4 1 2.5��Щ
        )
    {
        for (i = 1; i < Left_Inflection_Point1; i++)
        {
            Exist_Left[i] = 0;
        }
        for (i = 1; i < Iwidth; i++)  //TODO ͬ��
        {
            Exist_Right[i] = 0;
        }
        Lost_Right = 1;
    }
}

void CrossRightLineJudge(void)
{
    //�����������
    uint8 i, j;
    uint8 Min_Right_Line_Point = Ilength - 1;
    for (i = 1; i < Iwidth; i++)
    {
        if (Exist_Right[i] && (Min_Right_Line_Point > Point_Left[i]))
        {
            Min_Right_Line_Point = (uint8)Point_Left[i];
        }
    }
    //�������δ����1/3�򲻼��
    if (Min_Right_Line_Point > (2 * Ilength / 3))
    {
        return;
    }

    //�����ҹյ�1 2
    uint8 Right_Inflection_Point1 = 1;
    uint8 Right_Inflection_Point2 = 1;

    /************************�����߹յ�1***********************/
    for (i = Iwidth - 1; i >= 1; i--)
    {   //�������ϱ���
        if (Exist_Right[i])
        {   //ǰ���Ǹ��д����ұ߽��
            for (j = i + 1; j < Iwidth; j++)
            {   //�����������Ч��
                if (Exist_Right[j])
                {   //�ҵ������Ч��
                    if (Point_Right[j] < Point_Right[i])
                    {   //���������ת�����·��ұ߽�����Ϸ��ұ߽������
                        Right_Inflection_Point1 = j;
                    }
                    break;
                }
            }
        }
        //������ҵ��յ�1,break
        if (Right_Inflection_Point1 != 1)
        {
            break;
        }
    }

    /************************�����߹յ�2***********************/
    for (i = Right_Inflection_Point1 - 1; i >= 1; i--)
    {   //�������ϱ���
        if (Exist_Right[i])
        {   //ǰ���Ǹ��д����ұ߽��
            for (j = i + 1; j < Iwidth; j++)
            {   //�����������Ч��
                if (Exist_Right[j])
                {   //�ҵ������Ч��
                    if (Point_Right[j] > Point_Right[i])
                    {   //���������ת�����·��ұ߽�����Ϸ��ұ߽������
                        Right_Inflection_Point2 = j;
                    }
                    break;
                }
            }
        }
        //������ҵ��յ�2,break
        if (Right_Inflection_Point2 != 1)
            break;
    }

    /************************��С���˷��������ֱ��***********************/
    float K_Right1, B_Right1, K_Right2, B_Right2;
    float Aver_X, Aver_Y;
    float ValidCnt = 0;
    int Sum_X=0,Sum_Y=0,Sum_XX=0,Sum_XY=0;

    //��ϵ�һ��
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

    //��ϵڶ���
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

    //ǰ��δreturn��˵�����¶ζ���ϳɹ������濪ʼ����������
    float Fit_Err1 = 0, Fit_Err2 = 0;
    uint8 cnt1 = 0, cnt2 = 0;
    float Fit_Val;
    for (i = Right_Inflection_Point2; i < Iwidth; i++)
    {
        if (Exist_Right[i])
        {
            if (i < Right_Inflection_Point1)
            {   //�ڶ���
                Fit_Val = K_Right2 * i + B_Right2;
                Fit_Err2 += ((Point_Left[i] - Fit_Val) * (Point_Left[i] - Fit_Val));
                cnt2++;
            }
            else
            {   //��һ��
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
        ((K_Right1 < -2 && K_Right2 > 1) || (K_Right1 < 0 && K_Right2 > 2.5f))  //TODO ͬ��
    )
    {
        for (uint8 i = 1; i < Right_Inflection_Point1; i++)
        {
            Exist_Right[i] = 0;
        }
        for (uint8 i = 1; i < Iwidth; i++) //TODO ������
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
            {   //�����Ƿ�һֱ����
                Is_Left_All_To_Right = 0;
            }
            MinLeft_Row = i;
        }
        if (Exist_Right[i])
        {
            Right_ValidCnt++;
            if (Exist_Right[i] && (Point_Right[i] > Point_Right[i + 1])) //TODO ͬ��
            {   //�����Ƿ�һֱ����
                Is_Right_All_To_Left = 0;
            }
            MinRight_Row = i;
        }
    }
    /************************************����*****************************************/
    if (Is_Left_All_To_Right && (Left_ValidCnt >= (Iwidth - 1)*0.5 )) //TODO 0.5�����Ÿ�С
    {
        //����ĩ�㿪ʼ����(��������)dfs,���Ϊ��ߵ�
        //����>��>��
        //ѧ����������߼�˳���ע�Ͳ�ͬ
        short Point_I = MinLeft_Row, Point_J = Point_Left[MinLeft_Row];
        while (1)
        {
            if (IsEdgePoint(Point_I - 1, Point_J + 1))
            {   //����
                Point_I -= 1;
                Point_J += 1;

                Point_Left[Point_I] = Point_J;
                Exist_Left[Point_I] = 1;
            }
            else if (IsEdgePoint(Point_I - 1, Point_J))
            {   //��
                Point_I -= 1;

                Point_Left[Point_I] = Point_J;
                Exist_Left[Point_I] = 1;
            }
            else if (IsEdgePoint(Point_I, Point_J + 1))
            {   //��
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

    /************************************����*****************************************/
    if (Is_Right_All_To_Left && (Right_ValidCnt >= 0.5 * (Iwidth - 1))) // TODO ͬ��
    {
        //����ĩ�㿪ʼ����(��������)dfs,���Ϊ�ұߵ�
        //����>��>��
        //ѧ����������߼�˳���ע�Ͳ�ͬ��
        short Point_I = MinRight_Row, Point_J = Point_Right[MinRight_Row];
        while (1)
        {
            if (IsEdgePoint(Point_I - 1, Point_J - 1))
            {   //����
                Point_I -= 1;
                Point_J -= 1;

                Point_Right[Point_I] = Point_J;
                Exist_Right[Point_I] = 1;
            }
            else if (IsEdgePoint(Point_I - 1, Point_J))
            {   //��
                Point_I -= 1;

                Point_Right[Point_I] = Point_J;
                Exist_Right[Point_I] = 1;
            }
            else if (IsEdgePoint(Point_I, Point_J - 1))
            {   //��
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
    //����Խ�籣��
    if (i < 2 || i >= Iwidth - 3)
    {
        return 0;
    }
    if (j < 2 || j >= Ilength - 3)
    {
        return 0;
    }
    //�������Ǻڵ�0
    if (!imagin_cam[i][j])
    {
        return 0;
    }
    //�������Ҳ���ȫ�ǰ׵�1
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

                        for (kk = j + 1; kk <= i; kk++) //���������ֱ�߹�ʽ�����ȱ�ĵ�(�ȳ˺������ֹ��ִ�г���ʹ��Ϊ0)
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
    {   //�Ҷ���
        uint8 Left_ValidCnt = 0; //��߽�����
        short Left_Col_Max = 0;  //��ʾ��߽�������Զ���쵽������
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
    {   //����
        uint8 Right_ValidCnt = 0; //�ұ߽�����
        short Right_Col_Min = Ilength - 1;  //��ʾ�ұ߽�������Զ���쵽������
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
    {   //������
        uint8 Road_Len_Of_Image;//ͼƬ�����㵽��·��
        uint8 Road_Len_Of_Array;//����Road_Len_K * i + Road_Len_B����õ���·��

        for (i = Iwidth >> 1; i < Iwidth; i++)
        {   //����������ұߵ����
            if (Exist_Left[i] && Exist_Right[i])
            {
                Road_Len_Of_Image = Point_Right[i] - Point_Left[i];
                Road_Len_Of_Array = planc.b_nihe * i + planc.k_nihe;

                //����ʵ��·��Ӧȫ�������roadLen - 10
                if ((Road_Len_Of_Image < Ilength - 5) && (Road_Len_Of_Image < Road_Len_Of_Array - 14)) //TODO ����
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
            {   //�������߲���
                Std_Point = 47 + (planc.k_nihe * i + planc.b_nihe) ;
                if ((Std_Point < 141) && (Point_Right[i] < 141))
                {
                    line_f->ValidCnt++;
                    Sum_Err += (Std_Point - Point_Right[i]);
                }
            }
        }

        //�����㹻,��Error����
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
                Center_Fix = Aver_Err * 0.5;// TODO ����
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
        RepairWholeLine(line_f);//�޸�����
    }
    else if (Lost_Right)
    {

        line_f->ValidCnt = 0;
        float Sum_Err = 0;
        float Std_Point;

        for (i = 58; i > 2; i--)
        {
            if (Exist_Left[i])
            {   //�������߲���
                Std_Point = Ilenghalf - (planc.k_nihe * i + planc.b_nihe) ;
                if ((Std_Point > -47) && (Point_Left[i] > -47))
                {
                    line_f->ValidCnt++;
                    Sum_Err += (Point_Left[i] - Std_Point);
                }
            }
        }

        //�����㹻,��Error����
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

        //����
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
        RepairWholeLine(line_f);//�޸�����
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

            else if ((Exist_Left[i]) && (i < Iwidth * 0.7)) //���Ҷ�����һ������ʱ ��δ������
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
        RepairWholeLine(line_f);//�޸�����

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

