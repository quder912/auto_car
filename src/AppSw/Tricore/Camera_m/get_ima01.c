#include "get_ima01.h"
#include "Came.h"

short white_num;
Getinfo runinfo;

void Get_Image_grey(void)
{
    short i = 0, j = 0, row = 0, line = 0 ;
           unsigned short data_i;

    for (i = 0; i < IMAGEH; i += 2)          //���۸� 120 / 2  = 60��
    {
        for (j = 0; j < IMAGEW; j += 2)     //���ۿ�188 / 2  = 94��
        {
            data_i=Image_Data[i][j]+Image_Data[i+1][j]+Image_Data[i][j+1]+Image_Data[i+1][j+1];
            imagin_init[row][line] = (uint8)(data_i>>2);
            line++;
        }
        line = 0;
        row++;
    }
    return;
}



void Get_Image01(unsigned char mode)
{
    unsigned short i = 0, j = 0;
    unsigned short Threshold = 0;
    unsigned long tv = 0;
    white_num=0;

    if (mode == 0)
    {
        Threshold = Getdajin()+1;
    }
    else if (mode == 1)
    {
        for (i = 0; i < LCDH; i++)
        {
            for (j = 0; j < LCDW; j++)
            {
                tv += imagin_init[i][j];
            }
        }
        Threshold =(unsigned short)(tv /( Iwidth * Ilength));   //��ƽ��ֵ,����Խ��ԽС��ȫ��Լ35��������ĻԼ160��һ������´�Լ100
        Threshold = Threshold + 20;      //�˴���ֵ���ã����ݻ����Ĺ������趨
    }
    else if (mode == 2)
    {
        Threshold = 80;                          //�ֶ�������ֵ
     //   lq_sobel(imagin_init, imagin_cam, (unsigned char) Threshold); //TODO �������Boolָ��

        return;

    }
    else if (mode == 3)
    {
     //   lq_sobelAutoThreshold(imagin_init, imagin_cam);  //��̬������ֵ
        return;
    }
    for (i = 0; i < Iwidth; i++)
    {
        for (j = 0; j < Ilength; j++)
        {
            if (imagin_init[i][j] > Threshold)
            {
                imagin_cam[i][j] = 1;
                white_num++;
            }
            else
            {
                imagin_cam[i][j] = 0;
            }
        }
    }
}










unsigned short Getdajin(void)  //TODO _Bool����ָ��ת���������� ����ȫ�ֱ���
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
    signed short MinValue, MaxValue;
    unsigned short Threshold = 0;
    unsigned char HistoGram[256];

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ

    for (j = 0; j < Iwidth; j++)
    {
        for (i = 0; i < Ilength; i++)
        {
            HistoGram[imagin_init[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //��ȡ���Ҷȵ�ֵ

/*
    if (MaxValue == MinValue)
        return MaxValue;         // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)
        return MinValue;        // ͼ����ֻ�ж�����ɫ
*/

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  ��������

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //�Ҷ�ֵ����
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //ǰ�����ص���
        PixelFore = Amount - PixelBack;           //�������ص���
        OmegaBack = (float) PixelBack / Amount;   //ǰ�����ذٷֱ�
        OmegaFore = (float) PixelFore / Amount;   //�������ذٷֱ�
        PixelshortegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //�����Ҷ�ֵ
        MicroBack = (float) PixelshortegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
        MicroFore = (float) PixelshortegralFore / PixelFore;   //�����ҶȰٷֱ�
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //������䷽��
        if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //���������ֵ;
}













