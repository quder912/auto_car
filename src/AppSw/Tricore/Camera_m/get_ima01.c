#include "get_ima01.h"
#include "Came.h"

short white_num;
Getinfo runinfo;

void Get_Image_grey(void)
{
    short i = 0, j = 0, row = 0, line = 0 ;
           unsigned short data_i;

    for (i = 0; i < IMAGEH; i += 2)          //神眼高 120 / 2  = 60，
    {
        for (j = 0; j < IMAGEW; j += 2)     //神眼宽188 / 2  = 94，
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
        Threshold =(unsigned short)(tv /( Iwidth * Ilength));   //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
        Threshold = Threshold + 20;      //此处阈值设置，根据环境的光线来设定
    }
    else if (mode == 2)
    {
        Threshold = 80;                          //手动调节阈值
     //   lq_sobel(imagin_init, imagin_cam, (unsigned char) Threshold); //TODO 解决——Bool指针

        return;

    }
    else if (mode == 3)
    {
     //   lq_sobelAutoThreshold(imagin_init, imagin_cam);  //动态调节阈值
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










unsigned short Getdajin(void)  //TODO _Bool数组指针转换还有问题 暂用全局变量
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    signed short MinValue, MaxValue;
    unsigned short Threshold = 0;
    unsigned char HistoGram[256];

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //初始化灰度直方图

    for (j = 0; j < Iwidth; j++)
    {
        for (i = 0; i < Ilength; i++)
        {
            HistoGram[imagin_init[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

/*
    if (MaxValue == MinValue)
        return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue;        // 图像中只有二个颜色
*/

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  像素总数

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //前景像素点数
        PixelFore = Amount - PixelBack;           //背景像素点数
        OmegaBack = (float) PixelBack / Amount;   //前景像素百分比
        OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
        PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
        MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //返回最佳阈值;
}













