#include "forreport.h"
#include "Came.h"
#include "cam_findmid.h"
#include <LQ_UART.h>
#include <LQ_GPIO.h>


void CAMERA_Reprot_switch(uint8 mode)
{
    short j, i;
    unsigned char imah[120][188];
    unsigned char pianyi;

    memset(imah,0,sizeof(imah));

    switch(mode)
    {
        case 0: //ԭͼ
        {
            for(i=0;i<120;i++)
            {
                for(j=0;j<188;j++)
                {
                    imah[i][j]=Image_Data[i][j];
                }
            }
            imah[20][93] = 0xd0;
            imah[20][94] = 0xd0;
            imah[40][93] = 0xd0;
            imah[40][94] = 0xd0;
            imah[60][93] = 0xd0;
            imah[60][94] = 0xd0;
            imah[80][93] = 0xd0;
            imah[80][94] = 0xd0;
            imah[100][93] = 0xd0;
            imah[100][94] = 0xd0;

            imah[10][93] = 0xd0;
            imah[10][94] = 0xd0;
            imah[30][93] = 0xd0;
            imah[30][94] = 0xd0;
            imah[50][93] = 0xd0;
            imah[50][94] = 0xd0;
            imah[70][93] = 0xd0;
            imah[70][94] = 0xd0;
            imah[90][93] = 0xd0;
            imah[90][94] = 0xd0;
            imah[110][93] = 0xd0;
            imah[110][94] = 0xd0;

            imah[15][94] = 0x09;
            imah[25][94] = 0x09;
            imah[35][94] = 0x09;
            imah[45][94] = 0x09;
            imah[55][94] = 0x09;
            imah[65][94] = 0x09;
            imah[75][94] = 0x09;
            imah[85][94] = 0x09;
            imah[95][94] = 0x09;
            imah[105][94] = 0x09;
            imah[115][94] = 0x09;
            imah[5][94] = 0x09;



            break;
        }

        case 1:  //�Ҷ�
        {

            imah[29][141]=imah[29][46]=imah[90][141]=imah[90][46]=254;
            for(i=0;i< 60; i++)
            {
                for (j = 0; j < 94; j++)
                {
                    imah[i+30][j+47]=imagin_init[i][j];
                }
            }break;
        }

        case 2:  //�ڰ�
        {
            for(i=47;i <141; i++)
            {
               imah[29][i]=50;
               imah[i-30][93]=50;
               imah[i-30][94]=50;
            }

            imah[29][141]=imah[29][46]=imah[90][141]=imah[90][46]=170;

            imah[46][46]=imah[46][141]=254; //����

            imah[29][47+31]=imah[29][47+68]=imah[28][78]=imah[28][115]=253; //�Ͼ�
            imah[37][46]=imah[37][45]=imah[37][141]=imah[37][142]=253; //�¾�

            for(i=0;i < 60; i++)
            {
                for (j = 0; j < 94; j++)
                {
                    imah[i+30][j+47]=(imagin_cam[i][j])?200:0;
                }
            }
            break;
        }


        case 3:   //��before
        {

            for(j=47;j<141;j++)
            {
                imah[60][j]=80;
                imah[0][j]=80;
            }

            for(i=0;i < 60; i++)
            {
                imah[i][93]=80;
                imah[i][94]=80;

                if(Exist_Left[i])
                {
                    imah[i][Point_Left[i]+47]=254;
                }
                if(Exist_Right[i])
                {
                    imah[i][Point_Right[i]+47]=254;
                }
            }
            break;
        }

        case 4: //��after
        {
            for(j=47;j<141;j++)
            {
                imah[60][j]=30;
                imah[0][j]=30;
            }

            for(i=0;i < 60; i++)
            {
                imah[i][93]=30;
                imah[i][94]=30;

                if(Exist_Left[i])
                {
                    pianyi=Point_Left[i]+47;
                    imah[i][pianyi]=70;
                }

               if(Exist_Right[i])
               {
                   pianyi=Point_Right[i]+47;
                   imah[i][pianyi]=130;
               }

               if(Exist_Center[i])
               {
                   pianyi=Point_Center[i]+47;
                   imah[i][pianyi]=240;
               }
            }
            break;
        }

    }

    UART_PutChar(UART0, 0xfe);  //֡ͷ
    UART_PutChar(UART0, 0xef);  //֡ͷ

    for (i = 0; i < 120; i++)
    {
        for (j = 0; j < 188; j++)
        {
            if (imah[i][j] == 0xfe)  //��ֹ������֡β
            {
                imah[i][j] = 0xff;
            }
            UART_PutChar(UART0, imah[i][j]); //��������

        }
    }
    UART_PutChar(UART0, 0xef);  //֡β
    UART_PutChar(UART0, 0xfe);  //֡β

}
