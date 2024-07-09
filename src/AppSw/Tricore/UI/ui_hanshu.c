#include "W25Q.h"
#include "ui_hanshu.h"



uint8 key_scan(uint8 KEY)                   // 按键扫描  KEY: 需要扫描(检测)的按键
{
    static uint16 cnt = 0;
    sint16 i;
    uint8 value = 0;
    if((KEY & key_up) == key_up)
    {
        if(PIN_Read(key_up_IO) ==  0)
        {
            delayms(20);
            if(PIN_Read(key_up_IO) ==  0)
            {
                i = 450-cnt;
                while(PIN_Read(key_up_IO) ==  0 && (i--)) delayms(1);
                if(i <= 0) cnt += 30;
                else cnt = 0;
                if(cnt > 250) cnt = 250;
                value |= key_up;
            }
        }
    }
    if((KEY & key_down) == key_down)
    {
        if(PIN_Read(key_down_IO) ==  0)
        {
            delayms(20);
            if(PIN_Read(key_down_IO) ==  0)
            {
                i = 450 - cnt;
                while(PIN_Read(key_down_IO) ==  0 && (i--)) delayms(1);
                    if(i <= 0) cnt += 30;
                else cnt = 0;
                if(cnt > 250) cnt = 250;
                value |= key_down;
            }
        }
    }
    if((KEY & key_left) == key_left)
    {
        if(PIN_Read(key_left_IO) ==  0)
        {
            delayms(20);
            if(PIN_Read(key_left_IO) ==  0)
            {
                i = 400 - cnt;
                while(PIN_Read(key_left_IO) ==  0 && (i--)) delayms(1);
                if(i <= 0) cnt += 25;
                else cnt = 0;
                if(cnt > 370) cnt = 370;
                value |= key_left;
            }
        }
    }
    if((KEY & key_right) == key_right)
    {
        if(PIN_Read(key_right_IO) ==  0)
        {
            delayms(50);
            if(PIN_Read(key_right_IO) ==  0)
            {
                i = 400 - cnt;
                while(PIN_Read(key_right_IO) ==  0 && (i--)) delayms(1);
                if(i <= 0) cnt += 25;
                else cnt = 0;
                if(cnt > 370) cnt = 370;
                value |= key_right;
            }
        }
    }
    if((KEY & key_in) == key_in)
    {
        if(PIN_Read(key_in_IO) ==  0)
        {
            delayms(50);
            if(PIN_Read(key_in_IO) ==  0)
            {
                while(PIN_Read(key_in_IO) ==  0) delayms(1);
                delayms(10);
                cnt = 0;
                value |= key_in;
            }
        }
    }
    if((KEY & key_out) == key_out)
    {
        if(PIN_Read(key_out_IO) == 0)
        {
            delayms(50);
            if(PIN_Read(key_out_IO) == 0)
            {
                while(PIN_Read(key_out_IO) ==  0 ) delayms(1);
                delayms(10);
                cnt = 0;
                value |= key_out;
            }
        }
    }

    return value;
}

/*
 * @note 我加了mode
 */
void OLED_Write68(unsigned char x,unsigned char y, char ch[] , _Bool mode)
{
//    while(using_OLED)
//    {
//        __asm("NOP") ;
//    }
//    using_OLED=1;
    unsigned char c=0,i=0,j=0;
    while (ch[j]!='\0')
    {
        c =ch[j]-32;
        if(x>126){x=0;y++;}
        OLED_Set_Pos(x,y);
        if(mode)
        {
            for(i=0;i<6;i++)
                OLED_WrDat(F6x8[c][i]);
        }
        else
        {
            for(i=0;i<6;i++)
                OLED_WrDat(~F6x8[c][i]);
        }

        x+=6;
        j++;
    }
//    using_OLED=0;
}
