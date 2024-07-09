//#include "about_int.h"
#include "hanshu.h"





/*
 * @note 只支持点按
 */
unsigned int scan_key(GPIO_Name_t Pin)
{
    unsigned int keybit=1;
    if(PIN_Read(Pin)==0)
        {
            delayms(25);
            if(PIN_Read(Pin)==0)
            {
                delayms(30);
                while(PIN_Read(Pin)==0);
                keybit=0;
            }
        }
    return keybit;
}


/*
 * @use 判断斑马线
 */
uint8 Garage_check(GPIO_Name_t Pin1 ,GPIO_Name_t Pin2)
{
    uint8 IObit=0;
       IObit = (PIN_Read(Pin1)==1||PIN_Read(Pin2)==1);
    return IObit;
}


/*
 * @return 1/0
 */
uint8 num2char(float num, char *array)
{
    sint8 i = 0, j = 0, k = 0;
    char table[10] = "\0";
    uint32 t;
    if(num != 0)
    {
        if(num < 0)
        {
            array[j++] = '-';
            num = -num;
        }
        if(num < 1) array[j++] = '0';
        if(num - (uint32)num > 0)
        {
            num *= 1000000;
            k += 6;
        }
        t = (sint32)num;
        while(t % 10 == 0 && k)
        {
            k--;
            t /= 10;
        }
        while(t)
        {
            table[i++] = t % 10 + '0';
            t /= 10;
        }
        while(k > i) table[i++] = '0';
        for(i = i - 1; i >= 0; i--)
        {
            if(i == k - 1) array[j++] = '.';
            array[j++] = table[i];
        }
        array[j] = '\0';
        return j;
    }
    else
    {
        array[0] = '0';
        array[1] = '\0';
        return 0;
    }
}





/*
 * @use 蜂鸣器响1
 */
void buzzer(_Bool bu)
{
    if(bu!=0)
    PIN_Write(P11_3,1);
    else
        PIN_Write(P11_3,0);

}






