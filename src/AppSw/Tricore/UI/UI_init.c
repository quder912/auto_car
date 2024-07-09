#include "UI_init.h"
#include "W25Q.h"
#include "LQ_QSPI.h"
#include "stdio.h"


//#define First_download 0


uint16 testspinum;


void Test_SPI(void)
{
    char TXT[110]={70,53,54,58,61,62,62,62,62};
    char get[110]={0};
    static unsigned int i=1048576;
    W25QXX_Write((unsigned char*)TXT,i,100);
    delayms(500);
    W25QXX_Read((unsigned char*)get,i,100);

  //  OLED_P6x8Str(2,2,TXT);
  //  OLED_P6x8Str(2,3,get);

    delayms(1000);

    sprintf(get," num: %d",i);
    OLED_P6x8Str(2,5,get);
    i+=256;


}
