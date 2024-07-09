/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC3xxxA核心板
【编    写】chiusir
【E-mail】chiusir@163.com
【软件版本】V1.1 版权所有，单位使用请先联系授权
【最后更新】2020年10月28日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】AURIX Development Studio1.2.2及以上版本
【Target 】 TC3xxxA
【Crystal】 20.000Mhz
【SYS PLL】 300MHz
________________________________________________________________
基于iLLD_1_0_1_11_0底层程序,
使用例程的时候，建议采用没有空格的英文路径，
=================================================================
程序配套视频地址：https://space.bilibili.com/95313236
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <LQ_GPIO.h>
#include <LQ_STM.h>
#include <LQ_OLED096.h>

//龙邱OLED SPI口模块管脚顺序
#define OLED_CK    P20_11     //OLED CK管脚
#define OLED_DI	   P20_14     //OLED DI管脚
#define OLED_RST   P20_10     //OLED RST管脚
#define OLED_DC	   P20_12     //OLED DC管脚
#define OLED_CS	   P00_0     //OLED CS管脚 默认拉低，可以不用

#define OLED_DC_H  PIN_Write(OLED_DC, 1);
#define OLED_DC_L  PIN_Write(OLED_DC, 0);

#define OLED_RST_H PIN_Write(OLED_RST, 1);
#define OLED_RST_L PIN_Write(OLED_RST, 0);

#define OLED_DI_H  PIN_Write(OLED_DI, 1);
#define OLED_DI_L  PIN_Write(OLED_DI, 0);

#define OLED_CK_H  PIN_Write(OLED_CK, 1);
#define OLED_CK_L  PIN_Write(OLED_CK, 0);


#define X_WIDTH 132
#define Y_WIDTH 64

_Bool using_OLED=0;

/*!
  * @brief    OLED初始化
  *
  * @param    无
  *
  * @return   无
  *
  * @note     如果修改管脚 需要修改初始化的管脚 
  *
  * @see      OLED_Init();
  *
  * @date     2019/6/13 星期四
  */
void OLED_Init(void)
{ 
    PIN_InitConfig(OLED_DC, PIN_MODE_OUTPUT, 0);
    PIN_InitConfig(OLED_RST, PIN_MODE_OUTPUT, 0);
	PIN_InitConfig(OLED_DI, PIN_MODE_OUTPUT, 0);
	PIN_InitConfig(OLED_CK, PIN_MODE_OUTPUT, 0);
	PIN_InitConfig(OLED_CS, PIN_MODE_OUTPUT, 0);
    
	OLED_CK_H;
    //OLED_CS=1;	//预制SLK和SS为高电平
	OLED_RST_L;
    delayms(50);
    OLED_RST_H;
    
    OLED_WrCmd(0xae);//--turn off oled panel
    OLED_WrCmd(0x00);//---set low column address
    OLED_WrCmd(0x10);//---set high column address
    OLED_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WrCmd(0x81);//--set contrast control register
    OLED_WrCmd(0xcf); // Set SEG Output Current Brightness
    OLED_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    OLED_WrCmd(0xa6);//--set normal display
    OLED_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
    OLED_WrCmd(0x3f);//--1/64 duty
    OLED_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WrCmd(0x00);//-not offset
    OLED_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
    OLED_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WrCmd(0xd9);//--set pre-charge period
    OLED_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WrCmd(0xda);//--set com pins hardware configuration
    OLED_WrCmd(0x12);
    OLED_WrCmd(0xdb);//--set vcomh
    OLED_WrCmd(0x40);//Set VCOM Deselect Level
    OLED_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WrCmd(0x02);//
    OLED_WrCmd(0x8d);//--set Charge Pump enable/disable
    OLED_WrCmd(0x14);//--set(0x10) disable
    OLED_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
    OLED_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7)
    OLED_WrCmd(0xaf);//--turn on oled panel
    OLED_CLS();      //初始清屏
    OLED_Set_Pos(0,0);
    
} 
/** 6*8字库  */
const unsigned char F6x8[][6] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // sp
    { 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00 },   // !
    { 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 },   // "
    { 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14 },   // #
    { 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12 },   // $
    { 0x00, 0x62, 0x64, 0x08, 0x13, 0x23 },   // %
    { 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 },   // &
    { 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 },   // '
    { 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00 },   // (
    { 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00 },   // )
    { 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14 },   // *
    { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 },   // +
    { 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00 },   // ,
    { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 },   // -
    { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 },   // .
    { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 },   // /
    { 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E },   // 0
    { 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 },   // 1
    { 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 },   // 2
    { 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 },   // 3
    { 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 },   // 4
    { 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 },   // 5
    { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 },   // 6
    { 0x00, 0x01, 0x71, 0x09, 0x05, 0x03 },   // 7
    { 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 },   // 8
    { 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E },   // 9
    { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 },   // :
    { 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 },   // ;
    { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 },   // <
    { 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 },   // =
    { 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 },   // >
    { 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 },   // ?
    { 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E },   // @
    { 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C },   // A
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 },   // B
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 },   // C
    { 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C },   // D
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 },   // E
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 },   // F
    { 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A },   // G
    { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F },   // H
    { 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 },   // I
    { 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 },   // J
    { 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 },   // K
    { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 },   // L
    { 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F },   // M
    { 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F },   // N
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E },   // O
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 },   // P
    { 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E },   // Q
    { 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 },   // R
    { 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 },   // S
    { 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 },   // T
    { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F },   // U
    { 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F },   // V
    { 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F },   // W
    { 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 },   // X
    { 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 },   // Y
    { 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 },   // Z
    { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },   // [
    { 0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55 },   // 55
    { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 },   // ]
    { 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 },   // ^
    { 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 },   // _
    { 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 },   // '
    { 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 },   // a
    { 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 },   // b
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 },   // c
    { 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F },   // d
    { 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 },   // e
    { 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 },   // f
    { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C },   // g
    { 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 },   // h
    { 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 },   // i
    { 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00 },   // j
    { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 },   // k
    { 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 },   // l
    { 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 },   // m
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 },   // n
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 },   // o
    { 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18 },   // p
    { 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC },   // q
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 },   // r
    { 0x00, 0x48, 0x54, 0x54, 0x54, 0x20 },   // s
    { 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 },   // t
    { 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C },   // u
    { 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C },   // v
    { 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C },   // w
    { 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 },   // x
    { 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C },   // y
    { 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 },   // z
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x14 }    // horiz lines
};
/* MDK AC6编译器不支持 GB3212编码的中文 因此会出现警告 不影响正常使用 忽略即可 */
/** 简单的汉字字库 */


/** 14*16 简单的汉字字库*/
const unsigned char F14x16[] = {
    0x20,0x20,0x20,0x20,0xFE,0x00,0x00,0x00,0xFE,0x80,0x40,0x20,0x10,0x00,
    0x10,0x10,0x08,0x04,0x7F,0x00,0x00,0x00,0x3F,0x40,0x40,0x40,0x70,0x00,//北0
    0x08,0x08,0xE8,0x28,0x28,0x2A,0x2C,0x28,0x28,0x28,0xE8,0x08,0x08,0x00,
    0x00,0x20,0x13,0x0A,0x42,0x42,0x7E,0x02,0x02,0x0A,0x13,0x20,0x00,0x00,//京1
    0x00,0xF0,0x90,0x90,0x90,0x90,0xFE,0x90,0x90,0x90,0x90,0xF0,0x00,0x00,
    0x00,0x0F,0x04,0x04,0x04,0x04,0x3F,0x44,0x44,0x44,0x44,0x47,0x70,0x00,//电2
    0x80,0x82,0x82,0x82,0x82,0x82,0xC2,0xA2,0x92,0x8A,0x86,0x82,0x80,0x00,
    0x00,0x00,0x00,0x00,0x40,0x40,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//子3
    0x40,0x48,0x48,0xF8,0x44,0x44,0x20,0xC4,0x18,0x00,0xFE,0x00,0x00,0x00,
    0x08,0x04,0x03,0x7F,0x02,0x04,0x02,0x02,0x02,0x02,0x7F,0x01,0x01,0x00,//科4
    0x10,0x10,0xFE,0x10,0x00,0x90,0x90,0x90,0xFE,0x90,0x90,0x10,0x00,0x00,
    0x42,0x42,0x7F,0x01,0x40,0x40,0x23,0x14,0x08,0x14,0x23,0x40,0x40,0x00,//技5
    0x04,0xFC,0x24,0x24,0xFC,0x04,0x00,0xFC,0x04,0x04,0x04,0xFC,0x00,0x00,
    0x10,0x1F,0x11,0x09,0x7F,0x08,0x40,0x33,0x02,0x02,0x02,0x13,0x60,0x00,//职6
    0x10,0x60,0x80,0x00,0xFE,0x00,0x00,0x00,0xFE,0x00,0x80,0x70,0x00,0x00,
    0x40,0x40,0x43,0x40,0x7F,0x40,0x40,0x40,0x7F,0x42,0x41,0x40,0x40,0x00,//业7
    0x70,0x10,0x92,0x94,0x90,0x92,0x94,0x90,0x98,0x94,0x12,0x10,0x70,0x00,
    0x04,0x04,0x04,0x04,0x44,0x44,0x7E,0x06,0x05,0x04,0x04,0x04,0x04,0x00,//学8
    0xFC,0x04,0x64,0x9C,0x00,0x18,0x48,0x48,0x4A,0x4C,0x48,0x48,0x18,0x00,
    0x7F,0x08,0x08,0x07,0x40,0x22,0x12,0x0E,0x02,0x02,0x3E,0x42,0x62,0x00,//院9
    0x20,0x20,0x20,0x20,0xA0,0x7E,0x20,0xE0,0x24,0x28,0x20,0xA0,0x20,0x00,
    0x40,0x20,0x10,0x0C,0x23,0x20,0x10,0x3F,0x44,0x42,0x41,0x40,0x70,0x00,//龙10
    0x00,0xFC,0x44,0x44,0x44,0xC2,0x42,0x00,0xFC,0x04,0x44,0xB4,0x0C,0x00,
    0x10,0x1F,0x10,0x10,0x08,0x0F,0x08,0x00,0x7F,0x00,0x10,0x10,0x0F,0x00,//邱11
    0x28,0xA6,0x64,0x3C,0x64,0xA4,0x20,0xFC,0x84,0x84,0x84,0xFC,0x00,0x00,
    0x01,0x00,0x7E,0x2A,0x2A,0x2A,0x2A,0x2A,0x2A,0x2A,0x7E,0x00,0x00,0x00,//智12
    0x10,0x98,0x94,0x92,0x90,0x98,0x30,0x00,0x3E,0x48,0x48,0x44,0x74,0x00,
    0x00,0x7F,0x0A,0x0A,0x4A,0x7F,0x00,0x00,0x3F,0x44,0x44,0x42,0x71,0x00,//能13
    0x10,0x90,0xFE,0x90,0x10,0x00,0xFC,0x04,0x04,0x04,0xFC,0x00,0x00,0x00,
    0x02,0x01,0x7F,0x00,0x41,0x30,0x0F,0x00,0x00,0x00,0x3F,0x40,0x70,0x00,//机14
    0x80,0x9E,0x92,0x92,0x92,0x9E,0xC0,0x9E,0x92,0xB2,0xD2,0x9E,0x80,0x00,
    0x04,0x7C,0x4A,0x4A,0x49,0x78,0x00,0x78,0x49,0x4A,0x4A,0x7C,0x04,0x00,//器15
    0x00,0x00,0x00,0x00,0x00,0x80,0x7E,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
    0x40,0x40,0x20,0x10,0x0C,0x03,0x00,0x03,0x0C,0x10,0x20,0x40,0x40,0x00,//人16
    0x40,0x42,0xCC,0x00,0x84,0x6C,0x54,0x4C,0xD4,0x44,0x52,0x4E,0x02,0x00,
    0x40,0x20,0x1F,0x20,0x42,0x5A,0x52,0x52,0x5F,0x52,0x52,0x5A,0x42,0x00,//遥17
    0x10,0x10,0xFE,0x10,0x00,0x98,0x48,0x2A,0x0C,0x08,0x28,0x48,0x98,0x00,
    0x42,0x42,0x7F,0x01,0x00,0x40,0x42,0x42,0x7E,0x42,0x42,0x42,0x40,0x00,//控18
    0x80,0x9E,0x92,0x92,0x92,0x9E,0xC0,0x9E,0x92,0xB2,0xD2,0x9E,0x80,0x00,
    0x04,0x7C,0x4A,0x4A,0x49,0x78,0x00,0x78,0x49,0x4A,0x4A,0x7C,0x04,0x00,//器19
    0x10,0x10,0xFE,0x10,0x50,0x48,0x58,0xEA,0x4C,0x68,0x58,0x48,0x40,0x00,
    0x42,0x42,0x7F,0x01,0x42,0x4A,0x2E,0x13,0x12,0x2A,0x26,0x42,0x02,0x00,//接20
    0x00,0xF8,0x00,0x00,0xFE,0x00,0x40,0x20,0xDE,0x10,0x10,0xF0,0x10,0x00,
    0x00,0x07,0x02,0x01,0x7F,0x00,0x40,0x20,0x17,0x08,0x16,0x21,0x40,0x00,//收21
    0x00,0x30,0x2C,0x20,0xA0,0x70,0x2E,0x20,0x20,0x20,0x24,0x28,0x20,0x00,
    0x10,0x08,0x44,0x42,0x41,0x27,0x29,0x11,0x29,0x25,0x43,0x40,0x40,0x00,//发22
    0x80,0x84,0x98,0x00,0x80,0x92,0x94,0x90,0xF0,0x90,0x94,0x92,0x80,0x00,
    0x40,0x20,0x1F,0x20,0x40,0x50,0x48,0x46,0x41,0x42,0x44,0x58,0x40,0x00,//送23
    0x10,0x10,0xFE,0x10,0x00,0xB8,0x88,0x88,0xEA,0x8C,0x88,0x88,0xB8,0x00,
    0x42,0x42,0x7F,0x01,0x00,0x40,0x44,0x27,0x18,0x08,0x14,0x23,0x40,0x00,//按24
    0x20,0x50,0xCE,0x48,0x00,0xC8,0xB8,0x20,0xA8,0xFE,0xA8,0xF8,0x20,0x00,
    0x02,0x02,0x3F,0x52,0x28,0x10,0x2F,0x20,0x4A,0x7F,0x4A,0x4A,0x48,0x00,//键25
    0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x19,0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//：26
};



/*!
  * @brief    简单延时函数
  *
  * @param    Del_1ms ：延时时间
  *
  * @return   无
  *
  * @note     无
  *
  * @see      lq_oled_short_delay(1);
  *
  * @date     2019/6/13 星期四
  */
void lq_oled_short_delay(unsigned int Del)	//
{
  while(Del--)
  {
	  __asm("NOP");

  }
}
/*!
  * @brief    OLED画点命令
  *
  * @param    data ：要画的点
  *
  * @return   无
  *
  * @note     无
  *
  * @see      内部调用
  *
  * @date     2019/6/13 星期四
  */
void OLED_WrDat(unsigned char data)
{
    unsigned char i=8;
    //OLED_CS=0;;
    OLED_DC_H;;
    OLED_CK_L;;
    lq_oled_short_delay(1);
    while(i--)
    {
        if(data&0x80){OLED_DI_H;}
        else{OLED_DI_L;}
        OLED_CK_H;
        lq_oled_short_delay(1);
        //asm("nop");            
        OLED_CK_L;
        data<<=1;    
    }
    //OLED_CS=1;
}

/*!
  * @brief    OLED命令
  *
  * @param    cmd ：命令
  *
  * @return   无
  *
  * @note     无
  *
  * @see      内部调用
  *
  * @date     2019/6/13 星期四
  */
void OLED_WrCmd(unsigned char cmd)
{
    unsigned char i=8;
    
    //OLED_CS=0;;
    OLED_DC_L;
    OLED_CK_L;
    lq_oled_short_delay(1);
    while(i--)
    {
        if(cmd&0x80){OLED_DI_H;}
        else{OLED_DI_L;}
        OLED_CK_H;
        lq_oled_short_delay(1);
        OLED_CK_L;
        cmd<<=1;
    } 	
    //OLED_CS=1;
}

/*!
  * @brief    OLED设置点
  *
  * @param    x ：坐标   
  * @param    y ：坐标
  *
  * @return   无
  *
  * @note     左上角为0,0  右下角 127,63
  *
  * @see      内部调用
  *
  * @date     2019/6/13 星期四
  */
void OLED_Set_Pos(unsigned char x, unsigned char y)
{ 
    OLED_WrCmd(0xb0+y);
    OLED_WrCmd(((x&0xf0)>>4)|0x10);
    OLED_WrCmd((x&0x0f));
    return;
} 

/*!
  * @brief    OLED全亮
  *
  * @param    无   
  *
  * @return   无
  *
  * @note     无
  *
  * @see      OLED_Fill(); //OLED全亮 用于检测坏点
  *
  * @date     2019/6/13 星期四
  */
void OLED_Fill(void)
{
    unsigned char y,x;
    
    for(y=0;y<8;y++)
    {
        OLED_WrCmd(0xb0+y);
        OLED_WrCmd(0x01);
        OLED_WrCmd(0x10);
        for(x=0;x<X_WIDTH;x++)
            OLED_WrDat(0xff);
    }
}

/*!
  * @brief    OLED清屏
  *
  * @param    无  
  *
  * @return   无
  *
  * @note     无
  *
  * @see      OLED_CLS(); //清屏
  *
  * @date     2019/6/13 星期四
  */
void OLED_CLS(void)
{

    unsigned char y,x;	
    for(y=0;y<8;y++)
    {
        OLED_WrCmd(0xb0+y);
        OLED_WrCmd(0x01);
        OLED_WrCmd(0x10);
        for(x=0;x<X_WIDTH;x++)
            OLED_WrDat(0);
    }

}



/*!
  * @brief    画一个点
  *
  * @param    x  : 0-127 
  * @param    y  : 0-63
  *
  * @return   无
  *
  * @note     OLED是按字节显示的 一个字节8位，代表8个纵向像素点，
  * @note     因此如果画点 OLED_PutPixel(0,0); OLED_PutPixel(0,1); 只会显示点(0, 1) (0,0)则在画点(0,1)时清除掉
  *
  * @see      OLED_PutPixel(0, 0); //画一个点 左上角为0,0  右下角 127,63
  *
  * @date     2019/6/13 星期四
  */
void OLED_PutPixel(unsigned char x,unsigned char y)
{
    unsigned char data1;  //data1当前点的数据 
    
    OLED_Set_Pos(x,(unsigned char)(y>>3));
    data1 =(unsigned char)(0x01<<(y%8)); 	
    OLED_WrCmd((unsigned char)(0xb0+(y>>3)));
    OLED_WrCmd((unsigned char)(((x&0xf0)>>4)|0x10));
    OLED_WrCmd((unsigned char)((x&0x0f)|0x00));
    OLED_WrDat(data1);
}

/*!
  * @brief    清除8个点
  *
  * @param    x  : 0-127
  * @param    y  : 0-63
  *
  * @return   无
  *
  * @note     OLED是按字节显示的 一个字节8位，代表8个纵向像素点，
  *
  * @see      OLED_ClrDot(0, 0); //清除(0,0)-(0,7)8个点 左上角为0,0  右下角 127,63
  *
  * @date     2019/6/13 星期四
  */
void OLED_ClrDot(unsigned char x,unsigned char y)
{
    OLED_Set_Pos(x,(unsigned char)(y>>3));
    OLED_WrCmd((unsigned char)(0xb0+(y>>3)));
    OLED_WrCmd((unsigned char)(((x&0xf0)>>4)|0x10));
    OLED_WrCmd((unsigned char)((x&0x0f)|0x00));
    OLED_WrDat(0x00);
} 


/*!
  * @brief    画矩形
  *
  * @param    x1  : 0-127   左上角坐标（x1,y1）
  * @param    y1  : 0-63
  * @param    x2  : 0-127   右下角坐标（x2，y2）
  * @param    y2  : 0-63
  * @param    gif : 是否开启动画效果
  *
  * @return   无
  *
  * @note     无
  *
  * @see      OLED_Rectangle(0, 0, 127,62,0);   //绘制矩形
  *
  * @date     2019/6/13 星期四
  */
void OLED_Rectangle(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2,unsigned char gif)
{	
	unsigned char n; 
    OLED_Set_Pos(x1,y1>>3);
    for(n=x1;n<=x2;n++)
    {
        OLED_WrDat(0x01<<(y1%8));
        if(gif == 1) 	
            delayms(10);
    }  
    OLED_Set_Pos(x1,y2>>3);
    for(n=x1;n<=x2;n++)
    {
        OLED_WrDat(0x01<<(y2%8));
        if(gif == 1) 	
            delayms(10);
    }
    
    for(n = y1>>3; n <= y2>>3; n++)
    {
        OLED_Set_Pos(x1, n);
        OLED_WrDat(0xff);
        if(gif == 1) 	
            delayms(50);
    }
    
    for(n = y1>>3; n <= y2>>3; n++)
    {
        OLED_Set_Pos(x2, n);
        OLED_WrDat(0xff);
        if(gif == 1) 	
            delayms(50);
    }
}  


/*!
  * @brief    写入一组标准ASCII字符串
  *
  * @param    x  : 0-127   
  * @param    y  : 0-7
  * @param    ch : 要显示的字符串
  *
  * @return   无
  *
  * @note     无
  *
  * @see      OLED_P6x8Str(0,0,"OLED 3.");  //显示字符串
  *
  * @date     2019/6/13 星期四
  */
void OLED_P6x8Str(unsigned char x,unsigned char y, char ch[])
{
//    while(using_OLED);
//    using_OLED=1;
    unsigned char c=0,i=0,j=0;      
    while (ch[j]!='\0')
    {    
        c =ch[j]-32;
        if(x>126){x=0;y++;}
        OLED_Set_Pos(x,y);
        for(i=0;i<6;i++)     
            OLED_WrDat(F6x8[c][i]);
        x+=6;
        j++;
    }
//    using_OLED=0;
}

 
/*!
  * @brief    写入一组标准ASCII字符串
  *
  * @param    x  : 0-127   
  * @param    y  : 0-6
  * @param    ch : 要显示的字符串
  *
  * @return   无
  *
  * @note     一个字符高16  OLED高度64  
  *
  * @see      OLED_P8x16Str(0,0,"OLED 3.");  //显示字符串
  *
  * @date     2019/6/13 星期四
  */


/*!
  * @brief    输出汉字字符串
  *
  * @param    x  : 0-127   
  * @param    y  : 0-6
  * @param    ch : 要显示的字符串
  *
  * @return   无
  *
  * @note     一个字符高16  OLED高度64   字库需要自己制作
  *
  * @see      OLED_P14x16Str(0,4,"北京龙邱");   //显示汉字
  *
  * @date     2019/6/13 星期四
  */


/*!
  * @brief    显示BMP图片 最大 128×64
  *
  * @param    x0  : 0-127   左上角坐标（x0,y0）
  * @param    y0  : 0-63
  * @param    wide: 1-127   图片宽度
  * @param    high: 1-64    图片高度
  * @param    bmp : 要显示的图片
  *
  * @return   无
  *
  * @note     图片需要用软件转化
  *
  * @see      OLED_Show_BMP(0, 0, 104, 64, longqiu104x64); //画龙邱loge
  *
  * @date     2019/6/13 星期四
  */
void OLED_Show_BMP(unsigned char x0,unsigned char y0,unsigned char wide,unsigned char high,const unsigned char * bmp)
{ 	
    unsigned int ii=0;
    unsigned char x,y;
	unsigned char x1,y1;
	if(wide + x0 > 128 || high + y0 > 64)
	{
		return;  //超出显示范围
	}
	y1 = (y0+high-1)/8;
	x1 = x0 + wide;
    for(y=y0/8;y<=y1;y++)
    {
        OLED_Set_Pos(x0,y);
        for(x=x0;x<x1;x++)
        {      
            OLED_WrDat(bmp[ii++]);
        }
    }
}

/*!
  * @brief    显示龙邱图片128×64
  *
  * @param    无 
  *
  * @return   无
  *
  * @note     无
  *
  * @see      OLED_Show_LQLogo();
  *
  * @date     2019/6/13 星期四
  */





/*!
  * @brief    画边线 OV7725显示图像时 可以用这个画边线
  *
  * @param    无 
  *
  * @return   无
  *
  * @note     无
  *
  * @see      OLED_Show_Frame80();
  *
  * @date     2019/6/13 星期四
  */
void OLED_Show_Frame80(void)
{ 	
    
    OLED_Set_Pos(23,1);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(23,2);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(23,3);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(23,4);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(23,5);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(23,6);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(23,7);
    OLED_WrDat(0xFF);
    
    OLED_Set_Pos(104,1);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(104,2);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(104,3);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(104,4);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(104,5);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(104,6);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(104,7);
    OLED_WrDat(0xFF);
}

/*!
  * @brief    画边线 MT9V034显示图像时 可以用这个画边线
  *
  * @param    无 
  *
  * @return   无
  *
  * @note     无
  *
  * @see      OLED_Show_Frame94();
  *
  * @date     2019/6/13 星期四
  */
void OLED_Show_Frame94(void)
{ 	
    //OLED_Set_Pos(13,0);//第0行，第13列
    //OLED_WrDat(0xFF); //画竖线
    OLED_Set_Pos(16,1);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(16,2);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(16,3);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(16,4);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(16,5);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(16,6);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(16,7);
    OLED_WrDat(0xFF);
    
    //OLED_Set_Pos(114,0);
    //OLED_WrDat(0xFF);
    OLED_Set_Pos(111,1);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(111,2);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(111,3);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(111,4);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(111,5);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(111,6);
    OLED_WrDat(0xFF);
    OLED_Set_Pos(111,7);
    OLED_WrDat(0xFF);
}

/**
  * @brief    OLED 二值化图像显示
  *
  * @param    high ： 显示图像高度
  * @param    wide ： 显示图像宽度
  * @param    Pixle： 显示图像数据地址
  *
  * @return   无
  *
  * @note     无
  *
  * @see      OLED_Road(OLEDH, OLEDW, (unsigned char *)Pixle);
  *
  * @date     2019/6/25 星期二
  */
void OLED_Road(unsigned short high, unsigned short wide, unsigned char *Pixle)
{ 	 
    unsigned char i = 0, j = 0,temp=0;
    unsigned char wide_start = (128 - wide)/2;
    for(i=0;i<high-7;i+=8)
    {
        
        OLED_Set_Pos(wide_start,i/8+1);
        
        for(j=0;j<wide;j++) 
        { 
            temp=0;
            if(Pixle[(0+i)*wide + j]) temp|=1;
            if(Pixle[(1+i)*wide + j]) temp|=2;
            if(Pixle[(2+i)*wide + j]) temp|=4;
            if(Pixle[(3+i)*wide + j]) temp|=8;
            if(Pixle[(4+i)*wide + j]) temp|=0x10;
            if(Pixle[(5+i)*wide + j]) temp|=0x20;
            if(Pixle[(6+i)*wide + j]) temp|=0x40;
            if(Pixle[(7+i)*wide + j]) temp|=0x80;
            OLED_WrDat(temp);
        }
    }  
}



