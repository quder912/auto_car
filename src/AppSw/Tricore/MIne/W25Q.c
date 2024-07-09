#include "LQ_QSPI.h"
#include "W25Q.h"
#include "stdio.h"
/*
 *
 * @note 此已改为QSPI
 *
 *
 *
 *
 *
 */


unsigned short W25QXX_TYPE = 0;	//默认是W25Q128

//4Kbytes为一个Sector（扇区）   4kB
//16个扇区为1个Block    64kB
//W25Q128
//容量为16M字节,共有256个Block,4096个Sector

//初始化SPI FLASH的IO口  HALA用的是硬SPI4
//关于3字节地址模式和4字节地址模式：
//w25q128：128Mb(16M) 按字节编址 地址范围是 0x000000-0xFFFFFF 地址位数占了3个字节，所以叫三字节地址模式
//w25q256: 256Mb(32M) 按字节编制 地址范围是 0x00000000-01 FF FF FF 地址位数占了4个字节，所以叫四字节地址模式(事实上地址范围在0x01000000-0xFFFFFFFF的FALSH都是四字节地址模式)
//根据非易失性状态寄存器位ADP（S17）的设置，W25Q256FV可以工作在3字节地址模式或4字节地址模式。如果ADP = 0，则器件将以3字节地址模式工作；
//否则，器件将以3字节地址模式工作。如果ADP = 1，则设备将以4字节地址模式工作。 ADP的出厂默认值为0。

void W25QXX_Init(void)
{
    char txtn[20];
    QSPI_InitConfig(QSPI2_CLK_P15_8,QSPI2_MISO_P15_7,QSPI2_MOSI_P15_6,QSPI2_CS_P20_6,5000000,1);//设置SPI模式和波特率 SPI模式：CLK空闲时刻为高电平 在第二个CLK跳边沿（下降沿）捕获数据
	W25QXX_TYPE = W25QXX_ReadID();	//读取FLASH ID.
	sprintf(txtn,"N:%05d",W25QXX_TYPE);
	OLED_P6x8Str(2,0,txtn);
	W25QXX_Wait_Busy();
//
}


//读取W25QXX的状态寄存器，W25QXX一共有3个状态寄存器
//状态寄存器1：
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:默认0,状态寄存器保护位,配合WP使用
//TB,BP2,BP1,BP0:FLASH区域写保护设置
//WEL:写使能锁定
//BUSY:忙标记位(1,忙;0,空闲)
//默认:0x00
//状态寄存器2：
//BIT7  6   5   4   3   2   1   0
//SUS   CMP LB3 LB2 LB1 (R) QE  SRP1
//状态寄存器3：
//BIT7      6    5    4   3   2   1   0
//HOLD/RST  DRV1 DRV0 (R) (R) WPS ADP ADS
//regno:状态寄存器号，范:1~3
//返回值:状态寄存器值
unsigned char W25QXX_ReadSR(unsigned char regno)
{
	unsigned char command=0;
	switch(regno)
	{
	case 1:
		command=W25X_ReadStatusReg1;    //读状态寄存器1指令
		break;
	case 2:
		command=W25X_ReadStatusReg2;    //读状态寄存器2指令
		break;
	case 3:
		command=W25X_ReadStatusReg3;    //读状态寄存器3指令
		break;
	default:
		command=W25X_ReadStatusReg1;
		break;
	}
	unsigned char txbuff[2],rebuff[2];
	txbuff[0] = command;
	txbuff[1] = 0xff;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,2);  //发送读取状态寄存器命令
	return rebuff[1];
}

//写W25QXX状态寄存器
void W25QXX_Write_SR(unsigned char regno,unsigned char sr)
{
	unsigned char command=0;
	switch(regno)
	{
	case 1:
		command=W25X_WriteStatusReg1;    //写状态寄存器1指令
		break;
	case 2:
		command=W25X_WriteStatusReg2;    //写状态寄存器2指令
		break;
	case 3:
		command=W25X_WriteStatusReg3;    //写状态寄存器3指令
		break;
	default:
		command=W25X_WriteStatusReg1;
		break;
	}
	unsigned char txbuff[2],rebuff[2];
	txbuff[0] = command;
	txbuff[1] = sr;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,2);  //发送读取状态寄存器命令
}

//W25QXX写使能
//将WEL置位
void W25QXX_Write_Enable(void)
{
	unsigned char txbuff[1],rebuff[1];
	txbuff[0] = W25X_WriteEnable;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,1);
}

//W25QXX写禁止
//将WEL清零
void W25QXX_Write_Disable(void)
{
	unsigned char txbuff[1],rebuff[1];
	txbuff[0] = W25X_WriteDisable;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,1);
}


//读取芯片ID
//该指令以/CS 拉低开始，然后通过DI传输指令代码90H和24位的地址（全为000000H）。这之后，WINBOND 的ID （EFH）和芯片ID将在时钟的下降沿以高位在前的方式传出
//注意！！！：这里只有片选一直拉低，让命令连续，才能让FALSH正确工作，得到ID值，所以这里不可以写好几个readwriteNByte，因为这样会让片选中途拉高，导致命令不连续！
//返回值如下:
//0XEF13,表示芯片型号为W25Q80
//0XEF14,表示芯片型号为W25Q16
//0XEF15,表示芯片型号为W25Q32
//0XEF16,表示芯片型号为W25Q64
//0XEF17,表示芯片型号为W25Q128
//0XEF18,表示芯片型号为W25Q256
unsigned short W25QXX_ReadID(void)
{
	unsigned short Temp = 0;
	unsigned char txbuff[6],rebuff[6];
	txbuff[0] = 0x90; //前四个发送字节为命令 0x90000000
	txbuff[1] = 0x00;
	txbuff[2] = 0x00;
	txbuff[3] = 0x00;
	txbuff[4] = 0xFF; //这两个是为了保证接收rebuff的时序而发送的，并没有给FALSH命令，因为在FALSH得到0x90000000之后会在接下来两个时钟周期内发送ID（16位）
	txbuff[5] = 0xFF;
	QSPI_ReadWriteNByte(QSPI2, txbuff,rebuff,6);
	Temp|=(rebuff[4])<<8;  //高16位
	Temp|=rebuff[5];       //低16位
	return Temp;
}


//读取SPI FLASH
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit) 例如 0x11FFFF
//NumByteToRead:要读取的字节数(最大65535)
//命令加地址一共24位

unsigned char pbr[4500];
unsigned char txbuffr[4500];
void W25QXX_Read(unsigned char* pbruffer,unsigned long ReadAddr,unsigned short NumByteToRead)
{
	unsigned short i;
	txbuffr[0] = W25X_ReadData;
	txbuffr[1] = (unsigned char)((ReadAddr)>>16);
	txbuffr[2] = (unsigned char)((ReadAddr)>>8);
	txbuffr[3] = (unsigned char)ReadAddr;
	for(i=4; i<NumByteToRead+4; i++)
	{
		txbuffr[i] = 0xFF;
	}
	QSPI_ReadWriteNByte(QSPI2,txbuffr,pbr,NumByteToRead+4);
	for(i=4; i<NumByteToRead+4; i++)
	{
		pbruffer[i-4] = pbr[i];
	}
}

//SPI在一页(0~65535)内写入少于256个字节的数据
//在指定地址开始写入最大256字节的数据
//pBuffer:要写入FLASH的数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!

void W25QXX_Write_Page(unsigned char* pBuffer,unsigned long WriteAddr,unsigned short NumByteToWrite)
{
	unsigned short i;
	unsigned char txbuff[4+NumByteToWrite],rebuff[4+NumByteToWrite];
	W25QXX_Write_Enable();
	txbuff[0] = W25X_PageProgram;
	txbuff[1] = (unsigned char)((WriteAddr)>>16);
	txbuff[2] = (unsigned char)((WriteAddr)>>8);
	txbuff[3] = (unsigned char)WriteAddr;
	for(i=4; i<NumByteToWrite+4; i++)
	{
		txbuff[i] = pBuffer[i-4];
	}
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,NumByteToWrite+4);
	W25QXX_Wait_Busy();								//等待写入结束
}

//无检验写SPI FLASH
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void W25QXX_Write_NoCheck(unsigned char* pBuffer,unsigned long WriteAddr,unsigned short NumByteToWrite)
{
	unsigned short pageremain;
	pageremain=256-WriteAddr%256; //单页剩余的字节数
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;	//不大于256个字节
	while(1)
	{
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;					//写入结束了
		else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;
	
			NumByteToWrite-=pageremain;							//减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256;				//一次可以写入256个字节
			else pageremain=NumByteToWrite;						//不够256个字节了
		}
	};
}
//写SPI FLASH
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
unsigned char W25QXX_BUFFER[4096];
void W25QXX_Write(unsigned char* pBuffer,unsigned long WriteAddr,unsigned short NumByteToWrite)
{
	unsigned long secpos;
	unsigned short secoff;
	unsigned short secremain;
	unsigned short i;
	unsigned char * W25QXX_BUF;
	W25QXX_BUF=W25QXX_BUFFER;
	secpos=WriteAddr/4096;//扇区地址  将要写入的地址左移3位 0x123456 -> 0x000123
	secoff=WriteAddr%4096;//在扇区内的偏移  获得 0x000456
	secremain=4096-secoff;//扇区剩余空间大小   (0x001000)-(0x000456)

	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//不大于4096个字节
	while(1)
	{
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//读出整个扇区的内容
		for(i=0; i<secremain; i++) //校验数据
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)break;//需要擦除
		}
		if(i<secremain)//需要擦除
		{
			W25QXX_Erase_Sector(secpos);//擦除这个扇区
			for(i=0; i<secremain; i++)	 //复制
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//写入整个扇区
	
		} else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//写已经擦除了的,直接写入扇区剩余区间.
		if(NumByteToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;//扇区地址增1
			secoff=0;//偏移位置为0
	
			pBuffer+=secremain;  //指针偏移
			WriteAddr+=secremain;//写地址偏移
			NumByteToWrite-=secremain;				//字节数递减
			if(NumByteToWrite>4096)secremain=4096;	//下一个扇区还是写不完
			else secremain=NumByteToWrite;			//下一个扇区可以写完了
		}
	};
}


//擦除整个芯片
//等待时间超长...
void W25QXX_Erase_Chip(void)
{
	W25QXX_Write_Enable();							//SET WEL
	W25QXX_Wait_Busy();
	unsigned char txbuff[1],rebuff[1];
	txbuff[0] = W25X_ChipErase;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,1);
	W25QXX_Wait_Busy();								//等待芯片擦除结束
}

//擦除一个扇区
//Dst_Addr:扇区地址 根据实际容量设置  地址范围0x000XX0-0x000XXF(每个XX有16个扇区的地址)
//擦除一个扇区的最少时间:150ms
void W25QXX_Erase_Sector(unsigned long Dst_Addr)
{
	//监视falsh擦除情况,测试用
	//printf("fe:%x\r\n",Dst_Addr);
	Dst_Addr*=4096;//把扇区地址变成FALSH实际地址，*4096即为0x1000，左移三位，变成0xXX0000-0xXXF000的实际地址
	W25QXX_Write_Enable();						//SET WEL
	W25QXX_Wait_Busy();
	
	unsigned char txbuff[4],rebuff[4];
	txbuff[0] = W25X_SectorErase;
	txbuff[1] = (unsigned char)((Dst_Addr)>>16);
	txbuff[2] = (unsigned char)((Dst_Addr)>>8);
	txbuff[3] = (unsigned char)Dst_Addr;
	
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,4);
	W25QXX_Wait_Busy();							//等待擦除完成
}

//等待空闲
void W25QXX_Wait_Busy(void)
{
	while((W25QXX_ReadSR(1)&0x01)==0x01);		// 等待BUSY位清空
}

//进入掉电模式
void W25QXX_PowerDown(void)
{
	unsigned char txbuff[1],rebuff[1];
	txbuff[0] = W25X_PowerDown;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,1);
	delayms(1);
}

//唤醒
void W25QXX_WAKEUP(void)
{
	unsigned char txbuff[1],rebuff[1];
	txbuff[0] = W25X_ReleasePowerDown;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,1);
	delayms(1);
}


