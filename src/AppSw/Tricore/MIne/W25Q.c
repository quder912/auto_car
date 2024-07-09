#include "LQ_QSPI.h"
#include "W25Q.h"
#include "stdio.h"
/*
 *
 * @note ���Ѹ�ΪQSPI
 *
 *
 *
 *
 *
 */


unsigned short W25QXX_TYPE = 0;	//Ĭ����W25Q128

//4KbytesΪһ��Sector��������   4kB
//16������Ϊ1��Block    64kB
//W25Q128
//����Ϊ16M�ֽ�,����256��Block,4096��Sector

//��ʼ��SPI FLASH��IO��  HALA�õ���ӲSPI4
//����3�ֽڵ�ַģʽ��4�ֽڵ�ַģʽ��
//w25q128��128Mb(16M) ���ֽڱ�ַ ��ַ��Χ�� 0x000000-0xFFFFFF ��ַλ��ռ��3���ֽڣ����Խ����ֽڵ�ַģʽ
//w25q256: 256Mb(32M) ���ֽڱ��� ��ַ��Χ�� 0x00000000-01 FF FF FF ��ַλ��ռ��4���ֽڣ����Խ����ֽڵ�ַģʽ(��ʵ�ϵ�ַ��Χ��0x01000000-0xFFFFFFFF��FALSH�������ֽڵ�ַģʽ)
//���ݷ���ʧ��״̬�Ĵ���λADP��S17�������ã�W25Q256FV���Թ�����3�ֽڵ�ַģʽ��4�ֽڵ�ַģʽ�����ADP = 0������������3�ֽڵ�ַģʽ������
//������������3�ֽڵ�ַģʽ���������ADP = 1�����豸����4�ֽڵ�ַģʽ������ ADP�ĳ���Ĭ��ֵΪ0��

void W25QXX_Init(void)
{
    char txtn[20];
    QSPI_InitConfig(QSPI2_CLK_P15_8,QSPI2_MISO_P15_7,QSPI2_MOSI_P15_6,QSPI2_CS_P20_6,5000000,1);//����SPIģʽ�Ͳ����� SPIģʽ��CLK����ʱ��Ϊ�ߵ�ƽ �ڵڶ���CLK�����أ��½��أ���������
	W25QXX_TYPE = W25QXX_ReadID();	//��ȡFLASH ID.
	sprintf(txtn,"N:%05d",W25QXX_TYPE);
	OLED_P6x8Str(2,0,txtn);
	W25QXX_Wait_Busy();
//
}


//��ȡW25QXX��״̬�Ĵ�����W25QXXһ����3��״̬�Ĵ���
//״̬�Ĵ���1��
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
//״̬�Ĵ���2��
//BIT7  6   5   4   3   2   1   0
//SUS   CMP LB3 LB2 LB1 (R) QE  SRP1
//״̬�Ĵ���3��
//BIT7      6    5    4   3   2   1   0
//HOLD/RST  DRV1 DRV0 (R) (R) WPS ADP ADS
//regno:״̬�Ĵ����ţ���:1~3
//����ֵ:״̬�Ĵ���ֵ
unsigned char W25QXX_ReadSR(unsigned char regno)
{
	unsigned char command=0;
	switch(regno)
	{
	case 1:
		command=W25X_ReadStatusReg1;    //��״̬�Ĵ���1ָ��
		break;
	case 2:
		command=W25X_ReadStatusReg2;    //��״̬�Ĵ���2ָ��
		break;
	case 3:
		command=W25X_ReadStatusReg3;    //��״̬�Ĵ���3ָ��
		break;
	default:
		command=W25X_ReadStatusReg1;
		break;
	}
	unsigned char txbuff[2],rebuff[2];
	txbuff[0] = command;
	txbuff[1] = 0xff;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,2);  //���Ͷ�ȡ״̬�Ĵ�������
	return rebuff[1];
}

//дW25QXX״̬�Ĵ���
void W25QXX_Write_SR(unsigned char regno,unsigned char sr)
{
	unsigned char command=0;
	switch(regno)
	{
	case 1:
		command=W25X_WriteStatusReg1;    //д״̬�Ĵ���1ָ��
		break;
	case 2:
		command=W25X_WriteStatusReg2;    //д״̬�Ĵ���2ָ��
		break;
	case 3:
		command=W25X_WriteStatusReg3;    //д״̬�Ĵ���3ָ��
		break;
	default:
		command=W25X_WriteStatusReg1;
		break;
	}
	unsigned char txbuff[2],rebuff[2];
	txbuff[0] = command;
	txbuff[1] = sr;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,2);  //���Ͷ�ȡ״̬�Ĵ�������
}

//W25QXXдʹ��
//��WEL��λ
void W25QXX_Write_Enable(void)
{
	unsigned char txbuff[1],rebuff[1];
	txbuff[0] = W25X_WriteEnable;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,1);
}

//W25QXXд��ֹ
//��WEL����
void W25QXX_Write_Disable(void)
{
	unsigned char txbuff[1],rebuff[1];
	txbuff[0] = W25X_WriteDisable;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,1);
}


//��ȡоƬID
//��ָ����/CS ���Ϳ�ʼ��Ȼ��ͨ��DI����ָ�����90H��24λ�ĵ�ַ��ȫΪ000000H������֮��WINBOND ��ID ��EFH����оƬID����ʱ�ӵ��½����Ը�λ��ǰ�ķ�ʽ����
//ע�⣡����������ֻ��Ƭѡһֱ���ͣ�������������������FALSH��ȷ�������õ�IDֵ���������ﲻ����д�ü���readwriteNByte����Ϊ��������Ƭѡ��;���ߣ��������������
//����ֵ����:
//0XEF13,��ʾоƬ�ͺ�ΪW25Q80
//0XEF14,��ʾоƬ�ͺ�ΪW25Q16
//0XEF15,��ʾоƬ�ͺ�ΪW25Q32
//0XEF16,��ʾоƬ�ͺ�ΪW25Q64
//0XEF17,��ʾоƬ�ͺ�ΪW25Q128
//0XEF18,��ʾоƬ�ͺ�ΪW25Q256
unsigned short W25QXX_ReadID(void)
{
	unsigned short Temp = 0;
	unsigned char txbuff[6],rebuff[6];
	txbuff[0] = 0x90; //ǰ�ĸ������ֽ�Ϊ���� 0x90000000
	txbuff[1] = 0x00;
	txbuff[2] = 0x00;
	txbuff[3] = 0x00;
	txbuff[4] = 0xFF; //��������Ϊ�˱�֤����rebuff��ʱ������͵ģ���û�и�FALSH�����Ϊ��FALSH�õ�0x90000000֮����ڽ���������ʱ�������ڷ���ID��16λ��
	txbuff[5] = 0xFF;
	QSPI_ReadWriteNByte(QSPI2, txbuff,rebuff,6);
	Temp|=(rebuff[4])<<8;  //��16λ
	Temp|=rebuff[5];       //��16λ
	return Temp;
}


//��ȡSPI FLASH
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit) ���� 0x11FFFF
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
//����ӵ�ַһ��24λ

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

//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:Ҫд��FLASH�����ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!

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
	W25QXX_Wait_Busy();								//�ȴ�д�����
}

//�޼���дSPI FLASH
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ����
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void W25QXX_Write_NoCheck(unsigned char* pBuffer,unsigned long WriteAddr,unsigned short NumByteToWrite)
{
	unsigned short pageremain;
	pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;	//������256���ֽ�
	while(1)
	{
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;					//д�������
		else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;
	
			NumByteToWrite-=pageremain;							//��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256)pageremain=256;				//һ�ο���д��256���ֽ�
			else pageremain=NumByteToWrite;						//����256���ֽ���
		}
	};
}
//дSPI FLASH
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
unsigned char W25QXX_BUFFER[4096];
void W25QXX_Write(unsigned char* pBuffer,unsigned long WriteAddr,unsigned short NumByteToWrite)
{
	unsigned long secpos;
	unsigned short secoff;
	unsigned short secremain;
	unsigned short i;
	unsigned char * W25QXX_BUF;
	W25QXX_BUF=W25QXX_BUFFER;
	secpos=WriteAddr/4096;//������ַ  ��Ҫд��ĵ�ַ����3λ 0x123456 -> 0x000123
	secoff=WriteAddr%4096;//�������ڵ�ƫ��  ��� 0x000456
	secremain=4096-secoff;//����ʣ��ռ��С   (0x001000)-(0x000456)

	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//������4096���ֽ�
	while(1)
	{
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//������������������
		for(i=0; i<secremain; i++) //У������
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)break;//��Ҫ����
		}
		if(i<secremain)//��Ҫ����
		{
			W25QXX_Erase_Sector(secpos);//�����������
			for(i=0; i<secremain; i++)	 //����
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//д����������
	
		} else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������.
		if(NumByteToWrite==secremain)break;//д�������
		else//д��δ����
		{
			secpos++;//������ַ��1
			secoff=0;//ƫ��λ��Ϊ0
	
			pBuffer+=secremain;  //ָ��ƫ��
			WriteAddr+=secremain;//д��ַƫ��
			NumByteToWrite-=secremain;				//�ֽ����ݼ�
			if(NumByteToWrite>4096)secremain=4096;	//��һ����������д����
			else secremain=NumByteToWrite;			//��һ����������д����
		}
	};
}


//��������оƬ
//�ȴ�ʱ�䳬��...
void W25QXX_Erase_Chip(void)
{
	W25QXX_Write_Enable();							//SET WEL
	W25QXX_Wait_Busy();
	unsigned char txbuff[1],rebuff[1];
	txbuff[0] = W25X_ChipErase;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,1);
	W25QXX_Wait_Busy();								//�ȴ�оƬ��������
}

//����һ������
//Dst_Addr:������ַ ����ʵ����������  ��ַ��Χ0x000XX0-0x000XXF(ÿ��XX��16�������ĵ�ַ)
//����һ������������ʱ��:150ms
void W25QXX_Erase_Sector(unsigned long Dst_Addr)
{
	//����falsh�������,������
	//printf("fe:%x\r\n",Dst_Addr);
	Dst_Addr*=4096;//��������ַ���FALSHʵ�ʵ�ַ��*4096��Ϊ0x1000��������λ�����0xXX0000-0xXXF000��ʵ�ʵ�ַ
	W25QXX_Write_Enable();						//SET WEL
	W25QXX_Wait_Busy();
	
	unsigned char txbuff[4],rebuff[4];
	txbuff[0] = W25X_SectorErase;
	txbuff[1] = (unsigned char)((Dst_Addr)>>16);
	txbuff[2] = (unsigned char)((Dst_Addr)>>8);
	txbuff[3] = (unsigned char)Dst_Addr;
	
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,4);
	W25QXX_Wait_Busy();							//�ȴ��������
}

//�ȴ�����
void W25QXX_Wait_Busy(void)
{
	while((W25QXX_ReadSR(1)&0x01)==0x01);		// �ȴ�BUSYλ���
}

//�������ģʽ
void W25QXX_PowerDown(void)
{
	unsigned char txbuff[1],rebuff[1];
	txbuff[0] = W25X_PowerDown;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,1);
	delayms(1);
}

//����
void W25QXX_WAKEUP(void)
{
	unsigned char txbuff[1],rebuff[1];
	txbuff[0] = W25X_ReleasePowerDown;
	QSPI_ReadWriteNByte(QSPI2,txbuff,rebuff,1);
	delayms(1);
}


