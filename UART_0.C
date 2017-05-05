#include "uart_0.h"
#include "main.h"
#include "i2_rw.h"
#include "ad.h"



volatile unsigned char uartinitflag=2;
volatile unsigned char getover=0;
volatile unsigned char sendover=0;
volatile unsigned char rxtimer=0;
volatile unsigned char sendcount=0;
volatile unsigned char getcount=0;
volatile unsigned char sendmax=0;
volatile unsigned char waittime=0;
volatile unsigned char getdata[]={0,0,0,0,0,0,0,0};
volatile unsigned char senddata[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};




//�������У׼ͨѶ��ַ  100 101 102 103ʮ���Ƶ�ַ  �¶����� �¶����� ʪ������ ʪ������  
//ͨѶ��ַ 00 ������ 01 �¶����� 02 �¶����� 03 ʪ������ 04 ʪ������ 05 
void Uart0_Init(void)        //��ʱ����USART�ĳ�ʼ�� 
{
	unsigned int baud=0;	
	unsigned char i=0;
	const unsigned int baudcode[]={3,6,12,24,48,96,144,192,384};
	const unsigned char  waitcode[]={35,19,10,6,3,2,2,2};
//	baud=baudcode[configItem.baud];
//	waittime=waitcode[configItem.baud];//������������ʱ��
	baud=parItem.Baud/100;
	if(baud<3)
	{
		waittime=35;
	}
	else if(baud>384)
	{
		waittime=2;
	}
	else
	{
		for(i=0;i<8;i++)
		{
			if(baud>=baudcode[i] && baud<baudcode[i+1])
			{
				waittime=waitcode[i];
				break;
			}
		}
	}
	baud=27648/baud-1;
	/*RCSTA�Ĵ���*/	
	SPEN=0;//����ʹ��λ��λ 
	RX9=0;//8λ��������
	CREN=1;//��������

	
	/*TXSTA�Ĵ���*/
	TX9=0;//8λ��������
	TXEN=0;//��ֹ����
	SYNC=0;//ѡ���첽��ʽ
	BRGH=1;//���ٲ�����
	TRMT=0;//������λ�Ĵ�����0  


	
	BRG16=1;
	RCIE=1;//�����ж�ʹ��λ��λ
	TXIE=0;//�����ж�ʹ��λ��λ
	SPBRG=(unsigned char)baud;//OSC/16/BAUD-1  �Ǹ��ٲ�������ΪOSC/64/BAUD-1
    SPBRGH=(unsigned char)(baud>>8);
	FERR=0;
	OERR=0;
	SDIEN_OUT;
	SDI_RECEIVE;//RS485���շ��� 
	SPEN=1;//����ʹ��λ��λ  
}


/********************************************************************************************************
������:void Uart0_Handle
����:��鴮���Ƿ�������  �����������ݴ���
����:��
����:״̬  =1�ɹ�
********************************************************************************************************/
bit Uart0_Handle(void)
{
	unsigned int me_crc=0,it_crc=0;
	unsigned char cop=0;
	unsigned int meteraddr=0,meterlong=0;	
	unsigned char i=0;	
	unsigned int inttmp=0;
	long longtmp=0;
	if(getcount!=8 && getcount!=3)
	{getcount=0;return 0;}
	if(getcount==8)cop=1;
	getcount=0;		
	if(getdata[0]!=parItem.Addr)
	{
		if(getdata[1]!=WRITE || getdata[0]!=0)return 0;	
	}
	if(cop)	
	{
		if((getdata[1]!=READ)&&(getdata[1]!=READONLY)&&(getdata[1]!=WRITE))return 0;
		me_crc=modbusCRC((unsigned char *)getdata,6);	
		it_crc=getdata[7]<<8|getdata[6];
		if(it_crc!=me_crc)return 0;
		meteraddr=getdata[2]<<8|getdata[3];
		meterlong=getdata[4]<<8|getdata[5];
		switch(getdata[1])
		{			
			case READONLY://������Ĵ���
				if(meterlong==0)return 0;
				if((meteraddr+meterlong)>PVMAX)return 0;
				for(i=0;i<meterlong;i++)
				{			
					inttmp=*(&parItem.Temperature+meteraddr+i);
					senddata[3+i*2]=(unsigned char)(inttmp>>8);
					senddata[4+i*2]=(unsigned char)inttmp;				
				}			
				break; 	
			case READ:			//modbusЭ��ָ��   �����ּĴ���			
				if(meterlong==0)return 0;	
				if(meteraddr<ADDR_PAR_COUNT)
				{
					if(meteraddr+meterlong>DATAMAX1)return 0;
					for(i=0;i<meterlong;i++)
					{					
						inttmp=*(&parItem.Addr+meteraddr+i);					
						senddata[3+i*2]=(unsigned char)(inttmp>>8);
						senddata[4+i*2]=(unsigned char)inttmp;		
					}	
				}				
				else if(meteraddr>=3*ADDR_PAR_COUNT && meteraddr<4*ADDR_PAR_COUNT)
				{
					meteraddr%=ADDR_PAR_COUNT;
					if(meteraddr+meterlong>DATAMAX4)return 0;						
					for(i=0;i<meterlong;i++)
					{					
						inttmp=*(&bsItem[0].bsl+meteraddr+i);
						senddata[3+i*2]=(unsigned char)(inttmp>>8);
						senddata[4+i*2]=(unsigned char)inttmp;		
					}	
				}
				else if(meteraddr>=4*ADDR_PAR_COUNT && meteraddr<5*ADDR_PAR_COUNT)
				{
					meteraddr%=ADDR_PAR_COUNT;
					if(meteraddr+meterlong>DATAMAX5)return 0;
					for(i=0;i<meterlong;i++)
					{					
						if(meteraddr+i<4)
							inttmp=*(&bsItem[(meteraddr+i)/2].bslvalue+(meteraddr+i)%2);					
						else 
							inttmp=*(&parItem.jzflag+meteraddr+i-4);
						senddata[3+i*2]=(unsigned char)(inttmp>>8);
						senddata[4+i*2]=(unsigned char)inttmp;		
					}	
				}
				else 
					return 0;							
				break;
			case WRITE:			
				if(meteraddr<ADDR_PAR_COUNT)
				{
					if(meteraddr>=DATAMAX1)return 0;								
					I2_Write((EE_ADDR+meteraddr)*2,(unsigned char *)&meterlong,2);	
					*(&parItem.Addr+meteraddr)=meterlong;				
					if(meteraddr==BAUD_ADDR)
					{
						uartinitflag=1;
					}
				}			
				else if(meteraddr>=3*ADDR_PAR_COUNT && meteraddr<4*ADDR_PAR_COUNT)
				{
					meteraddr%=ADDR_PAR_COUNT;	
					if(meteraddr>=DATAMAX4)return 0;
					I2_Write((EE_BSL+meteraddr)*2,(unsigned char *)&meterlong,2);	
					*(&bsItem[0].bsl+meteraddr)=meterlong;	
				}		
				else if(meteraddr>=4*ADDR_PAR_COUNT && meteraddr<5*ADDR_PAR_COUNT)
				{
					meteraddr%=ADDR_PAR_COUNT;
					if(meteraddr>=DATAMAX5)return 0;						
					I2_Write((EE_BSLVALUE+meteraddr)*2,(unsigned char *)&meterlong,2);
					if(meteraddr<4)
						*(&bsItem[meteraddr/2].bslvalue+meteraddr%2)=meterlong;					
					else 
						*(&parItem.jzflag+meteraddr-4)=meterlong;					
				}
				else
					return 0;						
				break;
		}	
		if(getdata[0]==0)return 0;//�㲥��ַ����������
		senddata[0]=getdata[0];
		senddata[1]=getdata[1];	
		if(getdata[1]==READ || getdata[1]==READONLY)
		{			
			senddata[2]=meterlong*2;
			me_crc=modbusCRC((unsigned char *)senddata,3+senddata[2]);
			senddata[3+senddata[2]]=(unsigned char)me_crc;
			senddata[4+senddata[2]]=(unsigned char)(me_crc>>8);	
			sendmax=5+senddata[2];	
		}
		else	
		{
			for(i=2;i<8;i++)
			{
				senddata[i]=getdata[i];
			}
			sendmax=8;
		}
	}
	else
	{
		if(getdata[1]!=0)return 0;
		me_crc=juscanCRC((unsigned char *)getdata,2);
		it_crc=getdata[2];
		if(me_crc!=it_crc)return 0;
		senddata[0]=getdata[0];
		int pv,bsl,bsh;
		for(i=0;i<2;i++)
		{
			pv=(int)(*(&parItem.Temperature+i));
			bsl=bsItem[i].bsl;
			bsh=bsItem[i].bsh;
			if(pv<=bsl)
			{
				longtmp=0;
			}
			else if(pv>=bsh)
			{
				longtmp=1024;	
			}
			else
			{
				longtmp=pv-bsl;			
				longtmp*=1024;
				longtmp/=bsh-bsl;
			}
			senddata[2+i*2]=(unsigned char)(longtmp>>8);
			senddata[1+i*2]=(unsigned char )longtmp;
		}		
		senddata[5]=juscanCRC((unsigned char *)senddata,5);
		sendmax=6;	
	}
	sendcount=0;	
	CREN=0;	
	RCIE=0;
	SDI_SEND;		
	TXEN=1;	
	TXREG=senddata[sendcount];	
	TXIE=1;	
	return 1;
}
/********************************************************************************************************
������:modbusCRC
����:��ȡMODBUSЭ��16λCRCУ��
����:daBuf�������   len����
����:CRC16
********************************************************************************************************/
unsigned int modbusCRC(unsigned char *daBuf,unsigned char len)
{
   unsigned char BitFg=0;
   unsigned char i=0,j=0;
   unsigned int CRCBuf = 0xffff;
   for(j=0;j<len;j++)
   {
      CRCBuf ^= daBuf[j];
      for(i=0;i<8;i++)
      {
         BitFg = CRCBuf&1;
         CRCBuf >>= 1;        
         if(BitFg==1)CRCBuf ^= 0xa001;
      }
    }   
	return CRCBuf;
}


/********************************************************************************************************
������:juscanCRC
����:��ȡjuscanЭ��8λCRCУ��
����:daBuf�������   len����
����:CRC8
********************************************************************************************************/
unsigned char juscanCRC(unsigned char *daBuf,unsigned char len)
{
   	unsigned char CRCBuf=0;
	unsigned char i=0;
	for(i=0;i<len;i++)
	{
		CRCBuf^=daBuf[i];
	}
	return CRCBuf;
}



