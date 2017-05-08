
#include "main.h"
#include "i2_rw.h"
#include "uart_0.h"
#include "ad.h"
#include "bs.h"

__CONFIG(0x3002);

void Sys_Init(void);
void Sys_GetPar(void);
void Reset_Handle(void);
volatile _parItem parItem;	
volatile _bsItem bsItem[2];
volatile unsigned int Inittimer=0;
volatile unsigned int resetcount=0;
volatile unsigned char reset=0;

void main()
{		
	Sys_Init();
	Sys_GetPar();
	AD_Init();
	BS_Init();
	while(Inittimer<2000)AD_Handle();
	WDTCON=0x13;
	while(1)
	{
		asm("clrwdt");
		if(reset){
			Reset_Handle();
			reset=0;
			uartinitflag=2;
		}
		if(uartinitflag==2){
			uartinitflag=0;
			Uart0_Init();
		}
		if(sendover){		
			sendover=0;			
		}		
		if(getover){		
			Uart0_Handle();		
			getover=0;
		}
		AD_Handle();
		parItem.Temperature = adInfo.pv[0];
		parItem.Humidity = adInfo.pv[1];
		BS_Handle();
	}
}

void interrupt INT()
{
	if(T0IF && T0IE)
	{
		T0IF = 0;	
		TMR0 = 0xf6;		
		bsInfo.bscount++;
	/*	if (bsCCP[0] >= bsCCP[1]){
			LOGIC_OUTPUT = 1;//TO大于HO时，将板耗电走TO端流出
		}
		else{
			LOGIC_OUTPUT = 0;//反之走HO端流出
		}
*/
		if(bsInfo.bscount>=bsCCP[0])
			IO_BS1=0;
		else
			IO_BS1=1;		
		if(bsInfo.bscount>=bsCCP[1])
			IO_BS2=0;
		else
			IO_BS2=1;		
	}
	if((RCIE&&RCIF)||(TXIE&&TXIF))
	{						
		if(RCIF)
		{				
			if(getcount>=GETMAX)getcount=0;
			getdata[getcount]=RCREG;		
			getcount++;				
			rxtimer=0;			
		}
		if(TXIF)
		{				
			if(sendcount>=sendmax)
			{
				sendcount=0;						
				TXIE=0;//½ûÖ¹·¢ËÍ	
				TXEN=0;	
				SDI_RECEIVE;	
				CREN=1;	
				RCIE=1;		
				if(uartinitflag)uartinitflag=2;
				sendover=1;	
			}
			else
			{			
				sendcount++;
				TXREG=senddata[sendcount];			
			}
		}	
	}
	if(TMR1IF)//
	{			
		TMR1IE=0;
		TMR1IF=0;	
		bsindex++;
		if(bsindex > 0x07){
			bsindex=0;
		}
		if(bsindex < bsvalueL[0]){
			bsCCP[0]=bsvalueH[0];
		}
		else if(bsvalueH[0]){
			bsCCP[0]=bsvalueH[0]-1;
		}
		if(bsindex < bsvalueL[1]){
			bsCCP[1]=bsvalueH[1];
		}
		else if(bsvalueH[1]){
			bsCCP[1]=bsvalueH[1]-1;
		}
						
		if(getcount>0)	
		{	
			rxtimer++;	
			if(rxtimer>waittime)
			{
				getover=1;					
			}	
		}
		else 
		{	
			if(rxtimer>0)rxtimer=0;	
		}		
		if(KEY_RESET)
		{			
			if(resetcount>2000)reset=1;
			resetcount=0;
		}
		else
		{
			if(resetcount<60000)resetcount++;		
		}
		adlostcount++;
		Inittimer++;
		if( adhotcount>0)adhotcount++;
		TMR1H=0xfd;
		TMR1L=0x7e;
		TMR1IE=1;	
	}	
}

void Sys_GetPar(void)
{		
	unsigned char i;
	const unsigned int parreset[]={0,1000,1,9600,-400,800,0,1000,350,1650,350,1650,0,1};
	unsigned int meterinit;
	I2_Read(0x80,(unsigned char *)&meterinit,2);
	if(meterinit!=3020)
	{
		I2_Write(EE_TEMPERATURE*2,(unsigned char *)parreset,sizeof(parreset));
		meterinit=3020;
		I2_Write(0x80,(unsigned char *)&meterinit,2);
	}
	I2_Read(EE_TEMPERATURE*2,(unsigned char *)&parItem.Temperature,8);
	I2_Read(EE_JZFLAG*2,(unsigned char *)&parItem.jzflag,4);
	for(i=0;i<2;i++)
	{
		I2_Read(EE_BSL*2+i*4,(unsigned char *)&bsItem[i].bsl,4);
		I2_Read(EE_BSLVALUE*2+i*4,(unsigned char *)&bsItem[i].bslvalue,4);
	}
}

void Sys_Init()
{	
	INTCON = 0x00;
	OPTION = 0x80;		//½ûÖ¹ÉÏÀ­	
	
	CMCON0 = 0b00000111;		//±È½ÏÆ÷¹Ø±Õ			
	ANSEL = 0x00;		//Êý×Ö¿Ú	
	
	TMR1H=0xfd;// 
	TMR1L=0x7e;
	TMR1IF=0;
	TMR1IE=1;	
	T1CON=0x21;	

	KEY_RESETEN_IN;
	
		
	PEIE=1;
	GIE=1;
}
//

void Reset_Handle(void)
{		
	parItem.Addr=1;
	parItem.Baud=9600;
	I2_Write(EE_ADDR*2,(unsigned char *)&parItem.Addr,4);	
} 
