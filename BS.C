#include "ad.h"
#include "bs.h"
#include "main.h"

bank2 volatile _bsInfo bsInfo;
bank2 volatile unsigned char bsvalueH[2]={0,0};
bank2 volatile unsigned char bsvalueL[2]={0,0};
bank2 volatile unsigned char bsindex=0;
bank2 volatile unsigned char bsCCP[2];


void BS_Init(void);
void BS_Handle(void);
void BS_Init(void)
{
	IO_BS1_EN = 0;
	IO_BS2_EN = 0;
	T0CS=0;
	T0SE=0;
	PSA=0;
	PS2=0;PS1=0;PS0=1;//4·ÖÆµ
	TMR0 = 0xf6;			
	T0IF = 0;			
	T0IE = 1;	
}


void BS_Handle(void)
{
	unsigned char i=0;
	long bstmp=0;
	long inttmp=0;	
	if(parItem.jzflag==0)
	{
		for(i=0;i<CH_MAX;i++)
		{		
			if(parItem.outflag == 1)
			{
				inttmp=bsItem[i].bslvalue;
			}
			else	
			{
				inttmp=bsItem[i].bslvalue*5-bsItem[i].bshvalue;		
				inttmp/=4;	
			}		
			if(adInfo.pv[i]<=bsItem[i].bsl)
			{
				bstmp=inttmp;					
			}
			else if(adInfo.pv[i]>=bsItem[i].bsh)
			{
				bstmp=bsItem[i].bshvalue;
			}
			else
			{
				bstmp=adInfo.pv[i]-bsItem[i].bsl;				
				bstmp*=(bsItem[i].bshvalue-inttmp);
				bstmp/=(bsItem[i].bsh-bsItem[i].bsl);
				bstmp+=inttmp;
			}
			if(bstmp-1<0)bstmp=1;					
			bsInfo.pv[i]=(int)bstmp;			
		}	
	}
	else
	{
		switch(parItem.jzflag)
		{
			case 1:
				bsInfo.pv[0]=bsItem[0].bslvalue;
				break;
			case 2:
				bsInfo.pv[0]=bsItem[0].bshvalue;
				break;
			case 3:
				bsInfo.pv[1]=bsItem[1].bslvalue;
				break;
			case 4:
				bsInfo.pv[1]=bsItem[1].bshvalue;
				break;
		}
	}
	TMR1IE = 0;
	for(i=0;i<CH_MAX;i++)
	{
		bsvalueH[i]=(unsigned char)(bsInfo.pv[i]>>3);
		bsvalueL[i]=(unsigned char)(bsInfo.pv[i]&0x07);
	}	
	TMR1IE = 1;
}

