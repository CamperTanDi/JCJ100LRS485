#ifndef _BS_
#define _BS_

extern void BS_Init(void);
extern void BS_Handle(void);
typedef struct{
	int pv[CH_MAX];		
	unsigned char bscount;
}_bsInfo;
#define IO_BS1	RA0
#define IO_BS2	RA1
#define IO_BS1_EN	TRISA0
#define IO_BS2_EN	TRISA1

extern bank2 volatile _bsInfo bsInfo;
extern bank2 volatile unsigned char bsvalueH[2];
extern bank2 volatile unsigned char bsvalueL[2];
extern bank2 volatile unsigned char bsindex;
extern bank2 volatile unsigned char bsCCP[2];


#endif 