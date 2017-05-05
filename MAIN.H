/********************************************************************

	
	
*********************************************************************/

#ifndef __c_h__
#define __c_h__

#include <pic.h>




//----------------------------

	
#define KEY_RESET 		RA3
#define KEY_RESETEN 	TRISA3
#define KEY_RESETEN_IN  KEY_RESETEN=1
/*RA2,logic_output is a test for controling BL1551 chip*/
/*#define LOGIC_OUTPUTEN  TRISA2 = 0
#define LOGIC_OUTPUT    RA2*/
/*LOGICOUTPUT is end here*/
//----------------------------

#define EE_TEMPERATURE  0x00
#define EE_ADDR			0x02
#define EE_BSL			0x04
#define EE_BSLVALUE		0x08
#define EE_JZFLAG		0x0c



typedef struct {
	unsigned int Temperature;
	unsigned int Humidity;
	unsigned int Addr;
	unsigned int Baud;
	int jzflag;
	int outflag;
}_parItem;
extern volatile _parItem parItem;
extern volatile unsigned int resetcount;
extern volatile unsigned char reset;

typedef struct {
	int bsl;
	int bsh;
	int bslvalue;
	int bshvalue;
}_bsItem;
extern volatile _bsItem bsItem[2];
#endif

