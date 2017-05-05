#ifndef _UART_0
#define _UART_0

#define GETMAX 			8
#define READ 			3
#define READONLY 		4
#define WRITE 			6
#define DATAMAX1 		2
#define DATAMAX2		4
#define DATAMAX3		3
#define DATAMAX4		2
#define DATAMAX5		6
#define PVMAX 			2
#define ADDR_ADDR 		0
#define BAUD_ADDR 		1

#define SDI 			RA2
#define SDIEN			TRISA2
#define SDI_RECEIVE		SDI=0
#define SDI_SEND		SDI=1
#define SDIEN_OUT 		SDIEN=0
#define SDIEN_IN		SDIEN=1
#define ADDR_PAR_COUNT	0x64

extern unsigned int modbusCRC(unsigned char *daBuf,unsigned char len);
extern unsigned char juscanCRC(unsigned char *daBuf,unsigned char len);
extern void Uart0_Init(void);
extern bit Uart0_Handle(void);

extern volatile unsigned char uartinitflag;
extern volatile unsigned char getover;
extern volatile unsigned char sendover;
extern volatile unsigned char rxtimer;
extern volatile unsigned char sendcount;
extern volatile unsigned char getcount;
extern volatile unsigned char sendmax;
extern volatile unsigned char waittime;
extern volatile unsigned char getdata[];
extern volatile unsigned char senddata[];

#endif

