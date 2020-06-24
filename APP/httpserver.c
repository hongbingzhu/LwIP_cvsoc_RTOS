/* ------------------------------------------------------------------------------------------------ */
/* FILE :		httpserver_CY5_A9_GCC_DS5.c															*/
/*																									*/
/* CONTENTS :																						*/
/*				Demo for the Altera Cyclone V SoC FPGA development board							*/
/*				Standalone Webserver demo															*/
/*				Info & configuration through the serial port										*/
/*				Serial port settings:																*/
/*							Baud Rate: 115200														*/
/*							Data bits:      8														*/
/*							Stop bits:      1														*/
/*							Parity:      none														*/
/*							Flow Ctrl:   none														*/
/*							Emulation:   none														*/
/*																									*/
/*																									*/
/* Copyright (c) 2013, Code-Time Technologies Inc. All rights reserved.								*/
/*																									*/
/* Code-Time Technologies retains all right, title, and interest in and to this work				*/
/*																									*/
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS							*/
/* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF										*/
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL							*/
/* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR								*/
/* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,							*/
/* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR							*/
/* OTHER DEALINGS IN THE SOFTWARE.																	*/
/*																									*/
/*																									*/
/*	$Revision: 1.8 $																				*/
/*	$Date: 2013/10/03 15:55:16 $																	*/
/*																									*/
/* ------------------------------------------------------------------------------------------------ */

#include <stdio.h>
#include "alt_16550_uart.h"
#include "alt_ethernet.h"
#include "alt_eth_dma.h"
#include "alt_interrupt.h"	// IRQ number
#include "NetAddr.h"
#include "httpd.h"
#include "os.h"
#include "bsp_int.h"

/* Switch between OS mode and Standalone:
 * 1. Change NO_SYS in "lwipopts.h"
 * 2. Change Source file in Makefile (under ports/CycloneV)
 */
/* ------------------------------------------------ */

#define BAUDRATE	ALT_16550_BAUDRATE_115200		/* Serial port baud rate						*/

#define SYS_FREQ	800000000						/* Processor clock frequency: 800 MHz			*/

#define COREgetPeriph()			((unsigned int *)0xFFFEC000)
#define ISRinstall(Nmb, Fct)	(G_OSisrTbl[(Nmb)]=(Fct))

/* ------------------------------------------------ */

extern   u32_t G_IPnetDefGW;
extern   u32_t G_IPnetDefIP;
extern   u32_t G_IPnetDefNM;
extern   int   G_IPnetStatic;

void (*G_OSisrTbl[1024])(void);

ALT_16550_HANDLE_t G_UARThndl;

/* ------------------------------------------------ */

extern void EnbInt(void);
int  GetKey(void);								/* Non-blocking keyboard input					*/
void GICenable(int IntNmb, int Prio, int Edge);
void GICinit(void);
int  __putchar(int);
extern void Time_Update(void);
void TIMERinit(unsigned int Reload);
void UARTinit(int BaudRate);

/* ------------------------------------------------------------------------------------------------ */
void App_TimeTickHook(void)
{
	G_IPnetTime = OSTimeGet();
}

/* ------------------------------------------------------------------------------------------------ */

int lwip_app_main(void)
{
int  ii;											/* General purpose								*/
int  LastWait;										/* Last elapse time (to update display)			*/
int  WaitTime;										/* Elapsed time waiting for key pressed			*/

/* ------------------------------------------------ */
/* UART set-up										*/

	setvbuf(stdin,  NULL, _IONBF, 0);				/* By default, NewLib library flush the I/O		*/
	setvbuf(stdout, NULL, _IONBF, 0);				/* buffer when full or when new-line			*/

	UARTinit(BAUDRATE);

/* ------------------------------------------------ */
/* LED & switches setup								*/
#if 0
	GPIO_DIR(LED_0, 1);								/* Set all 4 LED pins in output					*/
	GPIO_DIR(LED_1, 1);
	GPIO_DIR(LED_2, 1);
	GPIO_DIR(LED_3, 1);

	GPIO_SET(LED_0, 1);								/* Make sure they are all OFF					*/
	GPIO_SET(LED_1, 1);
	GPIO_SET(LED_2, 1);
	GPIO_SET(LED_3, 1);

	GPIO_DIR(SW_0, 0);
	GPIO_DIR(SW_1, 0);
	GPIO_DIR(SW_2, 0);
	GPIO_DIR(SW_3, 0);
#endif
/* ------------------------------------------------ */
/* SysTick Set-up									*/
#if 0	// Use OS's system tick and OS's interrupt management.
	GICinit();
	EnbInt();

	ISRinstall(29, Time_Update);
	TIMERinit(((((SYS_FREQ/4)/100000)*(SYSTICK_MS*1000))/(10)));
	GICenable(29, 128, 1);							/* Private timer interrupt ID is 29 prio=mid	*/
													/* The Cyclone V is set with a /4 prescaler		*/
#endif
/* ------------------------------------------------ */
/* Default static Addresses & Mask					*/

	G_IPnetStatic = 0;								/* By default, use DHCP							*/
	G_IPnetDefIP  = IP4_INT32(IP_ADDR0,      IP_ADDR1,      IP_ADDR2,      IP_ADDR3);
	G_IPnetDefNM  = IP4_INT32(NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
	G_IPnetDefGW  = IP4_INT32(GATEWAY_ADDR0, GATEWAY_ADDR1, GATEWAY_ADDR2, GATEWAY_ADDR3);

/* ------------------------------------------------ */
/* UART prompt and key pressed check				*/

	printf("\n  Standalone Demo  \n");
	printf("  lwIP  Webserver   \n");
	printf("\nDefaults settings:\n");
	printf("IP address : "); PrintIPv4Addr(G_IPnetDefIP); printf("\n");
	printf("Net Mask   : "); PrintIPv4Addr(G_IPnetDefNM); printf("\n");
	printf("Gateway    : "); PrintIPv4Addr(G_IPnetDefGW); printf("\n");
	printf("\n");
	printf("DHCP will start in 5s.\n");
	printf("Type any key use static IP address\n");

	WaitTime = G_IPnetTime;
	LastWait = -1;
	do {
		ii = G_IPnetTime-WaitTime;
		if (LastWait != (5999-ii)/1000) {
			LastWait = (5999-ii)/1000;
			printf("\r\r\r\r%d s", LastWait);
		}
		G_IPnetStatic = GetKey();					/* Check if user pressed a key					*/
	} while((ii < 5000)								/* Check for 5 seconds							*/
	  &&    (G_IPnetStatic == 0));					/* Or until a key is pressed					*/

	printf("\r\r\r   \r\r\r\n");					/* Erase the remaining time from UART screen	*/

/* ------------------------------------------------ */
/* lwIP initialization								*/

	alt_eth_dma_mac_config();
	LwIP_Init();
	httpd_init();
    httpd_ssi_init();
	httpd_cgi_init();

	puts("The webserver is ready");

/* ------------------------------------------------ */
/* Processing loop									*/
#if NO_SYS
	while (1) {										/* Processing is an infinite loop				*/
		if (ETH_CheckFrameReceived()) {				/* Did we received a packet?					*/
			LwIP_Packet();							/* Process the packet							*/
		}
		LwIP_Periodic(G_IPnetTime);					/* Periodic timer for lwIP						*/
	}
#else
	extern void Emac0_IRQHandler(CPU_INT32U cpu_id);	/* in ports/.../RTOS/ethernetif.c */
	BSP_IntVectSet (ALT_INT_INTERRUPT_EMAC0_IRQ,
					2u,
					0x01,	// handle by Core-0
					Emac0_IRQHandler);

	BSP_IntSrcEn(ALT_INT_INTERRUPT_EMAC0_IRQ);

	return 0;
#endif
}

/* ------------------------------------------------------------------------------------------------ */
/* Private timer set-up																				*/
/* ------------------------------------------------------------------------------------------------ */

void TIMERinit(unsigned int Reload)
{

volatile unsigned int *TimReg;

	TimReg    = COREgetPeriph();					/* Private timer is at base of Peripherals		*/
	TimReg   += 0x600/sizeof(unsigned int);
	TimReg[2] = 0;									/* Disable timer & interrupts					*/
	TimReg[0] = Reload;								/* Private timer load value						*/
	TimReg[1] = Reload;								/* Start the count at reload value				*/
													/* Private timer control register				*/
	TimReg[2] = (0<<8)								/* Prescaler = /1								*/
	          | (1<<2)								/* Interrupt enable								*/
	          | (1<<1)								/* Auto-reload									*/
	          | (1<<0);								/* Timer enable									*/

	return;
}

/* ------------------------------------------------------------------------------------------------	*/
/* void GICinit()void)																				*/
/*																									*/
/* Init the GIC																						*/
/* ------------------------------------------------------------------------------------------------	*/
#define GIC_ICDDCR	(0x000/4)
#define GIC_ICDICTR	(0x004/4)
#define GIC_ICDISR	(0x080/4)
#define GIC_ICDISER	(0x100/4)
#define GIC_ICDICER	(0x180/4)
#define GIC_ICDIPR	(0x400/4)
#define GIC_ICDIPTR	(0x800/4)
#define GIC_ICDICFR	(0xc00/4)
#define GIC_ICDSGIR	(0xf00/4)
#define GIC_ICCICR	(0x000/4)
#define GIC_ICCPMR	(0x004/4)
#define GIC_ICCBPR	(0x008/4)

void GICinit(void)
{
int ii;
volatile unsigned int  *I32reg;				/* Byte addressable register					*/
volatile unsigned int  *CPUreg;
volatile unsigned int  *DISTreg;

	CPUreg    = COREgetPeriph();
	DISTreg   = CPUreg + (0x1000/4);
	CPUreg   += 0x100/4;

	CPUreg[GIC_ICCICR]  = 0;					/* Disable interrupts when setting up			*/

	I32reg = (volatile unsigned int *) &DISTreg[GIC_ICDICER];
	for (ii=0 ; ii<32 ; ii++) {					/* Clear all interrupt enable bits				*/
		I32reg[ii] = 0xFFFFFFFF;
	}

	CPUreg[GIC_ICCPMR]    = 0xFF;				/* Set the priority [ICCPMR register]			*/
	CPUreg[GIC_ICCBPR]    = 0xFF;				/* Set binary point register					*/
	CPUreg[GIC_ICCICR]    = 3;					/* Enable the signalling of the interrupts		*/
	DISTreg[GIC_ICDDCR]   = 1;					/* Enable the distributor (banked register)		*/
												/* Secure mode on (bit0), non-secure off (bit1)	*/
	return;
}

/* ------------------------------------------------------------------------------------------------	*/
/* void GICenable(int IntNmb, int Prio, int Edge)													*/
/*																									*/
/* Enable an interrupt in the GIC																	*/
/* ------------------------------------------------------------------------------------------------	*/

void GICenable(int IntNmb, int Prio, int Edge)
{
int CoreID;
volatile unsigned char *BYTEreg;				/* Byte addressable register					*/
volatile unsigned int  *CPUreg;
volatile unsigned int  *DISTreg;

	if (Edge != 0) {
		Edge = 2;
	}

	CPUreg  = COREgetPeriph();
	DISTreg = CPUreg + (0x1000/4);
	CPUreg += 0x100/4;

	CPUreg[GIC_ICCICR]  = 0;					/* Disable the signalling of the interrupts		*/

	CoreID = 0;

	if ((DISTreg[GIC_ICDICTR]&0x400) != 0) {	/* Set this interrupt as secure					*/
		DISTreg[GIC_ICDISR + (IntNmb/32)] &= ~(1<<(IntNmb&31));
	}

	DISTreg[GIC_ICDICFR + (IntNmb/16)] = Edge;	/* Set to level / edge sensitive				*/
												/* Set the priority								*/
	BYTEreg         = (volatile unsigned char *)&DISTreg[GIC_ICDIPR];
	BYTEreg[IntNmb] = Prio & 0xFF;
												/* Set the target CPU #							*/
	BYTEreg         = (volatile unsigned char *)&DISTreg[GIC_ICDIPTR];
	BYTEreg[IntNmb] = 1<<CoreID;
												/* Enable the interrupt							*/
	DISTreg[GIC_ICDISER + (IntNmb/32)] |= 1<<(IntNmb&31);

	CPUreg[GIC_ICCICR] = 3;						/* Enable the signalling of the interrupts		*/

	return;
 }

/* ------------------------------------------------------------------------------------------------ */
/*																									*/
/* UART functions needed by the Peripheral library to access the UART								*/
/*																									*/
/* ------------------------------------------------------------------------------------------------ */

void UARTinit(int BaudRate)
{
	G_UARThndl.location   = 0;
	G_UARThndl.clock_freq = 0;

	alt_16550_int_disable_all(&G_UARThndl);
	alt_16550_init(ALT_16550_DEVICE_SOCFPGA_UART0, 0, 0, &G_UARThndl);
	alt_16550_line_config_set(&G_UARThndl, ALT_16550_DATABITS_8,
	                                       ALT_16550_PARITY_DISABLE,
	                                       ALT_16550_STOPBITS_1);
	alt_16550_baudrate_set(&G_UARThndl, BaudRate);
	alt_16550_fifo_enable(&G_UARThndl);
	alt_16550_enable(&G_UARThndl);

	return;
}

/* ------------------------------------------------------------------------------------------------ */

#if defined(__GNUC__) && !defined(_write_r)
_ssize_t _write_r (struct _reent *rreenn, int fd, const void *vbuf, size_t len)
{
const char *buf;
int   size;

	size = len;
	buf  = vbuf;
	while (size > 0) {
		if (*buf == '\n') {
			__putchar((int)'\r');
		}
		__putchar((int)*buf);
		buf++;
		size--;
	}

	return(len);
}
#endif

/* ------------------------------------------------------------------------------------------------ */

#if defined(__GNUC__) && !defined(_read_r)
_ssize_t  _read_r(struct _reent *rreenn, int fd, void *vbuf, size_t size)
{
char    *buf;
uint32_t uu;

	buf = vbuf;
	do {
		alt_16550_fifo_level_get_rx(&G_UARThndl, &uu);
	} while (uu == 0);
	alt_16550_fifo_read(&G_UARThndl, &buf[0], 1);

	return(1);
}
#endif

/* ------------------------------------------------------------------------------------------------ */

int __putchar(int ccc)
{
	char     cc;
	uint32_t ss;
	uint32_t uu;

	cc = (char )ccc;
	alt_16550_fifo_size_get_tx(&G_UARThndl, &ss);
	do {
		alt_16550_fifo_level_get_tx(&G_UARThndl, &uu);
	} while (uu >= ss);
	alt_16550_fifo_write(&G_UARThndl, &cc, 1);

	return(0);
}

/* ------------------------------------------------------------------------------------------------ */

int GetKey(void)
{
	char     ccc;
	uint32_t uu;

	ccc = (char)0;
	alt_16550_fifo_level_get_rx(&G_UARThndl, &uu);
	if (uu != 0) {
		alt_16550_fifo_read(&G_UARThndl, &ccc, 1);
	}
	return((int)ccc);
}

/* ------------------------------------------------------------------------------------------------ */

#if defined(__GNUC__)

#elif defined(__CC_ARM)
struct __FILE
{
  int dummyVar; //Just for the sake of redefining __FILE, we won't we using it anyways ;)
};

FILE __stdout; //STDOUT
FILE __stdin;  //STDIN
int fputc(int c, FILE * stream) { if (c == '\n') __putchar('\r'); return __putchar(c); }

int fgetc(FILE * stream) { return GetKey(); }

#elif defined(__IAR_SYSTEMS_ICC__)

#endif

/* ------------------------------------------------------------------------------------------------ */
/* Dummies to remove linker warnings about function used when non semi-hosted						*/
/* ------------------------------------------------------------------------------------------------ */
#if  defined(__GNUC__)
int _close (int xxx)
{
	xxx = xxx;
	return(0);
}

/* ------------------------------------------------------------------------------------------------ */

off_t _lseek(int fildes, off_t offset, int whence)
{
	fildes = fildes;
	offset = offset;
	whence = whence;

	return(0);
}

/* ------------------------------------------------------------------------------------------------ */

int _fstat(int fildes, void *buf)
{
	fildes = fildes;
	buf    = buf;

	return(0);
}

/* ------------------------------------------------------------------------------------------------ */

int _isatty(int fd)
{
	fd = fd;

	return(0);
}
#endif
/* EOF */
