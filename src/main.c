#include <jendefs.h>
#include <AppHardwareApi.h>
#include <MMAC.h>
#include <xcv_pub.h>
#include <MicroSpecific.h>
#include "dbg.h"
#include "dbg_uart.h"
#include <string.h>
#include "Printf.h"
#include "UartBuffered.h"
#include "crc-ccitt.h"
#include "coordinator.h"
#include "utils.h"

// JN5169 air bit rate is 250kbits/s, baudrate (16-ary) is 62.5kBaud

// Rewrite serial communication:
//  Central pointer FIFO which aways points to wireshark packets, either:
//	- Read  FIFO: data coming from radio
//	- Write FIFO: data sent to radio
//	- General pupose: out of band data to send to wireshark (channel number, baud rate change, ...)

// Also timing is not very precise, as the timestamp is appended by the RX interrupt, which gives the time of the frame processing and not the time of arrival of the first data.
// uint32 u32MMAC_GetRxTime(void); has a resolution of 16 us

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

//#define DEBUG_PROTOCOL
//#define XIAOMI_SMART_BUTTON
#define XIAOMI_SMART_DOOR_SENSOR
#define HEARTBEAT_LED
//#define DEBUG_BAUDRATE		TRUE

#if DEBUG_BAUDRATE && !defined(DBG_ENABLE)
#warning("Please define TRACE=1 in Makefile to activate UART1 output.");
#endif

#define UART_TO_PC              E_AHI_UART_0        /* Uart to wireshark      */
#define UART_FOR_DEBUG          E_AHI_UART_1        /* Uart to debug terminal */

#define TRC_BUTTON_PRESS        FALSE
#define TRC_TIME_WRITE_BIN		TRUE
#define TRC_PC_COMMANDS			TRUE
#define TRC_RCV_TO_ISR_TIME		TRUE

#define BAUD_RATE               E_AHI_UART_RATE_115200 /* Baud rate to use   */

#define STARTUP_CHANNEL			20

#if defined(XIAOMI_SMART_BUTTON)
#define LED_PIN_BIT				(1 << 11)
#define MAIN_PIN_BIT			(1 << 16)
#define PAIR_PIN_BIT			(1 <<  0)
#elif defined(XIAOMI_SMART_DOOR_SENSOR)
#define LED_PIN_BIT				(1 << 11)
#define MAIN_PIN_BIT			(1 << 16)
#define PAIR_PIN_BIT			(1 <<  0)
#endif //def XIAOMI_SMART_BUTTON

typedef struct pcaprec_hdr_s {
        uint32_t ts_sec;         /* timestamp seconds */
        uint32_t ts_usec;        /* timestamp microseconds */
        uint32_t incl_len;       /* number of octets of packet saved in file */
        uint32_t orig_len;       /* actual length of packet */
} pcaprec_hdr_t;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

uint8 au8UartTxBuffer[1024];		// Well, this should be overkill at baudrates > 250k
uint8 au8UartRxBuffer[100];
//uint8 au8Uart1TxBuffer[100];
//uint8 au8Uart1RxBuffer[100];

uint8 g_u8Channel = STARTUP_CHANNEL;

volatile uint32_t g_u32Seconds = 0;
uint32_t g_u32SymbolCounter = 0;
uint32_t g_u32SymbolSeconds = 0;
#ifdef DEBUG_PROTOCOL
int g_iWSDumpStatus = 1;
#else
int g_iWSDumpStatus = 0;
#endif

const float afFrequencies[] = {
		2.405e6,
		2.410e6,
		2.415e6,
		2.420e6,
		2.425e6,
		2.430e6,
		2.435e6,
		2.440e6,
		2.445e6,
		2.450e6,
		2.455e6,
		2.460e6,
		2.465e6,
		2.470e6,
		2.475e6,
		2.480e6};

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

PRIVATE void vPutC(uint8 u8Data);
PRIVATE char acGetC(void);
PRIVATE void TickTimer_Cb(uint32_t u32Device, uint32_t u32ItemBitmap);
PRIVATE void SysCtrl_Cb(uint32_t u32Device, uint32_t u32ItemBitmap);

PRIVATE void WS_Send_Chan_Num(uint8_t u8Channel);
PRIVATE void WS_Send_Syntax_Error(const char * pBuffer);
PRIVATE void WS_Send_Test_Packet(void);

PRIVATE void vMMAC_Handler(uint32 u32Param);
#define NB_PHY_BUFFERS_RX	8
struct sTimedPHYFrame PHYBufferRx[NB_PHY_BUFFERS_RX];
volatile uint32_t g_u32PHYBufRxRIdx = 0;
volatile uint32_t g_u32PHYBufRxWIdx = 0;

struct sTimedPHYFrame PHYBufferTx[NB_PHY_BUFFERS_TX];
volatile uint32_t g_u32PHYBufTxRIdx = 0;
volatile uint32_t g_u32PHYBufTxWIdx = 0;
volatile uint32_t g_u32SendInProgress = 0;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void AppColdStart(void)
{
	uint32 u32Chip_Id = 0;(void)u32Chip_Id;
	uint8 u8Channelsave = 0;
	char bCharBuffer[64] = "";
	int iCharBufferPtr = 0;
	tsExtAddr sMacAddr;

	/* Turn off debugger */
	*(volatile uint32 *)0x020000a0 = 0;

    /* Disable watchdog if enabled by default */
#ifdef WATCHDOG_ENABLED
	vAHI_WatchdogStop();
#endif
	(void)u32AHI_Init();                              /* initialise hardware API */
	bAHI_SetClockRate(E_AHI_XTAL_32MHZ);
	vAHI_OptimiseWaitStates();

	volatile int n;
	for(n=0;n<100000 && !bAHI_Clock32MHzStable();n++){}      // wait for JN516X to move onto 32MHz Crystal

/* set up the tick timer, we'll use it for timestamps */
	vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_DISABLE);
	vAHI_TickTimerInit((void*)TickTimer_Cb);
	vAHI_TickTimerWrite(0x00000000);					// Starting  count
	vAHI_TickTimerInterval(16000000);					// Reference count 1Hz
	vAHI_TickTimerIntEnable(TRUE);
	vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_RESTART);
// Init UART 0 for wireshark
	vAHI_UartSetRTSCTS(UART_TO_PC, FALSE);
	vUartInit(UART_TO_PC, BAUD_RATE, au8UartTxBuffer, sizeof(au8UartTxBuffer), au8UartRxBuffer, sizeof(au8UartRxBuffer));/* uart for user interface */
#ifdef DEBUG_PROTOCOL
	vInitPrintf((void*)vPutC);	
	vPrintf("Cleartext log start\n");
#endif
// Init UART 1 for debug
	vAHI_UartSetLocation(UART_FOR_DEBUG, TRUE);
	vAHI_UartTxOnly(UART_FOR_DEBUG, TRUE);
	DBG_vUartInit(UART_FOR_DEBUG, DBG_E_UART_BAUD_RATE_115200);
	vAHI_UartSetBaudDivisor(UART_FOR_DEBUG, 1);
	vAHI_UartSetClocksPerBit(UART_FOR_DEBUG, 15);
	DBG_vPrintf(TRUE, "Cleartext log start\n");

// Lower UART ISR priority
	vAHI_InterruptSetPriority(MICRO_ISR_MASK_UART0 | MICRO_ISR_MASK_UART1, 1);		// 0 is ISR off, 1 is lowest priority, 15 is highest priority

	vMMAC_Enable();
	vMMAC_EnableInterrupts(vMMAC_Handler);
	vMMAC_ConfigureInterruptSources(E_MMAC_INT_TX_COMPLETE | E_MMAC_INT_RX_HEADER);
	vMMAC_ConfigureRadio();
	vMMAC_SetChannel(g_u8Channel);
	vMMAC_GetMacAddress(&sMacAddr);
	g_u64OurAddr = ((uint64_t)sMacAddr.u32H << 32) | sMacAddr.u32L;
#ifdef DEBUG_PROTOCOL
	vPrintf("MAC Address: %02x %02x %02x %02x %02x %02x %02x %02x", 
		(sMacAddr.u32H >> 24) & 0xFF, (sMacAddr.u32H >> 16) & 0xFF, (sMacAddr.u32H >>  8) & 0xFF, (sMacAddr.u32H >>  0) & 0xFF,
		(sMacAddr.u32L >> 24) & 0xFF, (sMacAddr.u32L >> 16) & 0xFF, (sMacAddr.u32L >>  8) & 0xFF, (sMacAddr.u32L >>  0) & 0xFF);
#endif //def DEBUG_PROTOCOL
	vMMAC_StartPhyReceive(&PHYBufferRx[g_u32PHYBufRxWIdx].sPHYFrame, E_MMAC_RX_START_NOW | E_MMAC_RX_ALLOW_FCS_ERROR);
	
	DBG_vUartInit(UART_FOR_DEBUG, DBG_E_UART_BAUD_RATE_115200);
	DBG_vPrintf(TRUE, "\nTRC: starting trace %d\n", u32MMAC_GetRxTime());
	DBG_vPrintf(TRUE, "bAHI_GetClkSource=%d\n", bAHI_GetClkSource());
	DBG_vPrintf(TRUE, "u8AHI_GetSystemClkRate=%d\n", u8AHI_GetSystemClkRate());

#if defined(HEARTBEAT_LED) || defined(ISR_LED)
	vAHI_DioSetDirection(0x00000000, LED_PIN_BIT);		// Set DIO11 as output (LED)
#endif	//defined(HEARTBEAT_LED) || defined(ISR_LED)

#ifdef XIAOMI_SMART_BUTTON
	vAHI_DioSetPullup(~MAIN_PIN_BIT, MAIN_PIN_BIT);  /* turn all pullups on except for DIO16 which is big button input      */
#else
	vAHI_DioSetPullup(0xffffffff, 0x00000000);  /* turn all pullups on      */
#endif

	vAHI_SysCtrlRegisterCallback(SysCtrl_Cb);
	vAHI_DioInterruptEdge(0x00000000, 0x00000080);				// uint32 u32Rising , uint32 u32Falling, RXD0 is on DIO7
	vAHI_DioInterruptEnable(0x00000080, 0x00000000);			// uint32 u32Enable, uint32 u32Disable

	/* read Chip_ID register */
	u32Chip_Id= READ_REG32(0x020000fc);

	while(1){
		if ((g_u8Channel != u8Channelsave) && g_iWSDumpStatus > 0){
			WS_Send_Chan_Num(g_u8Channel);
			u8Channelsave = g_u8Channel;
		}

		uint32_t u32PHYBufRIdx = g_u32PHYBufRxRIdx;
		if(g_u32PHYBufRxWIdx!=u32PHYBufRIdx){
#if !defined(DO_COORD_JOB)
			if(g_iWSDumpStatus > 0){
				Dump_Packet(&PHYBufferRx[u32PHYBufRIdx]);
			}
#else
			DoCoordJob(&PHYBufferRx[u32PHYBufRIdx].sPHYFrame);
#endif
			g_u32PHYBufRxRIdx = (u32PHYBufRIdx + 1) % NB_PHY_BUFFERS_RX;
		}

#ifdef HEARTBEAT_LED
		static int t_r = -1;
		if(t_r!=g_u32Seconds){
			t_r = g_u32Seconds;
			if(t_r & 1){
				vAHI_DioSetOutput(0x00000000, LED_PIN_BIT);		// Set DIO11 (LED)	ON
			}else{
				vAHI_DioSetOutput(LED_PIN_BIT, 0x00000000);		// Set DIO11 (LED)	OFF
			}
		}
#endif	//def HEARTBEAT_LED

#ifdef XIAOMI_SMART_BUTTON
		if(!(u32AHI_DioReadInput()&MAIN_PIN_BIT)){
			DBG_vPrintf(TRC_BUTTON_PRESS, "Main button\n");
		}
		if(!(u32AHI_DioReadInput()&PAIR_PIN_BIT)){
			DBG_vPrintfTRC_BUTTON_PRESS, ("Pair button\n");
		}
#endif

		if(bUartRxDataAvailable(UART_TO_PC)){
			char acKey = acGetC();

			if(iCharBufferPtr<sizeof(bCharBuffer)-1){
				bCharBuffer[iCharBufferPtr++] = acKey;
			}
			if(acKey=='\n'){
				if(bCharBuffer[0]=='C' && bCharBuffer[1]==':' && isdigit(bCharBuffer[2]) && isdigit(bCharBuffer[3])){
					int chan = (bCharBuffer[2] - '0')*10 + bCharBuffer[3] - '0';
					if(chan>=11 && chan<=26){
						g_u8Channel = chan;
						vMMAC_SetChannel(g_u8Channel);			//!!! TODO: check if Rx function must be called again after changing channel.
					}
					u8Channelsave = 0;
				}else if(bCharBuffer[0]=='B' && bCharBuffer[1]=='R' && bCharBuffer[2]=='Q'){
					ZB_Send_Beacon_Request();
// ------------ DEBUG ------------
#ifdef DO_COORD_JOB
				}else if(bCharBuffer[0]=='S' && bCharBuffer[1]=='B' && bCharBuffer[2]=='Q'){
					ZB_Send_Beacon();
#endif	//def DO_COORD_JOB
// ------------ DEBUG ------------
				}else if(bCharBuffer[0]=='S' && bCharBuffer[1]=='T' && bCharBuffer[2]=='O'){
					g_iWSDumpStatus = 0;
				}else if(bCharBuffer[0]=='I' && bCharBuffer[1]=='N' && bCharBuffer[2]=='I'){
					if(bCharBuffer[3]==':' && isdigit(bCharBuffer[4]) && isdigit(bCharBuffer[5])){
						int chan = (bCharBuffer[4] - '0')*10 + bCharBuffer[5] - '0';
						if(chan>=11 && chan<=26){
							g_u8Channel = chan;
							vMMAC_SetChannel(g_u8Channel);			//!!! TODO: check if Rx function must be called again after changing channel.
						}else{
							DBG_vPrintf(TRC_PC_COMMANDS, "Wrong chan=%d number (should be between 11 and 26 inclusive)\n", chan);
						}
					}
					g_iWSDumpStatus = 1;
					u8Channelsave = 0;
				}else if(bCharBuffer[0]=='B' && bCharBuffer[1]=='R' && bCharBuffer[2]=='D' && bCharBuffer[3]==':'){
					int ptr = 4;
					int baudrate = 0;
					while((ptr<iCharBufferPtr) && isdigit(bCharBuffer[ptr])){
						baudrate *= 10;
						baudrate += bCharBuffer[ptr] - '0';
						ptr++;
					}
					if(baudrate==1000000){
						for(n=0;n<100000;n++){}      // wait a bit
						vAHI_UartSetBaudDivisor(UART_TO_PC, 1);
						vAHI_UartSetClocksPerBit(UART_TO_PC, 15);
						for(n=0;n<100000;n++){}      // wait a bit
						WS_Send_Syntax_Error("Baudrate set to 1Mbaud");
					}else{
//						WS_Send_Syntax_Error("Invalid baudrate");
						bCharBuffer[iCharBufferPtr] = 0;
						WS_Send_Syntax_Error(bCharBuffer);
					}
				}else if(bCharBuffer[0]=='S' && bCharBuffer[1]=='T' && bCharBuffer[2]=='A'){
					if(bCharBuffer[3]==':' && isdigit(bCharBuffer[4]) && isdigit(bCharBuffer[5])){
						int chan = (bCharBuffer[4] - '0')*10 + bCharBuffer[5] - '0';
						if(chan>=11 && chan<=26){
							g_u8Channel = chan;
							vMMAC_SetChannel(g_u8Channel);			//!!! TODO: check if Rx function must be called again after changing channel.
							DBG_vPrintf(TRC_PC_COMMANDS, "Start chan=%d\n", chan);
						}else{
							DBG_vPrintf(TRC_PC_COMMANDS, "Wrong chan=%d number (should be between 11 and 26 inclusive)\n", chan);
						}
					}else{
						DBG_vPrintf(TRC_PC_COMMANDS, "Start\n");
					}
					g_iWSDumpStatus = 1;
					u8Channelsave = 0;
				}else if(bCharBuffer[0]=='T' && bCharBuffer[1]=='S' && bCharBuffer[2]=='T'){
					WS_Send_Test_Packet();
				}else if(iCharBufferPtr>0){
					bCharBuffer[iCharBufferPtr] = 0;
// Send this whether we are initiated or not
					WS_Send_Syntax_Error(bCharBuffer);
				}
				iCharBufferPtr = 0;
			}
		}
	}
}

PUBLIC void AppWarmStart(void)
{
	AppColdStart();
}

PRIVATE void vMMAC_Handler(uint32 u32Param)
{
//	E_MMAC_INT_TX_COMPLETE
//	E_MMAC_INT_RX_HEADER
//	E_MMAC_INT_RX_COMPLETE
#if defined(ISR_LED)
	vAHI_DioSetOutput(0x00000000, LED_PIN_BIT);		// Set DIO11 (LED)	ON
#endif
	if(u32Param & E_MMAC_INT_RX_HEADER){
		uint32_t u32CurWIdx = g_u32PHYBufRxWIdx;
		uint32_t u32SymbolTime = u32MMAC_GetRxTime();
// Worst case for this loop is the number of seconds since the last received frame, which should anyway be bounded
		while(u32SymbolTime-g_u32SymbolCounter>62500){
			g_u32SymbolCounter += 62500;
			g_u32SymbolSeconds++;
		}
		PHYBufferRx[u32CurWIdx].u32MicroSeconds = cpu_to_le32((u32SymbolTime-g_u32SymbolCounter) * 16);
		PHYBufferRx[u32CurWIdx].u32Seconds = cpu_to_le32(g_u32SymbolSeconds);
		uint32_t u32NextWIdx = (u32CurWIdx+1) % NB_PHY_BUFFERS_RX;
		if(u32NextWIdx==g_u32PHYBufRxRIdx){
//			DBG_vPrintf(TRUE, "RX error, buffer overflow!\n");
		}else{
			vMMAC_StartPhyReceive(&PHYBufferRx[u32NextWIdx].sPHYFrame, E_MMAC_RX_START_NOW | E_MMAC_RX_ALLOW_FCS_ERROR);
			g_u32PHYBufRxWIdx = u32NextWIdx;
		}
//	}else if(u32Param & E_MMAC_INT_TX_COMPLETE){
	}
	if(u32Param & E_MMAC_INT_TX_COMPLETE){
		uint32_t u32PHYBufRIdx = g_u32PHYBufTxRIdx;
		if(u32PHYBufRIdx!=g_u32PHYBufTxWIdx){
			vMMAC_StartPhyTransmit(&PHYBufferTx[u32PHYBufRIdx].sPHYFrame, E_MMAC_TX_START_NOW | E_MMAC_TX_NO_CCA);
			g_u32PHYBufTxRIdx = (u32PHYBufRIdx + 1) % NB_PHY_BUFFERS_TX;
		}else{
			vMMAC_StartPhyReceive(&PHYBufferRx[g_u32PHYBufRxWIdx].sPHYFrame, E_MMAC_RX_START_NOW | E_MMAC_RX_ALLOW_FCS_ERROR);
			g_u32SendInProgress = 0;
		}
	}else{
//		DBG_vPrintf(TRUE, "vMMAC_Handler(%08x)\n", u32Param);
	}
#if defined(ISR_LED)
	vAHI_DioSetOutput(LED_PIN_BIT, 0x00000000);		// Set DIO11 (LED)	OFF
#endif
}

// TODO: decode and display stack frame
#define exPutC(c)	vAHI_UartWriteData(UART_TO_PC, c)
PUBLIC void vException_BusError(uint32 u32StackPointer, uint32 u32Vector)
{
//	vPrintf("\nvException_BusError\n");
	exPutC('\n'); exPutC('0'); exPutC('v'); exPutC('E'); exPutC('x'); exPutC('c'); exPutC('e'); exPutC('p'); exPutC('t'); exPutC('i'); exPutC('o'); exPutC('n'); exPutC('_');
	exPutC('B');  exPutC('u'); exPutC('s'); exPutC('E'); exPutC('r'); exPutC('r'); exPutC('o'); exPutC('r'); exPutC('\n');
}

PUBLIC void vException_UnalignedAccess(uint32 u32StackPointer, uint32 u32Vector)
{
//	vPrintf("\nvException_UnalignedAccess\n");
	exPutC('\n'); exPutC('1'); exPutC('v'); exPutC('E'); exPutC('x'); exPutC('c'); exPutC('e'); exPutC('p'); exPutC('t'); exPutC('i'); exPutC('o'); exPutC('n'); exPutC('_');
	exPutC('U');  exPutC('n'); exPutC('a'); exPutC('l'); exPutC('i'); exPutC('g'); exPutC('n'); exPutC('e'); exPutC('d'); exPutC('A'); exPutC('c'); exPutC('c'); exPutC('e'); exPutC('s'); exPutC('s'); exPutC('\n');
}

PUBLIC void vException_IllegalInstruction(uint32 u32StackPointer, uint32 u32Vector)
{
//	vPrintf("\nvException_IllegalInstruction\n");
	exPutC('\n'); exPutC('2'); exPutC('v'); exPutC('E'); exPutC('x'); exPutC('c'); exPutC('e'); exPutC('p'); exPutC('t'); exPutC('i'); exPutC('o'); exPutC('n'); exPutC('_');
	exPutC('I');  exPutC('l'); exPutC('l'); exPutC('e'); exPutC('g'); exPutC('a'); exPutC('l'); exPutC('I'); exPutC('n'); exPutC('s'); exPutC('t'); exPutC('r'); exPutC('u'); exPutC('c'); exPutC('t'); exPutC('i'); exPutC('o'); exPutC('n'); exPutC('\n');
}

PUBLIC void vException_SysCall(uint32 u32StackPointer, uint32 u32Vector)
{
//	vPrintf("\nvException_SysCall\n");
	exPutC('\n'); exPutC('3'); exPutC('v'); exPutC('E'); exPutC('x'); exPutC('c'); exPutC('e'); exPutC('p'); exPutC('t'); exPutC('i'); exPutC('o'); exPutC('n'); exPutC('_');
	exPutC('S');  exPutC('y'); exPutC('s'); exPutC('C'); exPutC('a'); exPutC('l'); exPutC('l'); exPutC('\n');
}

PUBLIC void vException_Trap(uint32 u32StackPointer, uint32 u32Vector)
{
//	vPrintf("\nvException_Trap\n");
	exPutC('\n'); exPutC('4'); exPutC('v'); exPutC('E'); exPutC('x'); exPutC('c'); exPutC('e'); exPutC('p'); exPutC('t'); exPutC('i'); exPutC('o'); exPutC('n'); exPutC('_');
	exPutC('T');  exPutC('r'); exPutC('a'); exPutC('p'); exPutC('\n');
}

PUBLIC void vException_StackOverflow(uint32 u32StackPointer, uint32 u32Vector)
{
//	vPrintf("\nvException_StackOverflow\n");
	exPutC('\n'); exPutC('5'); exPutC('v'); exPutC('E'); exPutC('x'); exPutC('c'); exPutC('e'); exPutC('p'); exPutC('t'); exPutC('i'); exPutC('o'); exPutC('n'); exPutC('_');
	exPutC('S');  exPutC('t'); exPutC('a'); exPutC('c'); exPutC('k'); exPutC('O'); exPutC('v'); exPutC('e'); exPutC('r'); exPutC('f'); exPutC('l'); exPutC('o'); exPutC('w'); exPutC('\n');
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

PRIVATE void vPutC(uint8 u8Data)
{
	vUartWrite(UART_TO_PC, u8Data);
}

PRIVATE char acGetC(void)
{
	return(u8UartRead(UART_TO_PC));
}

// 68719x62500=0xFFFF8B9C	68720x62500=0x00007FC0
PRIVATE void TickTimer_Cb(uint32_t u32Device, uint32_t u32ItemBitmap)
{
	g_u32Seconds++;
// This processing is done in the Rx ISR, because the loop will be taken a number of times equal to the number of seconds elapsed since the last received packet.
//	uint32_t u32SymbolTime = u32MMAC_GetTime();
//	while(u32SymbolTime-g_u32SymbolCounter>62500){
//		g_u32SymbolCounter += 62500;
//		g_u32SymbolSeconds++;
//	}
}

PRIVATE void SysCtrl_Cb(uint32_t u32Device, uint32_t u32ItemBitmap)
{
	if(u32ItemBitmap & E_AHI_DIO7_INT){
	}
}

PRIVATE void WS_Send_Chan_Num(uint8_t u8Channel)
{
	pcaprec_hdr_t pcap_rec_hdr = {.ts_sec=0, .ts_usec=0, .incl_len=0, .orig_len=0};
	char dataFrame[16];
	int lenval = 0;
	uint32_t u32Seconds;
	uint32_t u32Fraction;
	do{
		u32Seconds  = g_u32Seconds;
		u32Fraction = u32AHI_TickTimerRead();
	}while(u32Seconds!=g_u32Seconds);
	pcap_rec_hdr.ts_sec  = cpu_to_le32(u32Seconds);
	pcap_rec_hdr.ts_usec = cpu_to_le32(u32Fraction/16);		// 16 MHz
	dataFrame[lenval++] = 0x07;					// Unknown packet type
	dataFrame[lenval++] = 0x00;					// Unknown packet type
	dataFrame[lenval++] = 0x00;					// Sequence number
	dataFrame[lenval++] = u8Channel;
	float fFreq = afFrequencies[u8Channel - 11];
	dataFrame[lenval++] = ((uint8_t*)&fFreq)[0];
	dataFrame[lenval++] = ((uint8_t*)&fFreq)[1];
	dataFrame[lenval++] = ((uint8_t*)&fFreq)[2];
	dataFrame[lenval++] = ((uint8_t*)&fFreq)[3];
	pcap_rec_hdr.incl_len = cpu_to_le32(lenval);
	pcap_rec_hdr.orig_len = cpu_to_le32(lenval);
	int ws_snd_cnt;
	for(ws_snd_cnt=0;ws_snd_cnt<sizeof(pcap_rec_hdr);ws_snd_cnt++){
		vPutC(((uint8_t*)&pcap_rec_hdr)[ws_snd_cnt]);
	}
	for(ws_snd_cnt=0;ws_snd_cnt<lenval;ws_snd_cnt++){
		vPutC(((uint8_t*)&dataFrame)[ws_snd_cnt]);
	}
}

PRIVATE void WS_Send_Syntax_Error(const char * pBuffer)
{
	pcaprec_hdr_t pcap_rec_hdr = {.ts_sec=0, .ts_usec=0, .incl_len=0, .orig_len=0};
	char dataFrame[300];
	int lenval = 0;
	uint32_t u32Seconds;
	uint32_t u32Fraction;
	do{
		u32Seconds  = g_u32Seconds;
		u32Fraction = u32AHI_TickTimerRead();
	}while(u32Seconds!=g_u32Seconds);
	pcap_rec_hdr.ts_sec  = cpu_to_le32(u32Seconds);
	pcap_rec_hdr.ts_usec = cpu_to_le32(u32Fraction/16);		// 16 MHz
	dataFrame[lenval++] = 0x07;					// Unknown packet type
	dataFrame[lenval++] = 0x00;					// Unknown packet type
	dataFrame[lenval++] = 0x01;					// Sequence number
	int sptr = 0;
	if(!(g_iWSDumpStatus > 0)){
		const char * errstr = "!!!wireshark communication not initialized!!!\n";
		while(lenval<sizeof(dataFrame)-1){				// -1 for terminating 0
			if(!errstr[sptr]){break;}
			dataFrame[lenval++] = errstr[sptr++];
		}
	}
	int bptr = 0;
	while(lenval<sizeof(dataFrame)-1){				// -1 for terminating 0
		if(!pBuffer[bptr]){break;}
		dataFrame[lenval++] = pBuffer[bptr++];
	}
	dataFrame[lenval++] = 0;
	pcap_rec_hdr.incl_len = cpu_to_le32(lenval);
	pcap_rec_hdr.orig_len = cpu_to_le32(lenval);
	int ws_snd_cnt;
	for(ws_snd_cnt=0;ws_snd_cnt<sizeof(pcap_rec_hdr);ws_snd_cnt++){
		vPutC(((uint8_t*)&pcap_rec_hdr)[ws_snd_cnt]);
	}
	for(ws_snd_cnt=0;ws_snd_cnt<lenval;ws_snd_cnt++){
		vPutC(((uint8_t*)&dataFrame)[ws_snd_cnt]);
	}
}

PRIVATE void WS_Send_Test_Packet(void)
{
	uint8_t u8tstFrame[] = {
// Entete du packet
		0x3e, 0xd7, 0xe8, 0x59, 0x92, 0x27, 0x0f, 0x00, 0x2f, 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00, 0x00, 
// Packet lui meme.
		0x41, 0x88, 0xb5, 0x35, 0xd4, 0xff, 0xff, 0x00, 0x00, 0x09, 0x12, 0xfc, 0xff, 0x00, 0x00, 0x01, 
		0x50, 0xff, 0x81, 0x63, 0x01, 0x00, 0x8d, 0x15, 0x00, 0x28, 0x1b, 0x85, 0x00, 0x00, 0xff, 0x81,
		0x63, 0x01, 0x00, 0x8d, 0x15, 0x00, 0x01, 0x92, 0x26, 0x14, 0x1b, 0xc4, 0x68, 
// A mettre si format de packet inclut FCS
		0xc3, 0x1d
	};
	uint32_t u32Seconds;
	uint32_t u32Fraction;
	uint32_t u32Micros;
	do{
		u32Seconds  = g_u32Seconds;
		u32Fraction = u32AHI_TickTimerRead();
	}while(u32Seconds!=g_u32Seconds);
	u32Micros = u32Fraction/16;
	u8tstFrame[0] = ((uint8_t*)&u32Seconds)[3];
	u8tstFrame[1] = ((uint8_t*)&u32Seconds)[2];
	u8tstFrame[2] = ((uint8_t*)&u32Seconds)[1];
	u8tstFrame[3] = ((uint8_t*)&u32Seconds)[0];
	u8tstFrame[4] = ((uint8_t*)&u32Micros)[3];
	u8tstFrame[5] = ((uint8_t*)&u32Micros)[2];
	u8tstFrame[6] = ((uint8_t*)&u32Micros)[1];
	u8tstFrame[7] = ((uint8_t*)&u32Micros)[0];

	int ws_snd_cnt;
	for(ws_snd_cnt=0;ws_snd_cnt<sizeof(u8tstFrame);ws_snd_cnt++){
		vPutC(u8tstFrame[ws_snd_cnt]);
	}
}

PUBLIC void Dump_Packet(struct sTimedPHYFrame * pTimedPHYPacket)
{
#ifndef DEBUG_PROTOCOL
	uint32_t u32T0 = u32AHI_TickTimerRead();
	pTimedPHYPacket->u32IncLen = cpu_to_le32(pTimedPHYPacket->sPHYFrame.u8PayloadLength);
	uint32_t u32NbToWrite = pTimedPHYPacket->sPHYFrame.u8PayloadLength+4*sizeof(uint32_t);
	uint32_t u32NbWritten = u32UartWriteBinary(UART_TO_PC, (uint8_t *)pTimedPHYPacket, u32NbToWrite);
	if(u32NbWritten!=u32NbToWrite){
		DBG_vPrintf(TRUE, "TX buffer full!\n");
		int cnt;
		uint8_t * pPayload = (uint8_t *)pTimedPHYPacket;
		for(cnt = u32NbWritten; cnt < u32NbToWrite; cnt++){
			vUartWrite(UART_TO_PC, pPayload[cnt]);
		}
	}
	u32T0 = u32AHI_TickTimerRead() - u32T0;
//	DBG_vPrintf(TRC_TIME_WRITE_BIN, "Took %d us to dump a %d bytes long packet\n", u32T0, pTimedPHYPacket->sPHYFrame.u8PayloadLength + 4*sizeof(uint32_t));

#else	
	vPrintf("    Length: %d\n", pPHYPacket->u8PayloadLength);
	vPrintf("\t");
	int cnt;
	for(cnt=0;cnt<pPHYPacket->u8PayloadLength;cnt++){
		vPrintf("%02x ", pPHYPacket->uPayload.au8Byte[cnt]);
	}
	vPrintf("\n");
#endif
}


