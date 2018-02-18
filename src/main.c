#include <jendefs.h>
#include <AppHardwareApi.h>
#include <AppApi.h>
#include <JPT.h>
#include <string.h>
#include "Printf.h"
#include "UartBuffered.h"
#include "crc-ccitt.h"

// JN5169 air bit rate is 250kbits/s

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define DBG_vPrintf(s, ...)	vPrintf(__VA_ARGS__)
#define DBG_E_UART_BAUD_RATE_38400	38400
#define DBG_vUartInit(...)

#define DEBUG_PROTOCOL
//#define XIAOMI_SMART_BUTTON
#define XIAOMI_SMART_DOOR_SENSOR
#define HEARTBEAT_LED
#define DO_COORD_JOB
//#define DEBUG_BAUDRATE		TRUE

#if DEBUG_BAUDRATE && !defined(DBG_ENABLE)
#warning("Please define TRACE=1 in Makefile to activate UART1 output.");
#endif

#define UART_TO_PC              E_AHI_UART_0        /* Uart to PC           */
#define UART_FOR_DEBUG			E_AHI_UART_1        /* Tx-only Uart for debug */

#define BAUD_RATE               E_AHI_UART_RATE_38400 /* Baud rate to use   */

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

#define READ_REG32(A)     *(volatile uint32 *)(A)

#define swap32(x)	__builtin_bswap32((x))
//#define swap16(x)	__builtin_bswap16((x))
#define swap16(x)	((((x) & 0xFF) << 8) | (((x) >> 8) & 0xFF))

#define isdigit(x)	(((x)>='0') && ((x)<='9'))

#define ARRAY_SIZE(arr)	(sizeof(arr)/sizeof(*arr))

typedef struct pcap_hdr_s {
        uint32_t magic_number;   /* magic number */
        uint16_t version_major;  /* major version number */
        uint16_t version_minor;  /* minor version number */
         int32_t thiszone;       /* GMT to local correction */
        uint32_t sigfigs;        /* accuracy of timestamps */
        uint32_t snaplen;        /* max length of captured packets, in octets */
        uint32_t network;        /* data link type */
} pcap_hdr_t;
//} __attribute__((packed)) pcap_hdr_t;

typedef struct pcaprec_hdr_s {
        uint32_t ts_sec;         /* timestamp seconds */
        uint32_t ts_usec;        /* timestamp microseconds */
        uint32_t incl_len;       /* number of octets of packet saved in file */
        uint32_t orig_len;       /* actual length of packet */
} pcaprec_hdr_t;
//} __attribute__((packed)) pcaprec_hdr_t;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

uint8 au8UartTxBuffer[1024];		// Well, this should be overkill at baudrates > 250k
uint8 au8UartRxBuffer[100];
uint8 au8Uart1TxBuffer[100];
uint8 au8Uart1RxBuffer[100];

uint8 u8TxPowerAdj	  = 0;
uint8 u8Attenuator3dB = 0;

uint8 g_u8Channel = STARTUP_CHANNEL;

volatile uint32_t g_u32Seconds = 0;
volatile uint32_t g_u32BaudRateCalced = 0;
volatile uint32_t g_au32FallingTimes[16];
volatile uint32_t g_u32FallingTimesWPtr = 0;
int g_iWSDumpStatus = 0;

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
PRIVATE void vPutC1(uint8 u8Data);
PRIVATE char acGetC(void);
PRIVATE void TickTimer_Cb(uint32_t u32Device, uint32_t u32ItemBitmap);
PRIVATE void SysCtrl_Cb(uint32_t u32Device, uint32_t u32ItemBitmap);
PRIVATE void WS_init(void);
PRIVATE void WS_Send_Chan_Num(uint8_t u8Channel);
PRIVATE void WS_Send_Syntax_Error(const char * pBuffer);
PRIVATE void WS_Send_Test_Packet(void);
PRIVATE void WS_Dump_Packet(tsJPT_PT_Packet * psPacket);
#ifdef DEBUG_PROTOCOL
PRIVATE void ClearText_Dump_Packet(tsJPT_PT_Packet * psPacket);
#endif	// def DEBUG_PROTOCOL
#ifdef DO_COORD_JOB
PRIVATE void DoCoordJob(tsJPT_PT_Packet * psPacket);
#endif	//def DO_COORD_JOB

PRIVATE void ZB_Send_Beacon(void);
PRIVATE void ZB_Send_Beacon_Request(void);
PRIVATE void ZB_Send_Ack(uint8_t seqnum);
PRIVATE void ZB_Send_Asso_Resp(void);
PRIVATE void ZB_JumpChannel(void);

PRIVATE MAC_DcfmIndHdr_s * prMlmeGetBuffer(void * pParam);
PRIVATE void prMlmeCallback(void * pParam, MAC_DcfmIndHdr_s *);
PRIVATE MAC_DcfmIndHdr_s * prMcpsGetBuffer(void *);
PRIVATE void prMcpsCallback(void *, MAC_DcfmIndHdr_s *);

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void AppColdStart(void)
{
	uint32 u32Chip_Id = 0;(void)u32Chip_Id;
	uint32_t u32StackVersion = 0;(void)u32Chip_Id;
	uint8 u8Channelsave = 0;
	tsJPT_PT_Packet sPacket;
	char bCharBuffer[64] = "";
	int iCharBufferPtr = 0;

	/* Turn off debugger */
	*(volatile uint32 *)0x020000a0 = 0;

    /* Disable watchdog if enabled by default */
#ifdef WATCHDOG_ENABLED
	vAHI_WatchdogStop();
#endif

#if 0
    /* Setup interface to MAC */
    (void)u32AHI_Init();                              /* initialise hardware API */
    u32StackVersion = u32AppQApiInit(NULL, NULL, NULL);

    /* Initialise coordinator state */
    sCoordinatorData.eState = E_STATE_IDLE;
    sCoordinatorData.u8TxPacketSeqNb  = 0;
    sCoordinatorData.u8RxPacketSeqNb  = 0;
    sCoordinatorData.u16NbrEndDevices = 0;

    /* Set up the MAC handles. Must be called AFTER u32AppQApiInit() */
    s_pvMac = pvAppApiGetMacHandle();
    s_psMacPib = MAC_psPibGetHandle(s_pvMac);

    /* Set Pan ID and short address in PIB (also sets match registers in hardware) */
    MAC_vPibSetPanId(s_pvMac, PAN_ID);
    MAC_vPibSetShortAddr(s_pvMac, COORDINATOR_ADR);

    /* Enable receiver to be on when idle */
    MAC_vPibSetRxOnWhenIdle(s_pvMac, TRUE, FALSE);

    /* Allow nodes to associate */
    s_psMacPib->bAssociationPermit = 1;
#endif

    (void)u32AHI_Init();                              /* initialise hardware API */

	volatile int n;
	for(n=0;n<100000;n++){}      // wait for JN516X to move onto 32MHz Crystal

/* set up the tick timer, we'll use it for timestamps */
	vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_DISABLE);
	vAHI_TickTimerInit((void*)TickTimer_Cb);
	vAHI_TickTimerWrite(0x00000000);					// Starting  count
	vAHI_TickTimerInterval(16000000);					// Reference count 1Hz
	vAHI_TickTimerIntEnable(TRUE);
	vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_RESTART);

	vAHI_UartSetRTSCTS(UART_TO_PC, FALSE);
	vUartInit(UART_TO_PC, BAUD_RATE, au8UartTxBuffer, sizeof(au8UartTxBuffer), au8UartRxBuffer, sizeof(au8UartRxBuffer));/* uart for user interface */
#ifdef DEBUG_PROTOCOL
	vInitPrintf((void*)vPutC);	
	vPrintf("Cleartext log start\n");
#endif

	u32StackVersion = u32AppApiInit(prMlmeGetBuffer, prMlmeCallback, NULL, prMcpsGetBuffer, prMcpsCallback, NULL);
	vPrintf("Stack version: %d.%d\n", u32StackVersion >> 16, u32StackVersion & 0xFFFF);

	void      *mac;
	MAC_Pib_s *pib;	
	MAC_MlmeReqRsp_s  mlmereq;
	MAC_MlmeSyncCfm_s mlmecfm;
	/* get mac and pib handles */
	mac   = pvAppApiGetMacHandle();
	pib   = MAC_psPibGetHandle(mac);

// Reset?
	MAC_vPibSetShortAddr(mac, 0x0001);		// A voir
//	MAC_vPibSetPromiscuousMode(mac, TRUE, TRUE);
	memcpy(&pib->sCoordExtAddr, pvAppApiGetMacAddrLocation(), sizeof(MAC_ExtAddr_s));
	mlmereq.u8Type = MAC_MLME_REQ_START;
	mlmereq.u8ParamLength = sizeof(MAC_MlmeReqScan_s);
	mlmereq.uParam.sReqStart.u16PanId          = 0xCAFE;
	mlmereq.uParam.sReqStart.u8Channel         = g_u8Channel;
	mlmereq.uParam.sReqStart.u8BeaconOrder     = 15;
	mlmereq.uParam.sReqStart.u8SuperframeOrder = 15;
	mlmereq.uParam.sReqStart.u8PanCoordinator  = TRUE;
	mlmereq.uParam.sReqStart.u8BatteryLifeExt  = FALSE;
	mlmereq.uParam.sReqStart.u8Realignment     = FALSE;
	mlmereq.uParam.sReqStart.u8SecurityEnable  = FALSE;
	vAppApiMlmeRequest(&mlmereq, &mlmecfm);
	vPrintf("    vAppApiMlmeRequest returned %d\n", mlmecfm.uParam.sCfmStart.u8Status);		// MAC_Enum_e -> MAC_ENUM_SUCCESS = 0
	MAC_vPibSetPromiscuousMode(mac, TRUE, FALSE);
    /* Enable receiver to be on when idle */
    MAC_vPibSetRxOnWhenIdle(mac, TRUE, FALSE);


#ifdef HEARTBEAT_LED
	vAHI_DioSetDirection(0x00000000, LED_PIN_BIT);		// Set DIO11 as output (LED)
#endif	//def HEARTBEAT_LED

#ifdef XIAOMI_SMART_BUTTON
	vAHI_DioSetPullup(~MAIN_PIN_BIT, MAIN_PIN_BIT);  /* turn all pullups on except for DIO16 which is big button input      */
#else
	vAHI_DioSetPullup(0xffffffff, 0x00000000);  /* turn all pullups on      */
#endif

#ifdef DBG_ENABLE
	vAHI_UartSetLocation(UART_FOR_DEBUG, TRUE);
	vAHI_UartTxOnly(UART_FOR_DEBUG, TRUE);
#if 1
	vUartInit(UART_FOR_DEBUG, BAUD_RATE, au8Uart1TxBuffer, sizeof(au8Uart1TxBuffer), NULL, 0);/* uart for user interface */
	vInitPrintf((void*)vPutC1);	
#else
	/* Send debug output to DBG_UART */
	DBG_vUartInit ( UART_FOR_DEBUG, DBG_E_UART_BAUD_RATE_38400 );
#endif
#endif
//	DBG_vUartInit ( UART_FOR_DEBUG, DBG_E_UART_BAUD_RATE_115200 );
//	DBG_vPrintf(DEBUG_BAUDRATE, "\nBEN: First Trace in app_start.c->main()->Youppee\n");
//	DBG_vPrintf(TRUE, "\nBEN: First Trace in app_start.c->main()->Youppee\n");
//	DBG_vPrintf(1, "\nBEN: First Trace in app_start.c->main()->Youppee\n");

	vAHI_SysCtrlRegisterCallback(SysCtrl_Cb);
	vAHI_DioInterruptEdge(0x00000000, 0x00000080);				// uint32 u32Rising , uint32 u32Falling, RXD0 is on DIO7
	vAHI_DioInterruptEnable(0x00000080, 0x00000000);			// uint32 u32Enable, uint32 u32Disable

	/* read Chip_ID register */
	u32Chip_Id= READ_REG32(0x020000fc);

	u8TxPowerAdj = 0;
	u8Attenuator3dB = 0;

	while(1){
		if ((g_u8Channel != u8Channelsave) && g_iWSDumpStatus > 0){
			WS_Send_Chan_Num(g_u8Channel);
			u8Channelsave = g_u8Channel;
		}
/*		if(bJPT_PacketRx(g_u8Channel, &sPacket)){
#ifdef DEBUG_PROTOCOL
			ClearText_Dump_Packet(&sPacket);
#else
			if(g_iWSDumpStatus > 0){
				WS_Dump_Packet(&sPacket);
			}
#endif
#if defined(DO_COORD_JOB)
			DoCoordJob(&sPacket);
#endif
		}*/

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
			vPrintf("Main button\n");
		}
		if(!(u32AHI_DioReadInput()&PAIR_PIN_BIT)){
			vPrintf("Pair button\n");
		}
#endif

//AFAC		vPrintf("DIO: %08x\n", u32AHI_DioReadInput());

		if(bUartRxDataAvailable(UART_TO_PC)){
			char acKey = acGetC();

			if(iCharBufferPtr<sizeof(bCharBuffer)-1){
				bCharBuffer[iCharBufferPtr++] = acKey;
			}
			if(acKey=='\n'){
				if(bCharBuffer[0]=='C' && bCharBuffer[1]==':' && isdigit(bCharBuffer[2]) && isdigit(bCharBuffer[3])){
					int chan = (bCharBuffer[2] - '0')*10 + bCharBuffer[3] - '0';
					if(chan>=11 && chan<=26){
					}
					u8Channelsave = 0;
				}else if(bCharBuffer[0]=='B' && bCharBuffer[1]=='R' && bCharBuffer[2]=='Q'){
					ZB_Send_Beacon_Request();
// ------------ DEBUG ------------
				}else if(bCharBuffer[0]=='S' && bCharBuffer[1]=='B' && bCharBuffer[2]=='Q'){
					ZB_Send_Beacon();
// ------------ DEBUG ------------
				}else if(bCharBuffer[0]=='S' && bCharBuffer[1]=='T' && bCharBuffer[2]=='O'){
					g_iWSDumpStatus = 0;
				}else if(bCharBuffer[0]=='I' && bCharBuffer[1]=='N' && bCharBuffer[2]=='I'){
					if(bCharBuffer[3]==':' && isdigit(bCharBuffer[4]) && isdigit(bCharBuffer[5])){
						int chan = (bCharBuffer[4] - '0')*10 + bCharBuffer[5] - '0';
						if(chan>=11 && chan<=26){
						}
					}
					g_iWSDumpStatus = 1;
					WS_init();
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
						}
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

// MAC_MlmeDcfmIndType_e
const char * MAC_MlmeDcfmIndType2Str(uint8_t type)
{
	switch(type){
	case MAC_MLME_DCFM_SCAN:			return "MAC_MLME_DCFM_SCAN";
	case MAC_MLME_DCFM_GTS:				return "MAC_MLME_DCFM_GTS";
	case MAC_MLME_DCFM_ASSOCIATE:		return "MAC_MLME_DCFM_ASSOCIATE";
	case MAC_MLME_DCFM_DISASSOCIATE:	return "MAC_MLME_DCFM_DISASSOCIATE";
	case MAC_MLME_DCFM_POLL:			return "MAC_MLME_DCFM_POLL";
	case MAC_MLME_DCFM_RX_ENABLE:		return "MAC_MLME_DCFM_RX_ENABLE";
	case MAC_MLME_IND_ASSOCIATE:		return "MAC_MLME_IND_ASSOCIATE";
	case MAC_MLME_IND_DISASSOCIATE:		return "MAC_MLME_IND_DISASSOCIATE";
	case MAC_MLME_IND_SYNC_LOSS:		return "MAC_MLME_IND_SYNC_LOSS";
	case MAC_MLME_IND_GTS:				return "MAC_MLME_IND_GTS";
	case MAC_MLME_IND_BEACON_NOTIFY:	return "MAC_MLME_IND_BEACON_NOTIFY";
	case MAC_MLME_IND_COMM_STATUS:		return "MAC_MLME_IND_COMM_STATUS";
	case MAC_MLME_IND_ORPHAN:			return "MAC_MLME_IND_ORPHAN";
#ifdef TOF_ENABLED
	case MAC_MLME_DCFM_TOFPOLL:			return "MAC_MLME_DCFM_TOFPOLL";
	case MAC_MLME_DCFM_TOFPRIME:		return "MAC_MLME_DCFM_TOFPRIME";
	case MAC_MLME_DCFM_TOFDATAPOLL:		return "MAC_MLME_DCFM_TOFDATAPOLL";
	case MAC_MLME_DCFM_TOFDATA:			return "MAC_MLME_DCFM_TOFDATA";
	case MAC_MLME_IND_TOFPOLL:			return "MAC_MLME_IND_TOFPOLL";
	case MAC_MLME_IND_TOFPRIME:			return "MAC_MLME_IND_TOFPRIME";
	case MAC_MLME_IND_TOFDATAPOLL:		return "MAC_MLME_IND_TOFDATAPOLL";
	case MAC_MLME_IND_TOFDATA:			return "MAC_MLME_IND_TOFDATA";
#endif
#if defined(DEBUG) && defined(EMBEDDED)
	case MAC_MLME_IND_VS_DEBUG_INFO:	return "MAC_MLME_IND_VS_DEBUG_INFO";
	case MAC_MLME_IND_VS_DEBUG_WARN:	return "MAC_MLME_IND_VS_DEBUG_WARN";
	case MAC_MLME_IND_VS_DEBUG_ERROR:	return "MAC_MLME_IND_VS_DEBUG_ERROR";
	case MAC_MLME_IND_VS_DEBUG_FATAL:	return "MAC_MLME_IND_VS_DEBUG_FATAL";
#endif /* defined(DEBUG) && defined(EMBEDDED) */
	case MAC_MLME_INVALID:				return "MAC_MLME_INVALID";
	default:							return "Unknown MLME message type!";
	}
}

// MAC_McpsDcfmIndType_e
const char * MAC_McpsDcfmIndType2Str(uint8_t type)
{
	switch(type){
	case MAC_MCPS_DCFM_DATA:		return "MAC_MCPS_DCFM_DATA";
	case MAC_MCPS_DCFM_PURGE:		return "MAC_MCPS_DCFM_PURGE";
	case MAC_MCPS_IND_DATA:			return "MAC_MCPS_IND_DATA";
	default:						return "Unknown MCPS message type!";
	}
}

PRIVATE MAC_MlmeDcfmInd_s sMlmeBuffer;
PRIVATE MAC_DcfmIndHdr_s * prMlmeGetBuffer(void * pParam)
{
	vPrintf("prMlmeGetBuffer\n");
	return (MAC_DcfmIndHdr_s *)&sMlmeBuffer;
}
PRIVATE void prMlmeCallback(void * pParam, MAC_DcfmIndHdr_s * pBuf)
{
	vPrintf("prMlmeCallback\n");
	vPrintf("    Type:   %d (%s)\n", pBuf->u8Type, MAC_MlmeDcfmIndType2Str(pBuf->u8Type));
	vPrintf("    Length: %d\n", pBuf->u8ParamLength);
}

PRIVATE MAC_McpsDcfmInd_s sMcpsBuffer;
PRIVATE MAC_DcfmIndHdr_s * prMcpsGetBuffer(void * pParam)
{
	vPrintf("prMcpsGetBuffer\n");
	return (MAC_DcfmIndHdr_s *)&sMcpsBuffer;
}
PRIVATE void prMcpsCallback(void * pParam, MAC_DcfmIndHdr_s * pBuf)
{
	vPrintf("prMcpsCallback\n");
	vPrintf("    Type:   %d (%s)\n", pBuf->u8Type, MAC_McpsDcfmIndType2Str(pBuf->u8Type));
	vPrintf("    Length: %d\n", pBuf->u8ParamLength);
	if(pBuf->u8Type==MAC_MCPS_IND_DATA){
		MAC_McpsDcfmIndParam_u * pIndFrame = (MAC_McpsDcfmIndParam_u*)pBuf;
		MAC_RxFrameData_s * pDataFrame = &pIndFrame->sIndData.sFrame;
		vPrintf("    Payload Length: %d\n", pDataFrame->u8SduLength);
		int cnt;
//		for(cnt=0;cnt<pDataFrame->u8SduLength;cnt++){
		for(cnt=0;cnt<MAC_MAX_DATA_PAYLOAD_LEN;cnt++){
			vPrintf("%02x ", pDataFrame->au8Sdu[cnt]);
		}
		vPrintf("\n");
		vPrintf("    Timestamp:      %d\n", pDataFrame->u32Timestamp);
	}
}
/*
void prMlmeCallback(void)
{
	vPrintf("prMlmeCallback\n");
}

void prMcpsCallback(void)
{
	vPrintf("prMcpsCallback\n");
}
*/
void prHwCallback(void)
{
	vPrintf("prHwCallback\n");
}

#define exPutC(c)	vUartWrite(UART_TO_PC, c)
PUBLIC void vException_BusError(void)
{
//	vPrintf("\nvException_BusError\n");
	exPutC('\n'); exPutC('0'); exPutC('v'); exPutC('E'); exPutC('x'); exPutC('c'); exPutC('e'); exPutC('p'); exPutC('t'); exPutC('i'); exPutC('o'); exPutC('n'); exPutC('_');
	exPutC('B');  exPutC('u'); exPutC('s'); exPutC('E'); exPutC('r'); exPutC('r'); exPutC('o'); exPutC('r'); exPutC('\n');
}

PUBLIC void vException_UnalignedAccess(void)
{
//	vPrintf("\nvException_UnalignedAccess\n");
	exPutC('\n'); exPutC('1'); exPutC('v'); exPutC('E'); exPutC('x'); exPutC('c'); exPutC('e'); exPutC('p'); exPutC('t'); exPutC('i'); exPutC('o'); exPutC('n'); exPutC('_');
	exPutC('U');  exPutC('n'); exPutC('a'); exPutC('l'); exPutC('i'); exPutC('g'); exPutC('n'); exPutC('e'); exPutC('d'); exPutC('A'); exPutC('c'); exPutC('c'); exPutC('e'); exPutC('s'); exPutC('s'); exPutC('\n');
}

PUBLIC void vException_IllegalInstruction(void)
{
//	vPrintf("\nvException_IllegalInstruction\n");
	exPutC('\n'); exPutC('2'); exPutC('v'); exPutC('E'); exPutC('x'); exPutC('c'); exPutC('e'); exPutC('p'); exPutC('t'); exPutC('i'); exPutC('o'); exPutC('n'); exPutC('_');
	exPutC('I');  exPutC('l'); exPutC('l'); exPutC('e'); exPutC('g'); exPutC('a'); exPutC('l'); exPutC('I'); exPutC('n'); exPutC('s'); exPutC('t'); exPutC('r'); exPutC('u'); exPutC('c'); exPutC('t'); exPutC('i'); exPutC('o'); exPutC('n'); exPutC('\n');
}

PUBLIC void vException_SysCall(void)
{
//	vPrintf("\nvException_SysCall\n");
	exPutC('\n'); exPutC('3'); exPutC('v'); exPutC('E'); exPutC('x'); exPutC('c'); exPutC('e'); exPutC('p'); exPutC('t'); exPutC('i'); exPutC('o'); exPutC('n'); exPutC('_');
	exPutC('S');  exPutC('y'); exPutC('s'); exPutC('C'); exPutC('a'); exPutC('l'); exPutC('l'); exPutC('\n');
}

PUBLIC void vException_Trap(void)
{
//	vPrintf("\nvException_Trap\n");
	exPutC('\n'); exPutC('4'); exPutC('v'); exPutC('E'); exPutC('x'); exPutC('c'); exPutC('e'); exPutC('p'); exPutC('t'); exPutC('i'); exPutC('o'); exPutC('n'); exPutC('_');
	exPutC('T');  exPutC('r'); exPutC('a'); exPutC('p'); exPutC('\n');
}

PUBLIC void vException_StackOverflow(void)
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

PRIVATE void vPutC1(uint8 u8Data)
{
	vUartWrite(UART_FOR_DEBUG, u8Data);
}

PRIVATE char acGetC(void)
{
	return(u8UartRead(UART_TO_PC));
}


PRIVATE void TickTimer_Cb(uint32_t u32Device, uint32_t u32ItemBitmap)
{
	g_u32Seconds++;
}

PRIVATE void SysCtrl_Cb(uint32_t u32Device, uint32_t u32ItemBitmap)
{
	if(u32ItemBitmap & E_AHI_DIO7_INT){
		g_au32FallingTimes[g_u32FallingTimesWPtr++] = u32AHI_TickTimerRead();
		if(g_u32FallingTimesWPtr==ARRAY_SIZE(g_au32FallingTimes)){g_u32FallingTimesWPtr=0;}
	}
}


PRIVATE void WS_init(void)
{
	// C3 : LINKTYPE_IEEE802_15_4	    195	DLT_IEEE802_15_4	    IEEE 802.15.4 wireless Personal Area Network, with each packet having the FCS at the end of the frame.
	// E6 : LINKTYPE_IEEE802_15_4_NOFCS	230	DLT_IEEE802_15_4_NOFCS	IEEE 802.15.4 wireless Personal Area Network, without the FCS at the end of the frame.
//	pcap_hdr_t pcap_hdr = {.magic_number=0xA1B2C3D4 , .version_major=2, .version_minor=4, .thiszone=0, .sigfigs=0, .snaplen=65535, .network=0xC3};
//	pcap_hdr_t pcap_hdr = {.magic_number=0xD4C3B2A1 , .version_major=0x0200, .version_minor=0x0400, .thiszone=0, .sigfigs=0, .snaplen=0xFFFF0000, .network=0xC3000000};
//	pcap_hdr_t pcap_hdr = {.magic_number=0xD4C3B2A1 , .version_major=0x0200, .version_minor=0x0400, .thiszone=0, .sigfigs=0, .snaplen=0xFFFF0000, .network=0x01000000};
	pcap_hdr_t pcap_hdr = {.magic_number=0xD4C3B2A1 , .version_major=0x0200, .version_minor=0x0400, .thiszone=0, .sigfigs=0, .snaplen=0xFFFF0000, .network=0xE6000000};

	int ws_snd_cnt;
	for(ws_snd_cnt=0;ws_snd_cnt<sizeof(pcap_hdr);ws_snd_cnt++){
		vPutC(((uint8_t*)&pcap_hdr)[ws_snd_cnt]);
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
	pcap_rec_hdr.ts_sec  = swap32(u32Seconds);
	pcap_rec_hdr.ts_usec = swap32(u32Fraction/16);			// 16 MHz
	dataFrame[lenval++] = 0x07;					// Unknown packet type
	dataFrame[lenval++] = 0x00;					// Unknown packet type
	dataFrame[lenval++] = 0x00;					// Sequence number
	dataFrame[lenval++] = u8Channel;
	float fFreq = afFrequencies[u8Channel - 11];
	dataFrame[lenval++] = ((uint8_t*)&fFreq)[0];
	dataFrame[lenval++] = ((uint8_t*)&fFreq)[1];
	dataFrame[lenval++] = ((uint8_t*)&fFreq)[2];
	dataFrame[lenval++] = ((uint8_t*)&fFreq)[3];
	pcap_rec_hdr.incl_len = swap32(lenval);
	pcap_rec_hdr.orig_len = swap32(lenval);
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
	pcap_rec_hdr.ts_sec  = swap32(u32Seconds);
	pcap_rec_hdr.ts_usec = swap32(u32Fraction/16);			// 16 MHz
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
	pcap_rec_hdr.incl_len = swap32(lenval);
	pcap_rec_hdr.orig_len = swap32(lenval);
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

#ifdef DEBUG_PROTOCOL
PRIVATE void ClearText_Dump_Packet(tsJPT_PT_Packet * psPacket)
{
	uint32_t u32Seconds;
	uint32_t u32Fraction;
	uint32_t u32Micros;
	do{
		u32Seconds  = g_u32Seconds;
		u32Fraction = u32AHI_TickTimerRead();
	}while(u32Seconds!=g_u32Seconds);
	u32Micros = u32Fraction/16;

	int cnt;
	vPrintf("%4d.%06d:New packet\n", u32Seconds, u32Micros);
	vPrintf("\t            Packet good [%3d] : %s\n", psPacket->bPacketGood, psPacket->bPacketGood?"yes":"no");
	vPrintf("\t              u16FrameControl : %04x\n", psPacket->u16FrameControl);
	vPrintf("\t        u16SourceShortAddress : %04x\n", psPacket->u16SourceShortAddress);
	vPrintf("\t   u16DestinationShortAddress : %04x\n", psPacket->u16DestinationShortAddress);
	vPrintf("\t     u64SourceExtendedAddress : ");
	for(cnt=0;cnt<8;cnt++){
		vPrintf("%02x ", (psPacket->u64SourceExtendedAddress >> (8*cnt)) & 0xFF);
	}
	vPrintf("\n");
	vPrintf("\tu64DestinationExtendedAddress : ");
	for(cnt=0;cnt<8;cnt++){
		vPrintf("%02x ", (psPacket->u64DestinationExtendedAddress >> (8*cnt)) & 0xFF);
	}
	vPrintf("\n");
	vPrintf("\t               u16SourcePanID : %04x\n", psPacket->u16SourcePanID);
	vPrintf("\t          u16DestinationPanID : %04x\n", psPacket->u16DestinationPanID);
	vPrintf("\t              u8PayloadLength : %4d\n", psPacket->u8PayloadLength);
	vPrintf("\t");
	for(cnt=0;cnt<psPacket->u8PayloadLength;cnt++){
		vPrintf("%02x ", psPacket->u8Payload[cnt]);
	}
	vPrintf("\n");
	vPrintf("\t              u16FrameControl : %04x\n", psPacket->u16FrameControl);
	vPrintf("\t                     u8Energy :   %02x\n", psPacket->u8Energy);
	vPrintf("\t                        u8SQI :   %02x\n", psPacket->u8SQI);
	vPrintf("\t             u8SequenceNumber :   %02x\n", psPacket->u8SequenceNumber);
	vPrintf("\t                        u8LQI :   %02x\n", psPacket->u8LQI);
	vPrintf("\t                       u8RSSI :   %02x\n", psPacket->u8RSSI);
}
#endif

PRIVATE void WS_Dump_Packet(tsJPT_PT_Packet * psPacket)
{
	int cnt;
	bool_t bSrcShortAddr = FALSE;
	bool_t bSrcExtAddr = FALSE;
	bool_t bDstShortAddr = FALSE;
	bool_t bDstExtAddr = FALSE;
	bool_t bIntraPan = FALSE;

	uint32_t u32Seconds;
	uint32_t u32Fraction;
	uint32_t u32Micros;
	do{
		u32Seconds  = g_u32Seconds;
		u32Fraction = u32AHI_TickTimerRead();
	}while(u32Seconds!=g_u32Seconds);
	u32Micros = u32Fraction/16;

/* ========================== Analyze Received Packet ==============================================*/
	if (psPacket->bPacketGood) {

//	IEEE 802.15.4 specification Section 5.2.1.1
// Bits
//	 0- 2: Frame type
//			000	Beacon
//			001 Data
//			010 Acknowledge
//			011 MAC command
//			1xx Reserved
//			111 Used to communicate with Wireshark lua script
//	    3: Security Enabled
//	    4: Frame Pending
//	    5: AR (Acknowledge Request)
//	    6: PAN ID Compression
//	 7- 9: Reserved
//	10-11: Dest. Addressing Mode
//
//	12-13: Frame version
//	14-15: Source Addressing Mode

		if((psPacket->u16FrameControl >> 6) & 1){ bIntraPan = TRUE; }

		/* Source addressing mode */
		switch((psPacket->u16FrameControl >> 14) & 3){

		/* PAN id and address field not present */
		case 0:
			bSrcShortAddr = FALSE;
			bSrcExtAddr = FALSE;
			break;

		/* Reserved */
		case 1:
// TODO: should we signal an error?
			// Je mets short mais je ne sais pas ce que cela doit etre
			bSrcShortAddr = FALSE;
			bSrcExtAddr = FALSE;
			break;

		/* Address field contains a 16 bit short address */
		case 2:
			// vPrintf("SSAD=%04x ",psPacket->u16SourceShortAddress);
			bSrcShortAddr = TRUE;
			bSrcExtAddr = FALSE;
			break;

		/* Address field contains a 64 bit extended address */
		case 3:
			// vPrintf("SEAD=%016lx ", psPacket->u64SourceExtendedAddress);
			bSrcShortAddr = FALSE;
			bSrcExtAddr = TRUE;
			break;
		}

		/* Destination addressing mode */
		switch((psPacket->u16FrameControl & 3 << 10) >> 10){

		/* PAN id and address field not present */
		case 0:
			bDstShortAddr = FALSE;
			bDstExtAddr = FALSE;
			break;

			/* Reserved */
		case 1:
			// Je mets short mais je ne sais pas ce que cela doit etre
			bDstShortAddr = FALSE;
			bDstExtAddr = FALSE;
			break;

			/* Address field contains a 16 bit short address */
		case 2:
			// vPrintf("DSAD=%04x ",psPacket->u16DestinationShortAddress);
			bDstShortAddr = TRUE;
			bDstExtAddr = FALSE;
			break;

			/* Address field contains a 64 bit extended address */
		case 3:
			// vPrintf("DEAD=%016lx ", psPacket->u64DestinationExtendedAddress);
			bDstShortAddr = FALSE;
			bDstExtAddr = TRUE;
			break;

		}
	}
	
	if ( (psPacket->bPacketGood) ) {
		vPutC(u32Seconds >>  0); vPutC(u32Seconds >>  8); vPutC(u32Seconds >> 16); vPutC(u32Seconds >> 24);
		vPutC(u32Micros  >>  0); vPutC(u32Micros  >>  8); vPutC(u32Micros  >> 16); vPutC(u32Micros  >> 24);
		
		int frame_len = 0;
		frame_len += 2;			// Control field
		frame_len += 1;			// Sequence numbe
		if(bDstShortAddr || bDstExtAddr){			frame_len += 2;	}	// Destination PAN (Std 802.15.4-2011 p59)
		if(bDstShortAddr){					frame_len += 2;	}
		if(bDstExtAddr){					frame_len += 8;	}
		if((bIntraPan==0) && (bSrcExtAddr || bSrcShortAddr)){	frame_len += 2;	}
		if(bSrcShortAddr){					frame_len += 2;	}
		if(bSrcExtAddr){					frame_len += 8;	}
		frame_len += psPacket->u8PayloadLength;

		vPutC( frame_len ); vPutC(0); vPutC(0); vPutC(0);
		vPutC( frame_len ); vPutC(0); vPutC(0); vPutC(0);
// MAC
	// FrameControl
		vPutC(psPacket->u16FrameControl&0xFF); vPutC(psPacket->u16FrameControl>>8);
	// SequenceNumber
		vPutC(psPacket->u8SequenceNumber);
	// Destination Pan ID
		if(bDstShortAddr || bDstExtAddr){
			vPutC(psPacket->u16DestinationPanID&0xFF); vPutC(psPacket->u16DestinationPanID>>8);
		}
	// Destination Address
		if(bDstShortAddr){
		// Destination Short Address
			vPutC(psPacket->u16DestinationShortAddress&0xFF); vPutC(psPacket->u16DestinationShortAddress>>8);
		}
		if ( bDstExtAddr ) {
		// Destination Long Address
			uint8 *data = (uint8 *)&(psPacket->u64DestinationExtendedAddress);
			vPutC(data[7]); vPutC(data[6]); vPutC(data[5]); vPutC(data[4]); vPutC(data[3]); vPutC(data[2]); vPutC(data[1]); vPutC(data[0]);
		}
	// Source Pan ID
		if ( (bIntraPan==0) && (bSrcExtAddr || bSrcShortAddr) ) {
			vPutC(psPacket->u16SourcePanID&0xFF); vPutC(psPacket->u16SourcePanID>>8);
		}
	// Source Address
		if(bSrcShortAddr){
		// Source Short Address
			vPutC(psPacket->u16SourceShortAddress&0xFF); vPutC(psPacket->u16SourceShortAddress>>8);
		}
		if ( bSrcExtAddr ) {
		// Source Long Address
			uint8 *data = (uint8 *)&(psPacket->u64SourceExtendedAddress);
			vPutC(data[7]); vPutC(data[6]); vPutC(data[5]); vPutC(data[4]); vPutC(data[3]); vPutC(data[2]); vPutC(data[1]); vPutC(data[0]);
		}
	// Payload
		for(cnt = 0; cnt < psPacket->u8PayloadLength; cnt++){
			vPutC( psPacket->u8Payload[cnt] );
		}
	}else {
		// Will need to inform Wireshark that a packet has been received but not forwarded.
	}
}

uint32_t g_u32IEEESeqNumber = 36;

// This is necessary in order to be able to receive data after having sent a frame
PRIVATE void ZB_JumpChannel(void)
{
}

PRIVATE void ZB_Send_Beacon_Request(void)
{
	tsJPT_PT_Packet sSendPacket;
	memset(&sSendPacket, 0xAA, sizeof(sSendPacket));
	int len = 0;
// IEEE 802.15.4
	sSendPacket.u16FrameControl = (2 << 10) | 3;		// Destination address 16 bits, command
	sSendPacket.u8SequenceNumber = g_u32IEEESeqNumber++;
	sSendPacket.u16DestinationPanID = 0xFFFF;
	sSendPacket.u16DestinationShortAddress = 0xFFFF;
	sSendPacket.u8Payload[len++] = 0x07;			// Command Identifier : Beacon Request
#if 0
	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl & 0xFF);				// Little Endian
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl >> 8);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u8SequenceNumber);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16DestinationPanID & 0xFF);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16DestinationPanID >> 8);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16DestinationShortAddress & 0xFF);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16DestinationShortAddress >> 8);
	u16fcs = crc_ccitt(u16fcs, &sSendPacket.u8Payload[0], len);

	sSendPacket.u8Payload[len++] = u16fcs & 0xFF;		// FCS
	sSendPacket.u8Payload[len++] = u16fcs >> 8;		// FCS
#endif
	sSendPacket.u8PayloadLength = len;

	sSendPacket.bPacketGood = 1;
	WS_Dump_Packet(&sSendPacket);

//!!!	vJPT_PacketTx(g_u8Channel, &sSendPacket);
	ZB_JumpChannel();
}

#ifdef DO_COORD_JOB
void DoCoordJob(tsJPT_PT_Packet * psPacket)
{
//	tsJPT_PT_Packet sSendPacket;
	if (psPacket->bPacketGood) {
		/* look at frame type */
		if((psPacket->u16FrameControl & 7) == 3){				// MAC Request
			if(psPacket->u8PayloadLength>0 && psPacket->u8Payload[0]==7){	// Beacon request
				ZB_Send_Beacon();
/*				int len = 0;
// IEEE 802.15.4
				sSendPacket.u16FrameControl = 2 << 14;			// Source address 16 bits
				sSendPacket.u8SequenceNumber = g_u32IEEESeqNumber++;
				sSendPacket.u16SourcePanID = 0x1515;
				sSendPacket.u16SourceShortAddress = 0x3615;
				sSendPacket.u8Payload[len++] = 0xFF;			// 16 bits PAN Coordinator, Association permit
				sSendPacket.u8Payload[len++] = 0xCF;			//
				sSendPacket.u8Payload[len++] = 0x00;			// GTS
				sSendPacket.u8Payload[len++] = 0x00;			// Pending addresses
// ZigBee
				sSendPacket.u8Payload[len++] = 0x00;			// Protocol ID
				sSendPacket.u8Payload[len++] = 0x22;			// Beacon
				sSendPacket.u8Payload[len++] = 0x84;			// 
				sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID - Should be a random number, but different from other neighbors
				sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
				sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
				sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
				sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
				sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
				sSendPacket.u8Payload[len++] = 0x15;			// Ext PAN ID
				sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
				sSendPacket.u8Payload[len++] = 0xFF;			// Tx Offset
				sSendPacket.u8Payload[len++] = 0xFF;			// Tx Offset
				sSendPacket.u8Payload[len++] = 0xFF;			// Tx Offset
				sSendPacket.u8Payload[len++] = 0x00;			// Update ID

#if 0
				uint16_t u16fcs = 0;
				u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl & 0xFF);				// Little Endian
				u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl >> 8);
				u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u8SequenceNumber);
				u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16SourcePanID & 0xFF);
				u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16SourcePanID >> 8);
				u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16SourceShortAddress & 0xFF);
				u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16SourceShortAddress >> 8);
				u16fcs = crc_ccitt(u16fcs, &sSendPacket.u8Payload[0], len);

				sSendPacket.u8Payload[len++] = u16fcs & 0xFF;		// FCS
				sSendPacket.u8Payload[len++] = u16fcs >> 8;		// FCS
#endif
				sSendPacket.u8PayloadLength = len;

				sSendPacket.bPacketGood = 1;
				WS_Dump_Packet(&sSendPacket);*/
			}
		}
	}
}

PRIVATE void ZB_Send_Beacon(void)
{
	tsJPT_PT_Packet sSendPacket;
	int len = 0;
// IEEE 802.15.4
	sSendPacket.u16FrameControl = (2 << 14);		// Source address 16 bits
	sSendPacket.u8SequenceNumber = g_u32IEEESeqNumber++;
	sSendPacket.u16SourcePanID = 0x1516;
	sSendPacket.u16SourceShortAddress = 0x3615;
	sSendPacket.u8Payload[len++] = 0xFF;			// 16 bits PAN Coordinator, Association permit
	sSendPacket.u8Payload[len++] = 0xCF;			//
	sSendPacket.u8Payload[len++] = 0x00;			// GTS
	sSendPacket.u8Payload[len++] = 0x00;			// Pending addresses
// ZigBee
	sSendPacket.u8Payload[len++] = 0x00;			// Protocol ID
	sSendPacket.u8Payload[len++] = 0x22;			// Beacon
	sSendPacket.u8Payload[len++] = 0x84;			// 
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID - Should be a random number, but different from other neighbors
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0x15;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0xFF;			// Tx Offset
	sSendPacket.u8Payload[len++] = 0xFF;			// Tx Offset
	sSendPacket.u8Payload[len++] = 0xFF;			// Tx Offset
	sSendPacket.u8Payload[len++] = 0x00;			// Update ID
#if 0
	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl & 0xFF);				// Little Endian
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl >> 8);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u8SequenceNumber);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16SourcePanID & 0xFF);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16SourcePanID >> 8);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16SourceShortAddress & 0xFF);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16SourceShortAddress >> 8);
	u16fcs = crc_ccitt(u16fcs, &sSendPacket.u8Payload[0], len);

	sSendPacket.u8Payload[len++] = u16fcs & 0xFF;		// FCS
	sSendPacket.u8Payload[len++] = u16fcs >> 8;		// FCS
#endif
	sSendPacket.u8PayloadLength = len;

	sSendPacket.bPacketGood = 1;
	WS_Dump_Packet(&sSendPacket);
//!!!	vJPT_PacketTx(g_u8Channel, &sSendPacket);
	ZB_JumpChannel();
}

PRIVATE void ZB_Send_Ack(uint8_t seqnum)
{
	tsJPT_PT_Packet sSendPacket;
	int len = 0;
// IEEE 802.15.4
	sSendPacket.u16FrameControl = 2;			// Ack
	sSendPacket.u8SequenceNumber = seqnum;
#if 0
	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl & 0xFF);				// Little Endian
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl >> 8);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u8SequenceNumber);

	sSendPacket.u8Payload[len++] = u16fcs & 0xFF;		// FCS
	sSendPacket.u8Payload[len++] = u16fcs >> 8;		// FCS
#endif
	sSendPacket.u8PayloadLength = len;

	sSendPacket.bPacketGood = 1;
	WS_Dump_Packet(&sSendPacket);
///!!!	vJPT_PacketTx(g_u8Channel, &sSendPacket);
	ZB_JumpChannel();
}

PRIVATE void ZB_Send_Asso_Resp(void)
{
	tsJPT_PT_Packet sSendPacket;
	int len = 0;
// IEEE 802.15.4
	sSendPacket.u16FrameControl = (3 << 14) | 0xcc63;	// Source address 64 bits
	sSendPacket.u8SequenceNumber = g_u32IEEESeqNumber++;
	sSendPacket.u16SourcePanID = 0x1516;
	sSendPacket.u64SourceExtendedAddress = 0x123456789ABCDEF0;
	sSendPacket.u64DestinationExtendedAddress = 0xAA55DEADBEEF1234;
	sSendPacket.u8Payload[len++] = 0x02;			// Command identifier
	sSendPacket.u8Payload[len++] = 0xCF;			//
	sSendPacket.u8Payload[len++] = 0x00;			// GTS
	sSendPacket.u8Payload[len++] = 0x00;			// Pending addresses
// ZigBee
	sSendPacket.u8Payload[len++] = 0x00;			// Protocol ID
	sSendPacket.u8Payload[len++] = 0x22;			// Beacon
	sSendPacket.u8Payload[len++] = 0x84;			// 
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID - Should be a random number, but different from other neighbors
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0x15;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0x00;			// Ext PAN ID
	sSendPacket.u8Payload[len++] = 0xFF;			// Tx Offset
	sSendPacket.u8Payload[len++] = 0xFF;			// Tx Offset
	sSendPacket.u8Payload[len++] = 0xFF;			// Tx Offset
	sSendPacket.u8Payload[len++] = 0x00;			// Update ID
#if 0
	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl & 0xFF);				// Little Endian
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl >> 8);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u8SequenceNumber);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16SourcePanID & 0xFF);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16SourcePanID >> 8);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16SourceShortAddress & 0xFF);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16SourceShortAddress >> 8);
	u16fcs = crc_ccitt(u16fcs, &sSendPacket.u8Payload[0], len);

	sSendPacket.u8Payload[len++] = u16fcs & 0xFF;		// FCS
	sSendPacket.u8Payload[len++] = u16fcs >> 8;		// FCS
#endif
	sSendPacket.u8PayloadLength = len;

	sSendPacket.bPacketGood = 1;
	WS_Dump_Packet(&sSendPacket);
///!!!	vJPT_PacketTx(g_u8Channel, &sSendPacket);
	ZB_JumpChannel();
}
#endif	//def DO_COORD_JOB

