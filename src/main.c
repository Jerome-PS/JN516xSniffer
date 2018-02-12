#include <jendefs.h>
#include <AppHardwareApi.h>
#include <JPT.h>
#include "Printf.h"
#include "UartBuffered.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

//#define XIAOMI_SMART_BUTTON

#define UART_TO_PC              E_AHI_UART_0        /* Uart to PC           */

#define BAUD_RATE               E_AHI_UART_RATE_38400 /* Baud rate to use   */

#define STARTUP_CHANNEL			20

#ifdef XIAOMI_SMART_BUTTON
#define LED_PIN_BIT				(1 << 11)
#define MAIN_PIN_BIT			(1 << 16)
#define PAIR_PIN_BIT			(1 <<  0)
#endif //def XIAOMI_SMART_BUTTON

#define READ_REG32(A)     *(volatile uint32 *)(A)

#define swap32(x)	__builtin_bswap32((x))
//#define swap16(x)	__builtin_bswap16((x))
#define swap16(x)	((((x) & 0xFF) << 8) | (((x) >> 8) & 0xFF))

#define isdigit(x)	(((x)>='0') && ((x)<='9'))

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

uint8 au8UartTxBuffer[100];
uint8 au8UartRxBuffer[100];

uint32 u32RadioMode           = E_JPT_MODE_LOPOWER;
uint32 u32ModuleRadioMode     = E_JPT_MODE_LOPOWER;

uint8 u8TxPowerAdj	  = 0;
uint8 u8Attenuator3dB = 0;

volatile uint32_t g_u32Seconds = 0;
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
PRIVATE char acGetC(void);
PRIVATE void TickTimer_Cb(uint32_t u32Device, uint32_t u32ItemBitmap);
PRIVATE void WS_init(void);
PRIVATE void WS_Send_Chan_Num(uint8_t u8Channel);
PRIVATE void WS_Send_Syntax_Error(const char * pBuffer);
PRIVATE void WS_Send_Test_Packet(void);
PRIVATE void WS_Dump_Packet(tsJPT_PT_Packet * psPacket);

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void AppColdStart(void)
{
	uint32 u32JPT_Ver = 0;(void)u32JPT_Ver;
	uint32 u32JPT_RadioModes = 0;(void)u32JPT_RadioModes;
	uint32 u32Chip_Id = 0;(void)u32Chip_Id;
#if (defined JENNIC_CHIP_JN5169)
	uint32 u32RadioParamVersion;
#endif
	uint8 u8Channel = 0;
	uint8 u8Channelsave = 0;
	tsJPT_PT_Packet sPacket;
	char bCharBuffer[64] = "";
	int iCharBufferPtr = 0;

	/* Turn off debugger */
	*(volatile uint32 *)0x020000a0 = 0;

	vAHI_WatchdogStop();

	u32JPT_Ver = u32JPT_Init();                 /* initialise production test API */
#if (defined JENNIC_CHIP_JN5169)
	vJPT_GetRadioConfig(&u32RadioParamVersion);
#endif

	u32AHI_Init();                              /* initialise hardware API */

	volatile int n;
	for(n=0;n<100000;n++);      // wait for JN516X to move onto 32MHz Crystal

/* set up the tick timer, we'll use it for timestamps */
	vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_DISABLE);
	vAHI_TickTimerInit((void*)TickTimer_Cb);
	vAHI_TickTimerWrite(0x00000000);					// Starting  count
	vAHI_TickTimerInterval(16000000);					// Reference count 1Hz
	vAHI_TickTimerIntEnable(TRUE);
	vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_RESTART);

	vAHI_UartSetRTSCTS(UART_TO_PC, FALSE);
	vUartInit(UART_TO_PC, BAUD_RATE, au8UartTxBuffer, sizeof(au8UartTxBuffer), au8UartRxBuffer, sizeof(au8UartRxBuffer));/* uart for user interface */

#ifdef HEARTBEAT_LED
	vAHI_DioSetDirection(0x00000000, LED_PIN_BIT);		// Set DIO11 as output (LED)
#endif	//def HEARTBEAT_LED

#ifdef XIAOMI_SMART_BUTTON
	vAHI_DioSetPullup(~MAIN_PIN_BIT, MAIN_PIN_BIT);  /* turn all pullups on except for DIO16 which is big button input      */
#else
	vAHI_DioSetPullup(0xffffffff, 0x00000000);  /* turn all pullups on      */
#endif

	/* read Chip_ID register */
	u32Chip_Id= READ_REG32(0x020000fc);

	u32RadioMode = E_JPT_MODE_LOPOWER;
	u32ModuleRadioMode = E_JPT_MODE_LOPOWER;
	u8TxPowerAdj = 0;
	u8Attenuator3dB = 0;
#ifdef RXPOWERADJUST_SUPPORT
	vJPT_SetMaxInputLevel(E_MAX_INP_LEVEL_10dB);
#endif	//def RXPOWERADJUST_SUPPORT

	/* enable protocol */
	bJPT_RadioInit(u32RadioMode);

	/* force channel change in bJPT_PacketRx */
	u8Channel = u8JPT_RadioGetChannel();
	if (u8Channel != STARTUP_CHANNEL){
		bJPT_PacketRx(STARTUP_CHANNEL, &sPacket);
	}else{
		bJPT_PacketRx(11, &sPacket);
		bJPT_PacketRx(STARTUP_CHANNEL, &sPacket);
	}
	u8Channel = u8JPT_RadioGetChannel();

	bJPT_RadioSetChannel(u8Channel);

	while(1){
		if ((u8Channel != u8Channelsave) && g_iWSDumpStatus > 0){
			WS_Send_Chan_Num(u8Channel);
			u8Channelsave = u8Channel;
		}
		if(bJPT_PacketRx(u8Channel, &sPacket)){
			if(g_iWSDumpStatus > 0){
				WS_Dump_Packet(&sPacket);
			}
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
					   	bJPT_PacketRx(chan, &sPacket);
						u8Channel = u8JPT_RadioGetChannel();
						bJPT_RadioSetChannel(u8Channel);
					}
					u8Channelsave = 0;
				}else if(bCharBuffer[0]=='S' && bCharBuffer[1]=='T' && bCharBuffer[2]=='O'){
					g_iWSDumpStatus = 0;
				}else if(bCharBuffer[0]=='S' && bCharBuffer[1]=='T' && bCharBuffer[2]=='A'){
					g_iWSDumpStatus = 1;
					WS_init();
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


PRIVATE void TickTimer_Cb(uint32_t u32Device, uint32_t u32ItemBitmap)
{
	g_u32Seconds++;
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
	pcap_rec_hdr.ts_usec = swap32(u32Fraction*10/16);		// 16 MHz
	dataFrame[lenval++] = 0x07;								// Unknown packet type
	dataFrame[lenval++] = 0x00;								// Unknown packet type
	dataFrame[lenval++] = 0x00;								// Sequence number
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
	pcap_rec_hdr.ts_usec = swap32(u32Fraction*10/16);		// 16 MHz
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

PRIVATE void WS_Dump_Packet(tsJPT_PT_Packet * psPacket)
{
	int FCS			= 0;
	int FCS_Length 	= 0;

	int cnt;
	bool_t bSrcShortAddr = FALSE;
	bool_t bSrcExtAddr = FALSE;
	bool_t bDstShortAddr = FALSE;
	bool_t bDstExtAddr = FALSE;
	bool_t bIntraPan = FALSE;

	if ( FCS ){ FCS_Length = 2; }else{ FCS_Length = 0; }

	uint32_t u32Seconds;
	uint32_t u32Fraction;
	uint32_t u32Micros;
	do{
		u32Seconds  = g_u32Seconds;
		u32Fraction = u32AHI_TickTimerRead();
	}while(u32Seconds!=g_u32Seconds);
	u32Micros = u32Fraction*10/16;

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

/* look at frame type */
	switch(psPacket->u16FrameControl & 7){
/* MAC Beacon Reply -----------------------------------------------------------------------------------------  */
	case 0:
		if ( psPacket->bPacketGood ) {
			vPutC(((uint8_t*)&u32Seconds)[3]);vPutC(((uint8_t*)&u32Seconds)[2]);vPutC(((uint8_t*)&u32Seconds)[1]);vPutC(((uint8_t*)&u32Seconds)[0]);
			vPutC(((uint8_t*)&u32Micros)[3]); vPutC(((uint8_t*)&u32Micros)[2]); vPutC(((uint8_t*)&u32Micros)[1]); vPutC(((uint8_t*)&u32Micros)[0]);
			vPutC( psPacket->u8PayloadLength + 7 + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);
			vPutC( psPacket->u8PayloadLength + 7 + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);
	// MAC
		// FrameControl
			vPutC(psPacket->u16FrameControl&0xFF); vPutC(psPacket->u16FrameControl>>8);
		// SequenceNumber
			vPutC(psPacket->u8SequenceNumber);
		// Source Pan ID
			vPutC(psPacket->u16SourcePanID&0xFF); vPutC(psPacket->u16SourcePanID>>8);
		// Source Short Address
			vPutC(psPacket->u16SourceShortAddress&0xFF); vPutC(psPacket->u16SourceShortAddress>>8);
		// Payload
			for(cnt = 0; cnt < psPacket->u8PayloadLength; cnt++){
				vPutC( psPacket->u8Payload[cnt] );
			}
		// FCS
			if (FCS) { vPutC(0xAA); vPutC(0xAA); }
		}else {
			// Will need to inform Wireshark that a packet has been received but not forwarded.
		}
		break;
/* MAC Data -------------------------------------------------------------------------------------------------- */
	case 1:
		if ( (psPacket->bPacketGood) ) {
			vPutC(((uint8_t*)&u32Seconds)[3]);vPutC(((uint8_t*)&u32Seconds)[2]);vPutC(((uint8_t*)&u32Seconds)[1]);vPutC(((uint8_t*)&u32Seconds)[0]);
			vPutC(((uint8_t*)&u32Micros)[3]); vPutC(((uint8_t*)&u32Micros)[2]); vPutC(((uint8_t*)&u32Micros)[1]); vPutC(((uint8_t*)&u32Micros)[0]);
			
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
		// FCS
			if (FCS) { vPutC(0xAA); vPutC(0xAA); }
		}else {
			// Will need to inform Wireshark that a packet has been received but not forwarded.
		}
		break;
/* Ack  ------------------------------------------------------------------------------------------------------ */
	case 2:
		if ( psPacket->bPacketGood ) {
			vPutC(((uint8_t*)&u32Seconds)[3]);vPutC(((uint8_t*)&u32Seconds)[2]);vPutC(((uint8_t*)&u32Seconds)[1]);vPutC(((uint8_t*)&u32Seconds)[0]);
			vPutC(((uint8_t*)&u32Micros)[3]); vPutC(((uint8_t*)&u32Micros)[2]); vPutC(((uint8_t*)&u32Micros)[1]); vPutC(((uint8_t*)&u32Micros)[0]);
			vPutC( psPacket->u8PayloadLength + 3 + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);
			vPutC( psPacket->u8PayloadLength + 3 + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);
	// MAC
		// FrameControl
			vPutC(psPacket->u16FrameControl&0xFF); vPutC(psPacket->u16FrameControl>>8);
		// SequenceNumber
			vPutC(psPacket->u8SequenceNumber);
		// FCS
			if (FCS) { vPutC(0xAA); vPutC(0xAA); }
		}else {
			// Will need to inform Wireshark that a packet has been received but not forwarded.
		}
		break;
/* MAC Command  ---------------------------------------------------------------------------------------------- */
	/* Beacon Request */
	/* Association Request */
	/* Association Response */
	case 3:
		if ( psPacket->bPacketGood ) {
			vPutC(((uint8_t*)&u32Seconds)[3]);vPutC(((uint8_t*)&u32Seconds)[2]);vPutC(((uint8_t*)&u32Seconds)[1]);vPutC(((uint8_t*)&u32Seconds)[0]);
			vPutC(((uint8_t*)&u32Micros)[3]); vPutC(((uint8_t*)&u32Micros)[2]); vPutC(((uint8_t*)&u32Micros)[1]); vPutC(((uint8_t*)&u32Micros)[0]);
			vPutC( psPacket->u8PayloadLength + 2+1+2 + 2*bSrcShortAddr + 8*bSrcExtAddr + 2*(1-bIntraPan)*bSrcExtAddr + 2*bDstShortAddr + 8*bDstExtAddr + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);
			vPutC( psPacket->u8PayloadLength + 2+1+2 + 2*bSrcShortAddr + 8*bSrcExtAddr + 2*(1-bIntraPan)*bSrcExtAddr + 2*bDstShortAddr + 8*bDstExtAddr + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);
	// MAC
		// FrameControl
			vPutC(psPacket->u16FrameControl&0xFF); vPutC(psPacket->u16FrameControl>>8);
		// SequenceNumber
			vPutC(psPacket->u8SequenceNumber);
		// Destination Pan ID
			vPutC(psPacket->u16DestinationPanID&0xFF); vPutC(psPacket->u16DestinationPanID>>8);
		// Destination  Address
			if ( bDstShortAddr ) {
				vPutC(psPacket->u16DestinationShortAddress&0xFF); vPutC(psPacket->u16DestinationShortAddress>>8);
			}
			if ( bDstExtAddr ) {
				uint8 *data = (uint8 *)&(psPacket->u64DestinationExtendedAddress);
				vPutC(data[7]); vPutC(data[6]); vPutC(data[5]); vPutC(data[4]); vPutC(data[3]); vPutC(data[2]); vPutC(data[1]); vPutC(data[0]);
			}
		// Source Pan ID
			if ( (bIntraPan==0) && (bSrcExtAddr) ) {
				vPutC(psPacket->u16SourcePanID&0xFF); vPutC(psPacket->u16SourcePanID>>8);
			}
		// Source  Address
			if ( bSrcShortAddr ) {
				vPutC(psPacket->u16SourceShortAddress&0xFF); vPutC(psPacket->u16SourceShortAddress>>8);
			}
			if ( bSrcExtAddr ) {
				uint8 *data = (uint8 *)&(psPacket->u64SourceExtendedAddress);
				vPutC(data[7]); vPutC(data[6]); vPutC(data[5]); vPutC(data[4]); vPutC(data[3]); vPutC(data[2]); vPutC(data[1]); vPutC(data[0]);
			}
		// Payload
			for(cnt = 0; cnt < psPacket->u8PayloadLength; cnt++){
				vPutC( psPacket->u8Payload[cnt] );
			}
		// FCS
			if (FCS) { vPutC(0xAA); vPutC(0xAA); }
		}else {
			// Will need to inform Wireshark that a packet has been received but not forwarded.
		}
		break;
/* Reserved  ------------------------------------------------------------------------------------------------  */
	default:
		// Will need to inform Wireshark that an unsupported packet has been received but not forwarded.
		break;
	}
}

