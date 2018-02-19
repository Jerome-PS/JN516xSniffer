#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "crc-ccitt.h"

const char * pipeName = "/tmp/sharkfifo";

// gcc tst.c src/crc-ccitt.c -I inc -o tst
// wireshark -k -i /tmp/sharkfifo -X lua_script:zb.lua -X lua_script1:comport=/dev/cu.usbserial -X lua_script1:channel=25 &
// mkfifo /tmp/sharkfifo

// IEEE 802.15.4 is litte endian, so work in little endian
#if BYTE_ORDER == BIG_ENDIAN
#define cpu_to_le32(x)	__builtin_bswap32((x))
#define cpu_to_le16(x)	((((x) & 0xFF) << 8) | (((x) >> 8) & 0xFF))
#else
#define cpu_to_le32(x)	(x)
#define cpu_to_le16(x)	(x)
#endif

typedef struct pcap_hdr_s {
        uint32_t magic_number;   /* magic number */
        uint16_t version_major;  /* major version number */
        uint16_t version_minor;  /* minor version number */
         int32_t thiszone;       /* GMT to local correction */
        uint32_t sigfigs;        /* accuracy of timestamps */
        uint32_t snaplen;        /* max length of captured packets, in octets */
        uint32_t network;        /* data link type */
} pcap_hdr_t __attribute__((packed));

typedef struct pcaprec_hdr_s {
        uint32_t ts_sec;         /* timestamp seconds */
        uint32_t ts_usec;        /* timestamp microseconds */
        uint32_t incl_len;       /* number of octets of packet saved in file */
        uint32_t orig_len;       /* actual length of packet */
} pcaprec_hdr_t __attribute__((packed));

#define bool_t	uint8_t
#define uint8	uint8_t
#define uint16	uint16_t
#define uint64	uint64_t
#define FALSE	0
#define TRUE	(!FALSE)

typedef struct {
    bool_t bPacketGood;
    uint16 u16FrameControl;
    uint16 u16SourceShortAddress;
    uint16 u16DestinationShortAddress;
    uint64 u64SourceExtendedAddress;
    uint64 u64DestinationExtendedAddress;
    uint16 u16SourcePanID;
    uint16 u16DestinationPanID;
    uint8 u8PayloadLength;
    uint8 u8Payload[127] __attribute__ ((aligned (4)));
    uint8 u8Energy;
    uint8 u8SQI;
    uint8 u8SequenceNumber;
    uint8 u8LQI;
    uint8 u8RSSI;
} tsJPT_PT_Packet ;

int g_fd = -1;

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

#define PRIVATE
PRIVATE void vPutC(uint8_t u8Data);
PRIVATE void WS_init(void);
PRIVATE void WS_Send_Chan_Num(uint8_t u8Channel);
PRIVATE void WS_Send_Test_Packet(void);
PRIVATE void WS_Dump_Packet(tsJPT_PT_Packet * psPacket);

PRIVATE void ZB_Send_Beacon(void);
PRIVATE void ZB_Send_Beacon_Request(void);
PRIVATE void ZB_Send_Ack(uint8_t seqnum);
PRIVATE void ZB_Send_Asso_Resp(void);

volatile uint32_t g_u32Seconds = 0;
int main(void)
{
	printf("About to open(%s)...", pipeName);fflush(stdout);
	g_fd = open(pipeName, O_WRONLY);
	if(g_fd<0){perror("open");}
	printf("done.\n");
//	ssize_t wr = write(g_fd, buf, len);
	printf("About to init...");fflush(stdout);
	WS_init();
	printf("done.\n");
	printf("About to send channel 25...");fflush(stdout);
	WS_Send_Chan_Num(25);
	printf("done.\n");
//	printf("About to send test packet...");fflush(stdout);
//	WS_Send_Test_Packet();
//	printf("done.\n");
	printf("About to send test Beacon Request...");fflush(stdout);
	ZB_Send_Beacon_Request();
	printf("done.\n");
	printf("About to send test Beacon...");fflush(stdout);
	ZB_Send_Beacon();
	printf("done.\n");
	printf("About to send test Ack (0x33)...");fflush(stdout);
	ZB_Send_Ack(0x33);
	printf("done.\n");
	printf("About to send test Association Response...");fflush(stdout);
	ZB_Send_Asso_Resp();
	printf("done.\n");
	printf("About to close...");fflush(stdout);
	close(g_fd);
	printf("done.\n");
}

PRIVATE void vPutC(uint8_t u8Data)
{
//    vUartWrite(UART_TO_PC, u8Data);
	ssize_t wr = write(g_fd, &u8Data, 1);
//	printf("Write [%02x]\n", u8Data);
	printf("%02x ", u8Data);
	if(wr<0){perror("write");}
//	fsync(g_fd);
}

PRIVATE void WS_init(void)
{
	// C3 : LINKTYPE_IEEE802_15_4	    195	DLT_IEEE802_15_4	    IEEE 802.15.4 wireless Personal Area Network, with each packet having the FCS at the end of the frame.
	// E6 : LINKTYPE_IEEE802_15_4_NOFCS	230	DLT_IEEE802_15_4_NOFCS	IEEE 802.15.4 wireless Personal Area Network, without the FCS at the end of the frame.
	pcap_hdr_t pcap_hdr;
//	pcap_hdr_t pcap_hdr = {.magic_number=0xA1B2C3D4 , .version_major=2, .version_minor=4, .thiszone=0, .sigfigs=0, .snaplen=65535, .network=0xC3};
//	pcap_hdr_t pcap_hdr = {.magic_number=0xD4C3B2A1 , .version_major=0x0200, .version_minor=0x0400, .thiszone=0, .sigfigs=0, .snaplen=0xFFFF0000, .network=0xC3000000};
//	pcap_hdr_t pcap_hdr = {.magic_number=0xD4C3B2A1 , .version_major=0x0200, .version_minor=0x0400, .thiszone=0, .sigfigs=0, .snaplen=0xFFFF0000, .network=0x01000000};
//	pcap_hdr_t pcap_hdr = {.magic_number=0xD4C3B2A1 , .version_major=0x0200, .version_minor=0x0400, .thiszone=0, .sigfigs=0, .snaplen=0xFFFF0000, .network=0xE6000000};
	pcap_hdr.magic_number  = tolel(0xA1B2C3D4);
	pcap_hdr.version_major = toles(2);
	pcap_hdr.version_minor = toles(4);
	pcap_hdr.thiszone      = tolel(0);
	pcap_hdr.sigfigs       = tolel(0);
	pcap_hdr.snaplen       = tolel(65535);
	pcap_hdr.network       = tolel(0xC3);

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
	uint32_t u32Seconds = time(NULL);
	uint32_t u32Fraction = 0;
/*	do{
		u32Seconds  = g_u32Seconds;
		u32Fraction = u32AHI_TickTimerRead();
	}while(u32Seconds!=g_u32Seconds);*/
	
	pcap_rec_hdr.ts_sec  = tolel(u32Seconds);
	pcap_rec_hdr.ts_usec = tolel(u32Fraction*10/16);		// 16 MHz
	dataFrame[lenval++] = 0x07;								// Unknown packet type
	dataFrame[lenval++] = 0x00;								// Unknown packet type
	dataFrame[lenval++] = 0x00;								// Sequence number
	dataFrame[lenval++] = u8Channel;
	float fFreq = afFrequencies[u8Channel - 11];
	*((uint32_t*)&fFreq) = tolel(*((uint32_t*)&fFreq));
	dataFrame[lenval++] = ((uint8_t*)&fFreq)[3];
	dataFrame[lenval++] = ((uint8_t*)&fFreq)[2];
	dataFrame[lenval++] = ((uint8_t*)&fFreq)[1];
	dataFrame[lenval++] = ((uint8_t*)&fFreq)[0];
	pcap_rec_hdr.incl_len = tolel(lenval);
	pcap_rec_hdr.orig_len = tolel(lenval);
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
	uint32_t u32Seconds = time(NULL);
	uint32_t u32Fraction = 0;
/*	do{
		u32Seconds  = g_u32Seconds;
		u32Fraction = u32AHI_TickTimerRead();
	}while(u32Seconds!=g_u32Seconds);*/
	u8tstFrame[0] = ((uint8_t*)&u32Seconds)[3];
	u8tstFrame[1] = ((uint8_t*)&u32Seconds)[2];
	u8tstFrame[2] = ((uint8_t*)&u32Seconds)[1];
	u8tstFrame[3] = ((uint8_t*)&u32Seconds)[0];
	u8tstFrame[4] = ((uint8_t*)&u32Fraction)[3];
	u8tstFrame[5] = ((uint8_t*)&u32Fraction)[2];
	u8tstFrame[6] = ((uint8_t*)&u32Fraction)[1];
	u8tstFrame[7] = ((uint8_t*)&u32Fraction)[0];

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

	uint32_t u32Seconds = time(NULL);
	uint32_t u32Fraction = 0;
	uint32_t u32Micros;
/*	do{
		u32Seconds  = g_u32Seconds;
		u32Fraction = u32AHI_TickTimerRead();
	}while(u32Seconds!=g_u32Seconds);*/
	u32Micros = u32Fraction*10/16;

/* ========================== Analyze Received Packet ==============================================*/
	if (psPacket->bPacketGood) {
		if(psPacket->u16FrameControl & 1 << 6){ bIntraPan = TRUE; }

		/* Source addressing mode */
		switch((psPacket->u16FrameControl & 3 << 14) >> 14){

			/* PAN id and address field not present */
			case 0:
				bSrcShortAddr = FALSE;
				bSrcExtAddr = FALSE;
				break;

			/* Reserved */
			case 1:
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
		if ( (psPacket->u8PayloadLength!=0) && (psPacket->bPacketGood) ) {
			vPutC(u32Seconds >>  0);vPutC(u32Seconds >>  8);vPutC(u32Seconds >> 16);vPutC(u32Seconds >> 24);
			vPutC(u32Micros  >>  0);vPutC(u32Micros  >>  8);vPutC(u32Micros  >> 16);vPutC(u32Micros  >> 24);
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
		if ( (psPacket->u8PayloadLength!=0) && (psPacket->bPacketGood) ) {
			vPutC(u32Seconds >>  0);vPutC(u32Seconds >>  8);vPutC(u32Seconds >> 16);vPutC(u32Seconds >> 24);
			vPutC(u32Micros  >>  0);vPutC(u32Micros  >>  8);vPutC(u32Micros  >> 16);vPutC(u32Micros  >> 24);
			vPutC( psPacket->u8PayloadLength + 9 + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);
			vPutC( psPacket->u8PayloadLength + 9 + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);
	// MAC
		// FrameControl
			vPutC(psPacket->u16FrameControl&0xFF); vPutC(psPacket->u16FrameControl>>8);
		// SequenceNumber
			vPutC(psPacket->u8SequenceNumber);
			if(bDstShortAddr){
		// Destination Pan ID
				vPutC(psPacket->u16DestinationPanID&0xFF); vPutC(psPacket->u16DestinationPanID>>8);
		// Destination Short Address
				vPutC(psPacket->u16DestinationShortAddress&0xFF); vPutC(psPacket->u16DestinationShortAddress>>8);
			}
			if(bSrcShortAddr){
		// Source Short Address
				vPutC(psPacket->u16SourceShortAddress&0xFF); vPutC(psPacket->u16SourceShortAddress>>8);
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
//		if ( (psPacket->u8PayloadLength==0) && (psPacket->bPacketGood) ) {
		if ( (psPacket->bPacketGood) ) {
			vPutC(u32Seconds >>  0);vPutC(u32Seconds >>  8);vPutC(u32Seconds >> 16);vPutC(u32Seconds >> 24);
			vPutC(u32Micros  >>  0);vPutC(u32Micros  >>  8);vPutC(u32Micros  >> 16);vPutC(u32Micros  >> 24);
			vPutC( psPacket->u8PayloadLength + 3 + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);
			vPutC( psPacket->u8PayloadLength + 3 + FCS_Length ); vPutC(0); vPutC(0); vPutC(0);
	// MAC
		// FrameControl
			vPutC(psPacket->u16FrameControl&0xFF); vPutC(psPacket->u16FrameControl>>8);
		// SequenceNumber
			vPutC(psPacket->u8SequenceNumber);
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
/* MAC Command  ---------------------------------------------------------------------------------------------- */
	/* Beacon Request */
	/* Association Request */
	/* Association Response */
	case 3:
		if ( (psPacket->u8PayloadLength!=0) && (psPacket->bPacketGood) ) {
			vPutC(u32Seconds >>  0);vPutC(u32Seconds >>  8);vPutC(u32Seconds >> 16);vPutC(u32Seconds >> 24);
			vPutC(u32Micros  >>  0);vPutC(u32Micros  >>  8);vPutC(u32Micros  >> 16);vPutC(u32Micros  >> 24);
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
				vPutC(psPacket->u16SourceShortAddress&0xFF); vPutC(psPacket->u16SourceShortAddress>>8); }
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

uint32_t g_u32IEEESeqNumber = 36;
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
//!!!
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

	sSendPacket.u8PayloadLength = len;

	sSendPacket.bPacketGood = 1;
	WS_Dump_Packet(&sSendPacket);
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
//!!!
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

	sSendPacket.u8PayloadLength = len;

	sSendPacket.bPacketGood = 1;
	WS_Dump_Packet(&sSendPacket);
}

PRIVATE void ZB_Send_Ack(uint8_t seqnum)
{
	tsJPT_PT_Packet sSendPacket;
	int len = 0;
// IEEE 802.15.4
	sSendPacket.u16FrameControl = 2;			// Ack
	sSendPacket.u8SequenceNumber = seqnum;
	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl & 0xFF);				// Little Endian
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl >> 8);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u8SequenceNumber);

	sSendPacket.u8Payload[len++] = u16fcs & 0xFF;		// FCS
	sSendPacket.u8Payload[len++] = u16fcs >> 8;		// FCS

	sSendPacket.u8PayloadLength = len;

	sSendPacket.bPacketGood = 1;
	WS_Dump_Packet(&sSendPacket);
}

PRIVATE void ZB_Send_Asso_Resp(void)
{
	tsJPT_PT_Packet sSendPacket;
	int len = 0;
// IEEE 802.15.4
	sSendPacket.u16FrameControl = (3 << 14) | 0xcc63;	// Source address 64 bits
	sSendPacket.u8SequenceNumber = g_u32IEEESeqNumber++;
	sSendPacket.u16DestinationPanID = 0x1516;
	sSendPacket.u64SourceExtendedAddress = 0x123456789ABCDEF0;
	sSendPacket.u64DestinationExtendedAddress = 0xAA55DEADBEEF1234;
	sSendPacket.u8Payload[len++] = 0x02;			// Command identifier - Association Response
	sSendPacket.u8Payload[len++] = 0xAA;			// Short address (16 bits)
	sSendPacket.u8Payload[len++] = 0xDD;			// 
	sSendPacket.u8Payload[len++] = 0x00;			// Association status (0 = Success)
//!!!
	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl & 0xFF);				// Little Endian
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16FrameControl >> 8);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u8SequenceNumber);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16DestinationPanID & 0xFF);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u16DestinationPanID >> 8);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64DestinationExtendedAddress >> 56);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64DestinationExtendedAddress >> 48);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64DestinationExtendedAddress >> 40);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64DestinationExtendedAddress >> 32);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64DestinationExtendedAddress >> 24);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64DestinationExtendedAddress >> 16);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64DestinationExtendedAddress >>  8);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64DestinationExtendedAddress >>  0);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64SourceExtendedAddress >> 56);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64SourceExtendedAddress >> 48);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64SourceExtendedAddress >> 40);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64SourceExtendedAddress >> 32);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64SourceExtendedAddress >> 24);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64SourceExtendedAddress >> 16);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64SourceExtendedAddress >>  8);
	u16fcs = crc_ccitt_byte(u16fcs, sSendPacket.u64SourceExtendedAddress >>  0);
	u16fcs = crc_ccitt(u16fcs, &sSendPacket.u8Payload[0], len);

	sSendPacket.u8Payload[len++] = u16fcs & 0xFF;		// FCS
	sSendPacket.u8Payload[len++] = u16fcs >> 8;		// FCS

	sSendPacket.u8PayloadLength = len;

	sSendPacket.bPacketGood = 1;
	WS_Dump_Packet(&sSendPacket);
}

