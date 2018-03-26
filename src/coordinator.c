#include <stdint.h>
#include <MMAC.h>
#include <string.h>
#include <AppHardwareApi.h>
#include "crc-ccitt.h"
#include "coordinator.h"
#include "utils.h"

uint32_t g_u32macDSN = 36;
uint32_t g_u32macBSN = 42;
uint16_t g_u16OurAddr = 0x3615;
uint64_t g_u64OurAddr = 0x0000000000000000;
//uint16_t g_u16OurPAN  = 0x1516;
uint16_t g_u16OurPAN  = 0x6fc2;
//uint64_t g_u64OurPAN  = 0x0015000000000000;
uint64_t g_u64OurPAN  = 0x38ccbd5f9eef6a4b;

uint16_t g_u16DevAddr = 0xaefb;

#ifdef DO_COORD_JOB

#define FCF_SRC_ADDR_MASK	0xC000
#define FCF_SRC_ADDR_EXT	0xC000
#define FCF_SRC_ADDR_SHORT	0x8000
#define FCF_SRC_ADDR_RESVD	0x4000
#define FCF_SRC_ADDR_NONE	0x0000
#define FCF_DST_ADDR_MASK	0x0C00
#define FCF_DST_ADDR_EXT	0x0C00
#define FCF_DST_ADDR_SHORT	0x0800
#define FCF_DST_ADDR_RESVD	0x0400
#define FCF_DST_ADDR_NONE	0x0000
#define FCF_PAN_COMPRESS	0x0040
#define FCF_PAN_ACK_REQ		0x0020
#define FCF_FRM_TYPE_MASK	0x0007
#define FCF_FRM_TYPE_MAC	0x0003
#define FCF_FRM_TYPE_ACK	0x0002
#define FCF_FRM_TYPE_DATA	0x0001
#define FCF_FRM_TYPE_BEACON	0x0000

#define MAC_CMD_ASSOC_REQ	0x01
#define MAC_CMD_ASSOC_RESP	0x02
#define MAC_CMD_DISSOS_NOTIF	0x03
#define MAC_CMD_DATA_REQ	0x04
#define MAC_CMD_PAN_CONFICT	0x05
#define MAC_CMD_ORPHAN_NOTIF	0x06
#define MAC_CMD_BEACON_REQ	0x07
#define MAC_CMD_COORD_REALIGN	0x08
#define MAC_CMD_GTS_REQ		0x09

PRIVATE struct sTimedPHYFrame * FIFO_Get_Buffer(void);
PRIVATE void FIFO_Transmit_Buffer(void);

/*
	Send ACK between macSIFSPeriod and macAckWaitDuration = macSIFSPeriod + phySHRDuration + ceiling(7 × phySymbolsPerOctet)
	macSIFSPeriod – 12 symbols >> 48us???
	bits are 250kHz (4us), symbols are 62.5kHz (16-ary, so 4 bits/symbol)
	192us < Tack < 512us
*/

int iHaveData = 0;

enum eZB_STATE {ZBS_IDLE, ZBS_TRANSPORT_KEY};
enum eZB_STATE ZBState = ZBS_IDLE;

struct sDevTable {
	uint64_t u64LongAddr;
	uint16_t u16ShortAddr;
};

struct sDevTable devAddr;

PUBLIC void DoCoordJob(tsPhyFrame * pPHYPacket)
{
	uint16_t u16fcsCalc = 0;
	uint16_t u16fcsRecv = 0;
	u16fcsCalc = crc_ccitt(0, &pPHYPacket->uPayload.au8Byte[0], pPHYPacket->u8PayloadLength);
	u16fcsRecv = (pPHYPacket->uPayload.au8Byte[pPHYPacket->u8PayloadLength-1] << 8) | pPHYPacket->uPayload.au8Byte[pPHYPacket->u8PayloadLength-2];
	if(1){
//	if(u16fcsCalc==u16fcsRecv){
		uint16_t u16FCF     = (pPHYPacket->uPayload.au8Byte[1] << 8) | pPHYPacket->uPayload.au8Byte[0];
		uint8_t  u8SeqNum   = pPHYPacket->uPayload.au8Byte[2];
		uint8_t * pu8Payload = &pPHYPacket->uPayload.au8Byte[3];
		uint16_t u16DstPAN  = 0x0000;
		uint16_t u16DstAddr = 0x0000;
		uint64_t u64DstAddr = 0x0000000000000000;
		uint16_t u16SrcPAN  = 0x0000;
		uint16_t u16SrcAddr = 0x0000;
		uint64_t u64SrcAddr = 0x0000000000000000;
		if((u16FCF & FCF_DST_ADDR_MASK)){
			u16DstPAN  = (pu8Payload[1] << 8) | pu8Payload[0];	pu8Payload += 2;
		}
		if((u16FCF & FCF_DST_ADDR_MASK) == FCF_DST_ADDR_SHORT){
			u16DstAddr = (pu8Payload[1] << 8) | pu8Payload[0];	pu8Payload += 2;
		}else if((u16FCF & FCF_DST_ADDR_MASK) == FCF_DST_ADDR_EXT){
			u64DstAddr = ((uint64_t)pu8Payload[7] << 56) | ((uint64_t)pu8Payload[6] << 48) | ((uint64_t)pu8Payload[5] << 40) | ((uint64_t)pu8Payload[4] << 32) | (pu8Payload[3] << 24) | (pu8Payload[2] << 16) | (pu8Payload[1] << 8) | pu8Payload[0]; pu8Payload += 8;
		}
		if((u16FCF & FCF_SRC_ADDR_MASK) && !(u16FCF & FCF_PAN_COMPRESS)){
			u16SrcPAN  = (pu8Payload[1] << 8) | pu8Payload[0];	pu8Payload += 2;
		}
		if((u16FCF & FCF_SRC_ADDR_MASK) == FCF_SRC_ADDR_SHORT){
			u16SrcAddr = (pu8Payload[1] << 8) | pu8Payload[0];	pu8Payload += 2;
		}else if((u16FCF & FCF_SRC_ADDR_MASK) == FCF_SRC_ADDR_EXT){
			u64SrcAddr = ((uint64_t)pu8Payload[7] << 56) | ((uint64_t)pu8Payload[6] << 48) | ((uint64_t)pu8Payload[5] << 40) | ((uint64_t)pu8Payload[4] << 32) | (pu8Payload[3] << 24) | (pu8Payload[2] << 16) | (pu8Payload[1] << 8) | pu8Payload[0]; pu8Payload += 8;
		}
// TODO: check that it is for us before responding!
		if(u16FCF & FCF_PAN_ACK_REQ){
//			volatile int n;
//			for(n=0;n<5000;n++){}
			ZB_Send_Ack(u8SeqNum);
		}
		if((u16FCF & FCF_FRM_TYPE_MASK) == FCF_FRM_TYPE_MAC){					// MAC Frame
//			if(pPHYPacket->u8PayloadLength>3 && pPHYPacket->uPayload.au8Byte[7]==7){	// Beacon request
			if(pu8Payload[0]==MAC_CMD_BEACON_REQ){						// Beacon request
//			if(1){
				ZB_Send_Beacon();
			}else if(pu8Payload[0]==MAC_CMD_ASSOC_REQ){					// Beacon request
//				pu8Payload[1]	// Capability information
				while(g_u32SendInProgress){}		//!!! TODO: delay?
//				volatile int n;
//				for(n=0;n<500000;n++){}
				iHaveData = MAC_CMD_ASSOC_RESP;
			}else if(pu8Payload[0]==MAC_CMD_DATA_REQ){					// Data request
				if(iHaveData == MAC_CMD_ASSOC_RESP){
					volatile int n;
					for(n=0;n<14000;n++){}
					ZB_Send_Asso_Resp(u64SrcAddr);
					ZBState = ZBS_TRANSPORT_KEY;
					iHaveData = 0;
				}else if(ZBState == ZBS_TRANSPORT_KEY){
					ZB_Send_Transport_Key(u16SrcAddr);
					ZBState = ZBS_IDLE;
				}
			}
		}
	}
}

PUBLIC void ZB_Send_Beacon(void)
{
	struct sTimedPHYFrame * pTimedSendPacket = FIFO_Get_Buffer();
	tsPhyFrame * pSendPacket = &pTimedSendPacket->sPHYFrame;
	int len = 0;
// IEEE 802.15.4
	uint16_t u16FCF     = (2 << 14);				// Source address 16 bits
	uint8_t   u8SeqNum  = g_u32macBSN++;

	pSendPacket->uPayload.au8Byte[len++] = u16FCF & 0xFF;		// Little Endian
	pSendPacket->uPayload.au8Byte[len++] = u16FCF >> 8;
	pSendPacket->uPayload.au8Byte[len++] = u8SeqNum;
	pSendPacket->uPayload.au8Byte[len++] = g_u16OurPAN  & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = g_u16OurPAN  >> 8;
	pSendPacket->uPayload.au8Byte[len++] = g_u16OurAddr & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = g_u16OurAddr >> 8;

	pSendPacket->uPayload.au8Byte[len++] = 0xFF;				// 16 bits PAN Coordinator, Association permit
	pSendPacket->uPayload.au8Byte[len++] = 0xCF;				//
	pSendPacket->uPayload.au8Byte[len++] = 0x00;				// GTS
	pSendPacket->uPayload.au8Byte[len++] = 0x00;				// Pending addresses
// ZigBee
	pSendPacket->uPayload.au8Byte[len++] = 0x00;				// Protocol ID
	pSendPacket->uPayload.au8Byte[len++] = 0x22;				// Beacon
	pSendPacket->uPayload.au8Byte[len++] = 0x84;				// 
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >>  0) & 0xFF;	// Ext PAN ID - Should be a random number, but different from other neighbors
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >>  8) & 0xFF;	// Ext PAN ID
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >> 16) & 0xFF;	// Ext PAN ID
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >> 24) & 0xFF;	// Ext PAN ID
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >> 32) & 0xFF;	// Ext PAN ID
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >> 40) & 0xFF;	// Ext PAN ID
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >> 48) & 0xFF;	// Ext PAN ID
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >> 56) & 0xFF;	// Ext PAN ID
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >>  0) & 0xFF;	// Ext PAN ID - Should be a random number, but different from other neighbors
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >>  8) & 0xFF;	// Ext PAN ID
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 16) & 0xFF;	// Ext PAN ID
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 24) & 0xFF;	// Ext PAN ID
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 32) & 0xFF;	// Ext PAN ID
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 40) & 0xFF;	// Ext PAN ID
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 48) & 0xFF;	// Ext PAN ID
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 56) & 0xFF;	// Ext PAN ID
	pSendPacket->uPayload.au8Byte[len++] = 0xFF;				// Tx Offset
	pSendPacket->uPayload.au8Byte[len++] = 0xFF;				// Tx Offset
	pSendPacket->uPayload.au8Byte[len++] = 0xFF;				// Tx Offset
	pSendPacket->uPayload.au8Byte[len++] = 0x00;				// Update ID

	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt(u16fcs, &pSendPacket->uPayload.au8Byte[0], len);
	pSendPacket->uPayload.au8Byte[len++] = u16fcs & 0xFF;			// FCS
	pSendPacket->uPayload.au8Byte[len++] = u16fcs >> 8;			// FCS
	
	pSendPacket->u8PayloadLength = len;
	FIFO_Transmit_Buffer();
	Dump_Packet(pTimedSendPacket);
}

PUBLIC void ZB_Send_Ack(uint8_t seqnum)
{
	struct sTimedPHYFrame * pTimedSendPacket = FIFO_Get_Buffer();
	tsPhyFrame * pSendPacket = &pTimedSendPacket->sPHYFrame;
	int len = 0;
// IEEE 802.15.4
	uint16_t u16FCF     = 2;					// Ack
	uint8_t   u8SeqNum  = seqnum;

	pSendPacket->uPayload.au8Byte[len++] = u16FCF & 0xFF;		// Little Endian
	pSendPacket->uPayload.au8Byte[len++] = u16FCF >> 8;
	pSendPacket->uPayload.au8Byte[len++] = u8SeqNum;

	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt(u16fcs, &pSendPacket->uPayload.au8Byte[0], len);
	pSendPacket->uPayload.au8Byte[len++] = u16fcs & 0xFF;		// FCS
	pSendPacket->uPayload.au8Byte[len++] = u16fcs >> 8;		// FCS

	pSendPacket->u8PayloadLength = len;
	FIFO_Transmit_Buffer();
	Dump_Packet(pTimedSendPacket);
}

PUBLIC void ZB_Send_Asso_Resp(uint64_t u64DeviceAddr)
{
	struct sTimedPHYFrame * pTimedSendPacket = FIFO_Get_Buffer();
	tsPhyFrame * pSendPacket = &pTimedSendPacket->sPHYFrame;
	int len = 0;
// IEEE 802.15.4
	uint16_t u16FCF     = (3 << 14) | 0xcc63;			// Source address 64 bits
	uint8_t   u8SeqNum  = g_u32macDSN++;
	uint16_t u16SrcPAN  = 0x1516;

	devAddr.u16ShortAddr = g_u16DevAddr;
	devAddr.u64LongAddr  = u64DeviceAddr;

	pSendPacket->uPayload.au8Byte[len++] = u16FCF & 0xFF;		// Little Endian
	pSendPacket->uPayload.au8Byte[len++] = u16FCF >> 8;
	pSendPacket->uPayload.au8Byte[len++] = u8SeqNum;
	pSendPacket->uPayload.au8Byte[len++] = g_u16OurPAN & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = g_u16OurPAN >> 8;
	pSendPacket->uPayload.au8Byte[len++] = (u64DeviceAddr >>  0) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (u64DeviceAddr >>  8) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (u64DeviceAddr >> 16) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (u64DeviceAddr >> 24) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (u64DeviceAddr >> 32) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (u64DeviceAddr >> 40) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (u64DeviceAddr >> 48) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (u64DeviceAddr >> 56) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >>  0) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >>  8) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 16) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 24) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 32) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 40) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 48) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 56) & 0xFF;
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >>  0) & 0xFF;
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >>  8) & 0xFF;
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >> 16) & 0xFF;
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >> 24) & 0xFF;
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >> 32) & 0xFF;
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >> 40) & 0xFF;
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >> 48) & 0xFF;
//	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurPAN  >> 56) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = MAC_CMD_ASSOC_RESP;	// Command identifier
	pSendPacket->uPayload.au8Byte[len++] = g_u16DevAddr & 0xFF;	// New short address
	pSendPacket->uPayload.au8Byte[len++] = g_u16DevAddr >> 8;	// 
	pSendPacket->uPayload.au8Byte[len++] = 0x00;				// Association Status

	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt(u16fcs, &pSendPacket->uPayload.au8Byte[0], len);
	pSendPacket->uPayload.au8Byte[len++] = u16fcs & 0xFF;			// FCS
	pSendPacket->uPayload.au8Byte[len++] = u16fcs >> 8;			// FCS

	pSendPacket->u8PayloadLength = len;
	FIFO_Transmit_Buffer();
	Dump_Packet(pTimedSendPacket);
}

const uint8_t key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

PUBLIC void ZB_Send_Transport_Key(uint16_t u16DeviceAddr)
{
	struct sTimedPHYFrame * pTimedSendPacket = FIFO_Get_Buffer();
	tsPhyFrame * pSendPacket = &pTimedSendPacket->sPHYFrame;
	int len = 0;
// IEEE 802.15.4
	uint16_t u16FCF     = 0x8861;				// Source address 16 bits
	uint8_t   u8SeqNum  = g_u32macDSN++;
	uint16_t u16SrcPAN  = 0x1516;
	uint32_t u32MIC = 0;

	pSendPacket->uPayload.au8Byte[len++] = u16FCF & 0xFF;		// Little Endian
	pSendPacket->uPayload.au8Byte[len++] = u16FCF >> 8;
	pSendPacket->uPayload.au8Byte[len++] = u8SeqNum;
	pSendPacket->uPayload.au8Byte[len++] = g_u16OurPAN & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = g_u16OurPAN >> 8;
	pSendPacket->uPayload.au8Byte[len++] = (u16DeviceAddr >>  0) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (u16DeviceAddr >>  8) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u16OurAddr  >>  0) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u16OurAddr  >>  8) & 0xFF;

// ZB Network Layer
	pSendPacket->uPayload.au8Byte[len++] = 0x48;
	pSendPacket->uPayload.au8Byte[len++] = 0x00;
	pSendPacket->uPayload.au8Byte[len++] = (u16DeviceAddr >>  0) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (u16DeviceAddr >>  8) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u16OurAddr  >>  0) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u16OurAddr  >>  8) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = 30;								// Radius
	pSendPacket->uPayload.au8Byte[len++] = 127;								// Seqnum

// ZB Application support Layer
	pSendPacket->uPayload.au8Byte[len++] = 0x21;
	pSendPacket->uPayload.au8Byte[len++] = 76;								// Counter
	// ZB Security Header
	pSendPacket->uPayload.au8Byte[len++] = 0x30;
	pSendPacket->uPayload.au8Byte[len++] = 0;								// Frame counter
	pSendPacket->uPayload.au8Byte[len++] = 0;								// Frame counter
	pSendPacket->uPayload.au8Byte[len++] = 0;								// Frame counter
	pSendPacket->uPayload.au8Byte[len++] = 0;								// Frame counter
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >>  0) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >>  8) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 16) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 24) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 32) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 40) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 48) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 56) & 0xFF;
	// Command Frame: Transport Key
	pSendPacket->uPayload.au8Byte[len++] = 0x05;
	pSendPacket->uPayload.au8Byte[len++] = 0x01;
	pSendPacket->uPayload.au8Byte[len++] = key[0];
	pSendPacket->uPayload.au8Byte[len++] = key[1];
	pSendPacket->uPayload.au8Byte[len++] = key[2];
	pSendPacket->uPayload.au8Byte[len++] = key[3];
	pSendPacket->uPayload.au8Byte[len++] = key[4];
	pSendPacket->uPayload.au8Byte[len++] = key[5];
	pSendPacket->uPayload.au8Byte[len++] = key[6];
	pSendPacket->uPayload.au8Byte[len++] = key[7];
	pSendPacket->uPayload.au8Byte[len++] = key[8];
	pSendPacket->uPayload.au8Byte[len++] = key[9];
	pSendPacket->uPayload.au8Byte[len++] = key[10];
	pSendPacket->uPayload.au8Byte[len++] = key[11];
	pSendPacket->uPayload.au8Byte[len++] = key[12];
	pSendPacket->uPayload.au8Byte[len++] = key[13];
	pSendPacket->uPayload.au8Byte[len++] = key[14];
	pSendPacket->uPayload.au8Byte[len++] = key[15];
	pSendPacket->uPayload.au8Byte[len++] = 0;								// Seqnum
	pSendPacket->uPayload.au8Byte[len++] = (devAddr.u64LongAddr >>  0) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (devAddr.u64LongAddr >>  8) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (devAddr.u64LongAddr >> 16) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (devAddr.u64LongAddr >> 24) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (devAddr.u64LongAddr >> 32) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (devAddr.u64LongAddr >> 40) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (devAddr.u64LongAddr >> 48) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (devAddr.u64LongAddr >> 56) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >>  0) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >>  8) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 16) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 24) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 32) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 40) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 48) & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = (g_u64OurAddr  >> 56) & 0xFF;
	
	// MIC
	pSendPacket->uPayload.au8Byte[len++] = (u32MIC >>  0) & 0xFF;	// MIC
	pSendPacket->uPayload.au8Byte[len++] = (u32MIC >>  8) & 0xFF;	// MIC
	pSendPacket->uPayload.au8Byte[len++] = (u32MIC >> 16) & 0xFF;	// MIC
	pSendPacket->uPayload.au8Byte[len++] = (u32MIC >> 24) & 0xFF;	// MIC

	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt(u16fcs, &pSendPacket->uPayload.au8Byte[0], len);
	pSendPacket->uPayload.au8Byte[len++] = u16fcs & 0xFF;			// FCS
	pSendPacket->uPayload.au8Byte[len++] = u16fcs >> 8;				// FCS

	pSendPacket->u8PayloadLength = len;
	FIFO_Transmit_Buffer();
	Dump_Packet(pTimedSendPacket);
}

#endif	//def DO_COORD_JOB

PUBLIC void ZB_Send_Beacon_Request(void)
{
	struct sTimedPHYFrame * pTimedSendPacket = FIFO_Get_Buffer();
	tsPhyFrame * pSendPacket = &pTimedSendPacket->sPHYFrame;
	memset(&pSendPacket, 0xAA, sizeof(*pSendPacket));
	int len = 0;
// IEEE 802.15.4
	uint16_t u16FCF     = (2 << 10) | 3;		// Destination address 16 bits, command
	uint8_t   u8SeqNum  = g_u32macDSN++;
	uint16_t u16DstPAN  = 0xFFFF;
	uint16_t u16DstAddr = 0xFFFF;
	uint8_t       u8Cmd = 0x07;			// Command Identifier : Beacon Request

	pSendPacket->uPayload.au8Byte[len++] = u16FCF & 0xFF;				// Little Endian
	pSendPacket->uPayload.au8Byte[len++] = u16FCF >> 8;
	pSendPacket->uPayload.au8Byte[len++] = u8SeqNum;
	pSendPacket->uPayload.au8Byte[len++] = u16DstPAN & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = u16DstPAN >> 8;
	pSendPacket->uPayload.au8Byte[len++] = u16DstAddr & 0xFF;
	pSendPacket->uPayload.au8Byte[len++] = u16DstAddr >> 8;
	pSendPacket->uPayload.au8Byte[len++] = u8Cmd;

	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt(u16fcs, &pSendPacket->uPayload.au8Byte[0], len);

	pSendPacket->uPayload.au8Byte[len++] = u16fcs & 0xFF;		// FCS
	pSendPacket->uPayload.au8Byte[len++] = u16fcs >> 8;		// FCS

	pSendPacket->u8PayloadLength = len;
	FIFO_Transmit_Buffer();
	Dump_Packet(pTimedSendPacket);
}

PRIVATE struct sTimedPHYFrame * FIFO_Get_Buffer(void)
{
//	uint32_t u32NextWIdx = (g_u32PHYBufTxWIdx+1) % NB_PHY_BUFFERS_TX;
//	if(u32NextWIdx != g_u32PHYBufTxRIdx){}
//TODO: what to do if there is no buffer space available?
	return &PHYBufferTx[g_u32PHYBufTxWIdx];
}

PRIVATE void FIFO_Transmit_Buffer(void)
{
	uint32_t u32Seconds;
	uint32_t u32Fraction;
	uint32_t u32Micros;
	do{
		u32Seconds  = g_u32Seconds;
		u32Fraction = u32AHI_TickTimerRead();
	}while(u32Seconds!=g_u32Seconds);
	u32Micros = u32Fraction/16;
	
	PHYBufferTx[g_u32PHYBufTxWIdx].u32Seconds      = cpu_to_le32(u32Seconds);
	PHYBufferTx[g_u32PHYBufTxWIdx].u32MicroSeconds = cpu_to_le32(u32Micros);
	// TODO: time this in ISR!!!

	uint32_t u32NextWIdx = (g_u32PHYBufTxWIdx+1) % NB_PHY_BUFFERS_TX;
	g_u32PHYBufTxWIdx = u32NextWIdx;
	if(!g_u32SendInProgress){
		vMMAC_StartPhyTransmit(&PHYBufferTx[g_u32PHYBufTxRIdx].sPHYFrame, E_MMAC_TX_START_NOW | E_MMAC_TX_NO_CCA);
		g_u32PHYBufTxRIdx = (g_u32PHYBufTxRIdx + 1) % NB_PHY_BUFFERS_TX;
		g_u32SendInProgress = 1;
	}
}

