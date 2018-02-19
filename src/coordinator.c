#include <stdint.h>
#include <MMAC.h>
#include <string.h>
#include "crc-ccitt.h"
#include "coordinator.h"

uint32_t g_u32IEEESeqNumber = 36;

PUBLIC void Dump_Packet(tsPhyFrame * pPHYPacket);

#ifdef DO_COORD_JOB
PUBLIC void DoCoordJob(tsPhyFrame * pPHYPacket)
{
	uint16_t u16fcsCalc = 0;
	uint16_t u16fcsRecv = 0;
	u16fcsCalc = crc_ccitt(0, &pPHYPacket->uPayload.au8Byte[0], pPHYPacket->u8PayloadLength);
	u16fcsRecv = (pPHYPacket->uPayload.au8Byte[pPHYPacket->u8PayloadLength-1] << 8) | pPHYPacket->uPayload.au8Byte[pPHYPacket->u8PayloadLength-2];
	if(u16fcsCalc==u16fcsRecv){
		uint16_t u16FCF     = (pPHYPacket->uPayload.au8Byte[1] << 8) | pPHYPacket->uPayload.au8Byte[0];
		if((u16FCF & 7) == 3){									// MAC Request
			if(pPHYPacket->u8PayloadLength>3 && pPHYPacket->uPayload.au8Byte[3]==7){	// Beacon request
				ZB_Send_Beacon();
			}
		}
	}
}

PUBLIC void ZB_Send_Beacon(void)
{
	tsPhyFrame sSendPacket;
	int len = 0;
// IEEE 802.15.4
	uint16_t u16FCF     = (2 << 14);				// Source address 16 bits
	uint8_t   u8SeqNum  = g_u32IEEESeqNumber++;
	uint16_t u16SrcPAN  = 0x1516;
	uint16_t u16SrcAddr = 0x3615;

	sSendPacket.uPayload.au8Byte[len++] = u16FCF & 0xFF;		// Little Endian
	sSendPacket.uPayload.au8Byte[len++] = u16FCF >> 8;
	sSendPacket.uPayload.au8Byte[len++] = u8SeqNum;
	sSendPacket.uPayload.au8Byte[len++] = u16SrcPAN & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = u16SrcPAN >> 8;
	sSendPacket.uPayload.au8Byte[len++] = u16SrcAddr & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = u16SrcAddr >> 8;

	sSendPacket.uPayload.au8Byte[len++] = 0xFF;			// 16 bits PAN Coordinator, Association permit
	sSendPacket.uPayload.au8Byte[len++] = 0xCF;			//
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// GTS
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Pending addresses
// ZigBee
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Protocol ID
	sSendPacket.uPayload.au8Byte[len++] = 0x22;			// Beacon
	sSendPacket.uPayload.au8Byte[len++] = 0x84;			// 
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID - Should be a random number, but different from other neighbors
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0x15;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0xFF;			// Tx Offset
	sSendPacket.uPayload.au8Byte[len++] = 0xFF;			// Tx Offset
	sSendPacket.uPayload.au8Byte[len++] = 0xFF;			// Tx Offset
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Update ID

	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt(u16fcs, &sSendPacket.uPayload.au8Byte[0], len);
	sSendPacket.uPayload.au8Byte[len++] = u16fcs & 0xFF;		// FCS
	sSendPacket.uPayload.au8Byte[len++] = u16fcs >> 8;		// FCS
	
	sSendPacket.u8PayloadLength = len;
	Dump_Packet(&sSendPacket);
	vMMAC_StartPhyTransmit(&sSendPacket, E_MMAC_TX_START_NOW | E_MMAC_TX_NO_CCA);
}

PUBLIC void ZB_Send_Ack(uint8_t seqnum)
{
	tsPhyFrame sSendPacket;
	int len = 0;
// IEEE 802.15.4
	uint16_t u16FCF     = 2;					// Ack
	uint8_t   u8SeqNum  = seqnum;

	sSendPacket.uPayload.au8Byte[len++] = u16FCF & 0xFF;		// Little Endian
	sSendPacket.uPayload.au8Byte[len++] = u16FCF >> 8;
	sSendPacket.uPayload.au8Byte[len++] = u8SeqNum;

	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt(u16fcs, &sSendPacket.uPayload.au8Byte[0], len);
	sSendPacket.uPayload.au8Byte[len++] = u16fcs & 0xFF;		// FCS
	sSendPacket.uPayload.au8Byte[len++] = u16fcs >> 8;		// FCS

	sSendPacket.u8PayloadLength = len;
	Dump_Packet(&sSendPacket);
	vMMAC_StartPhyTransmit(&sSendPacket, E_MMAC_TX_START_NOW | E_MMAC_TX_NO_CCA);
}

PUBLIC void ZB_Send_Asso_Resp(void)
{
	tsPhyFrame sSendPacket;
	int len = 0;
// IEEE 802.15.4
	uint16_t u16FCF     = (3 << 14) | 0xcc63;			// Source address 64 bits
	uint8_t   u8SeqNum  = g_u32IEEESeqNumber++;
	uint16_t u16SrcPAN  = 0x1516;
	tsExtAddr sMacAddr;
	uint64_t u64DstAddr = 0xAA55DEADBEEF1234;

	vMMAC_GetMacAddress(&sMacAddr);
	
	sSendPacket.uPayload.au8Byte[len++] = u16FCF & 0xFF;		// Little Endian
	sSendPacket.uPayload.au8Byte[len++] = u16FCF >> 8;
	sSendPacket.uPayload.au8Byte[len++] = u8SeqNum;
	sSendPacket.uPayload.au8Byte[len++] = u16SrcPAN & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = u16SrcPAN >> 8;
	sSendPacket.uPayload.au8Byte[len++] = (sMacAddr.u32H >> 24) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (sMacAddr.u32H >> 16) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (sMacAddr.u32H >>  8) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (sMacAddr.u32H >>  0) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (sMacAddr.u32L >> 24) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (sMacAddr.u32L >> 16) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (sMacAddr.u32L >>  8) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (sMacAddr.u32L >>  0) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (u64DstAddr >> 56) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (u64DstAddr >> 48) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (u64DstAddr >> 40) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (u64DstAddr >> 32) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (u64DstAddr >> 24) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (u64DstAddr >> 16) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (u64DstAddr >>  8) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = (u64DstAddr >>  0) & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = 0x02;			// Command identifier
	sSendPacket.uPayload.au8Byte[len++] = 0xCF;			//
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// GTS
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Pending addresses
// ZigBee
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Protocol ID
	sSendPacket.uPayload.au8Byte[len++] = 0x22;			// Beacon
	sSendPacket.uPayload.au8Byte[len++] = 0x84;			// 
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID - Should be a random number, but different from other neighbors
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0x15;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Ext PAN ID
	sSendPacket.uPayload.au8Byte[len++] = 0xFF;			// Tx Offset
	sSendPacket.uPayload.au8Byte[len++] = 0xFF;			// Tx Offset
	sSendPacket.uPayload.au8Byte[len++] = 0xFF;			// Tx Offset
	sSendPacket.uPayload.au8Byte[len++] = 0x00;			// Update ID

	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt(u16fcs, &sSendPacket.uPayload.au8Byte[0], len);
	sSendPacket.uPayload.au8Byte[len++] = u16fcs & 0xFF;		// FCS
	sSendPacket.uPayload.au8Byte[len++] = u16fcs >> 8;		// FCS

	sSendPacket.u8PayloadLength = len;
	Dump_Packet(&sSendPacket);
	vMMAC_StartPhyTransmit(&sSendPacket, E_MMAC_TX_START_NOW | E_MMAC_TX_NO_CCA);
}


#endif	//def DO_COORD_JOB

tsPhyFrame sSendPacket;
PUBLIC void ZB_Send_Beacon_Request(void)
{
	memset(&sSendPacket, 0xAA, sizeof(sSendPacket));
	int len = 0;
// IEEE 802.15.4
	uint16_t u16FCF     = (2 << 10) | 3;		// Destination address 16 bits, command
	uint8_t   u8SeqNum  = g_u32IEEESeqNumber++;
	uint16_t u16DstPAN  = 0xFFFF;
	uint16_t u16DstAddr = 0xFFFF;
	uint8_t       u8Cmd = 0x07;			// Command Identifier : Beacon Request

	sSendPacket.uPayload.au8Byte[len++] = u16FCF & 0xFF;				// Little Endian
	sSendPacket.uPayload.au8Byte[len++] = u16FCF >> 8;
	sSendPacket.uPayload.au8Byte[len++] = u8SeqNum;
	sSendPacket.uPayload.au8Byte[len++] = u16DstPAN & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = u16DstPAN >> 8;
	sSendPacket.uPayload.au8Byte[len++] = u16DstAddr & 0xFF;
	sSendPacket.uPayload.au8Byte[len++] = u16DstAddr >> 8;
	sSendPacket.uPayload.au8Byte[len++] = u8Cmd;

	uint16_t u16fcs = 0;
	u16fcs = crc_ccitt(u16fcs, &sSendPacket.uPayload.au8Byte[0], len);

	sSendPacket.uPayload.au8Byte[len++] = u16fcs & 0xFF;		// FCS
	sSendPacket.uPayload.au8Byte[len++] = u16fcs >> 8;		// FCS

	sSendPacket.u8PayloadLength = len;
	sSendPacket.au8Padding[0] = 0;
	sSendPacket.au8Padding[1] = 0;
	sSendPacket.au8Padding[2] = 0;
	Dump_Packet(&sSendPacket);
	vMMAC_StartPhyTransmit(&sSendPacket, E_MMAC_TX_START_NOW | E_MMAC_TX_NO_CCA);
}


