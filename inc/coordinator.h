#ifndef __coordinator_h__
#define __coordinator_h__

PUBLIC void ZB_Send_Beacon_Request(void);

#ifdef DO_COORD_JOB
PUBLIC void DoCoordJob(tsPhyFrame * pPHYPacket);
PUBLIC void ZB_Send_Beacon(void);
PUBLIC void ZB_Send_Ack(uint8_t seqnum);
PUBLIC void ZB_Send_Asso_Resp(uint64_t u64DeviceAddr);
PUBLIC void ZB_Send_Transport_Key(uint16_t u16DeviceAddr);
#endif	//def DO_COORD_JOB

struct sTimedPHYFrame {
	uint32_t	u32Seconds;
	uint32_t	u32MicroSeconds;
	uint32_t	u32IncLen;
	tsPhyFrame	sPHYFrame;
};

PUBLIC void Dump_Packet(struct sTimedPHYFrame * pTimedPHYPacket);

#define NB_PHY_BUFFERS_TX	8

extern struct sTimedPHYFrame PHYBufferTx[NB_PHY_BUFFERS_TX];
extern volatile uint32_t g_u32PHYBufTxRIdx;
extern volatile uint32_t g_u32PHYBufTxWIdx;
extern volatile uint32_t g_u32SendInProgress;

extern uint16_t g_u16OurAddr;
extern uint64_t g_u64OurAddr;
extern uint16_t g_u16OurPAN;
extern uint64_t g_u64OurPAN;

extern volatile uint32_t g_u32Seconds;

#endif	//ndef __coordinator_h__

