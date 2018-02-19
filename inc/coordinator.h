#ifndef __coordinator_h__
#define __coordinator_h__

PUBLIC void ZB_Send_Beacon_Request(void);

#ifdef DO_COORD_JOB
PUBLIC void DoCoordJob(tsPhyFrame * pPHYPacket);
PUBLIC void ZB_Send_Beacon(void);
PUBLIC void ZB_Send_Ack(uint8_t seqnum);
PUBLIC void ZB_Send_Asso_Resp(void);
#endif	//def DO_COORD_JOB

#endif	//ndef __coordinator_h__

