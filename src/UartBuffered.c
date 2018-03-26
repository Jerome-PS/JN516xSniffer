/****************************************************************************
 *
 * MODULE:             Buffered, interrupt driven serial I/O
 *
 * COMPONENT:          $RCSfile: UartBuffered_dma.c,v $
 *
 * VERSION:            $Name:  $
 *
 * REVISION:           $Revision: 1.2 $
 *
 * DATED:              $Date: 2009/01/22 14:02:35 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:
 *
 * DESCRIPTION:
 * common uart functions for 6x variants added
 *
 * Renamed to _dma to differentiate from original uartbuffered
 *
 * LAST MODIFIED BY:   $Author: lmitch $
 *                     $Modtime: $
 *
 ****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on
 * each copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd 2005, 2006, 2012. All rights reserved
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <AppHardwareApi.h>
#include "UartBuffered.h"
#include "Queue.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* Note: no longer using direct register access after JN514x */
#if (defined JN5139R1) || (defined JN5139)
#define UART_0_BASE_ADDR    0x30000000UL
#define UART_1_BASE_ADDR    0x40000000UL
#endif

#if ((defined JN5148) || (defined JN5148J01) || (defined JN5168) || (defined JN5148Z01))
#define UART_0_BASE_ADDR    0x02003000UL
#define UART_1_BASE_ADDR    0x02004000UL
#endif

#if (defined JN5142) || (defined JN5142J01)
#define UART_0_BASE_ADDR    0x02003000UL
#endif

#if (defined JN5142) || (defined JN5142J01)
#define UART_NUM_UARTS  1
#else
#define UART_NUM_UARTS  2
#endif

#define UART_DLM_OFFSET     0x04             /**< Offset of UART's DLM register */
#define UART_LCR_OFFSET     0x0C             /**< Offset of UART's LCR register */
#define UART_MCR_OFFSET     0x10             /**< Offset of UART's MCR register */
#define UART_EFR_OFFSET     0x20             /**< Offset of UART's EFR register */

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

PUBLIC void vUartSetBaudRate(uint8 u8Uart, uint32 u32BaudRate);
PRIVATE void vUartISR(uint32 u32DeviceId, uint32 u32ItemBitmap);
PRIVATE void vUartTxIsr(uint8 u8Uart);
PRIVATE void vUartRxIsr(uint8 u8Uart);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

PUBLIC tsQueue     asUart_TxQueue[UART_NUM_UARTS];
PRIVATE tsQueue     asUart_RxQueue[UART_NUM_UARTS];

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME:       vUartInit
 *
 * DESCRIPTION:
 * Initialises the specified UART.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   UART to initialise, eg, E_AHI_UART_0
 *                  u8BaudRate      R   Baudrate to use,
 *                                      eg,E_AHI_UART_RATE_9600
 *                  psFifo          R   Pointer to a tsUartFifo struct holding
 *                                      all data for this UART
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vUartInit(uint8 u8Uart, uint32 u32BaudRate, uint8 *pu8TxBuffer, uint32 u32TxBufferLen, uint8 *pu8RxBuffer, uint32 u32RxBufferLen)
{

    /* Initialise Tx & Rx Queue's */
    vQueue_Init(&asUart_TxQueue[u8Uart], pu8TxBuffer, u32TxBufferLen);
    vQueue_Init(&asUart_RxQueue[u8Uart], pu8RxBuffer, u32RxBufferLen);

    /* Configure the selected Uart */
    vAHI_UartEnable(u8Uart);

    vAHI_UartReset(u8Uart, TRUE, TRUE);
    vAHI_UartReset(u8Uart, FALSE, FALSE);

    vAHI_UartSetClockDivisor(u8Uart, (uint8)u32BaudRate);
//  vUartSetBaudRate(u8Uart, u32BaudRate);

#if (defined JN5139R1) || (defined JN5139)
    vAHI_UartSetRTSCTS(u8Uart, FALSE);
#endif

    /* install interrupt service calback */
    if(u8Uart == E_AHI_UART_0){
        vAHI_Uart0RegisterCallback((void*)vUartISR);
    } else {
        #if UART_NUM_UARTS == 2
        vAHI_Uart1RegisterCallback((void*)vUartISR);
        #endif
    }

    /* Enable TX Fifo empty and Rx data interrupts */
    vAHI_UartSetInterrupt(u8Uart, FALSE, FALSE, TRUE, TRUE, E_AHI_UART_FIFO_LEVEL_1);

}

/****************************************************************************
 *
 * NAME:       vUartSetBaudRate
 *
 * DESCRIPTION:
 * Sets the baud rate for the specified uart
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vUartSetBaudRate(uint8 u8Uart, uint32 u32BaudRate)
{
#if (defined JN5139R1) || (defined JN5139) || (defined JN5148) || (defined JN5148J01) || (defined JN5168) || (defined JN5148Z01) || (defined JN5142) || (defined JN5142J01)

    uint8 *pu8Reg;
    uint8  u8TempLcr;
    uint16 u16Divisor;
    uint32 u32Remainder;
    uint32 u32UartStartAddr;

    if(u8Uart == E_AHI_UART_0)
    {
        u32UartStartAddr    = UART_0_BASE_ADDR;
    }
    else
    {
     #if UART_NUM_UARTS == 2
        u32UartStartAddr    = UART_1_BASE_ADDR;
     #endif
    }


    /* Put UART into clock divisor setting mode */
    pu8Reg    = (uint8 *)(u32UartStartAddr + UART_LCR_OFFSET);
    u8TempLcr = *pu8Reg;
    *pu8Reg   = u8TempLcr | 0x80;

    /* Write to divisor registers:
       Divisor register = 16MHz / (16 x baud rate) */
    u16Divisor = (uint16)(16000000UL / (16UL * u32BaudRate));

    /* Correct for rounding errors */
    u32Remainder = (uint32)(16000000UL % (16UL * u32BaudRate));

    if (u32Remainder >= ((16UL * u32BaudRate) / 2))
    {
        u16Divisor += 1;
    }

    pu8Reg  = (uint8 *)u32UartStartAddr;
    *pu8Reg = (uint8)(u16Divisor & 0xFF);
    pu8Reg  = (uint8 *)(u32UartStartAddr + UART_DLM_OFFSET);
    *pu8Reg = (uint8)(u16Divisor >> 8);

    /* Put back into normal mode */
    pu8Reg    = (uint8 *)(u32UartStartAddr + UART_LCR_OFFSET);
    u8TempLcr = *pu8Reg;
    *pu8Reg   = u8TempLcr & 0x7F;

#else

    uint16 u16Divisor;

    /* Divide baud rate into divisor, accounting for rounding errors
     *
     * Divisor register = 16MHz / (16 x baud rate)
     *                  =  1MHz / (baud rate)
     *
     * To round the value, multiply everything by 2, add 1 and then divide by
     * 2; effectively this adds 0.5 before the value gets truncated, which is
     * a standard way to round a value using integer arithmetic */
    u16Divisor = ((2000000UL / u32BaudRate) + 1) / 2;

    /* Use the Hardware API function */
    vAHI_UartSetBaudDivisor(u8Uart, u16Divisor);

#endif
}

#ifdef UART_EXTRAS
/****************************************************************************
 *
 * NAME:       vUartDeInit
 *
 * DESCRIPTION:
 * Disables the specified UART.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   UART to disable, eg, E_AHI_UART_0
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vUartDeInit(uint8 u8Uart)
{

    /* Wait for any transmissions to complete */
    while(bUartTxInProgress(u8Uart));

    /* Disable TX Fifo empty and Rx data interrupts */
    vAHI_UartSetInterrupt(u8Uart, FALSE, FALSE, FALSE, FALSE, E_AHI_UART_FIFO_LEVEL_1);
#if 1
    /* remove interrupt service callback */
    if(u8Uart == E_AHI_UART_0)
    {
        vAHI_Uart0RegisterCallback((void*)NULL);
    }
    else
    {

        #if UART_NUM_UARTS == 2
        vAHI_Uart1RegisterCallback((void*)NULL);
        #endif
    }
#endif

    vAHI_UartDisable(u8Uart);
}
#endif

#ifdef UART_EXTRAS
/****************************************************************************
 *
 * NAME:       bUartReadBinary
 *
 * DESCRIPTION:
 * Reads a specified number of bytes from the uart and stores them in an area
 * of memory. Times out if no data is received for a specified time.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   UART to read from, eg. E_AHI_UART_0
 *                  pu8Ptr          W   Pointer to an area of memory that
 *                                      will receive the data.
 *                  u32Len          R   Number of bytes to receive
 *                  u32TimeoutTime  R   How long to wait for data before
 *                                      timeout
 *
 * RETURNS:
 * bool_t, TRUE if specified number of bytes were read, FALSE if timed out
 *
 ****************************************************************************/
PUBLIC bool_t bUartReadBinary(uint8 u8Uart, uint8 *pu8Ptr, uint32 u32Len, uint32 u32TimeoutTime)
{

    uint32 n;

    for(n = 0; n < u32Len; n++){
        if(!bUartReadWithTimeout(u8Uart, pu8Ptr++, u32TimeoutTime)){
            return(FALSE);
        }
    }

    return(TRUE);
}
#endif

#ifdef UART_EXTRAS
/****************************************************************************
 *
 * NAME:       bUartReadWithTimeout
 *
 * DESCRIPTION:
 * Attempts to read 1 byte from the RX buffer. If there is no data in the
 * buffer, then it will wait for u32TimeoutTime, and then exit.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   UART to use, eg, E_AHI_UART_0
 *                  pu8Data         W   pointer to 8 bit RX data destination
 *                  u32TimeoutTime  R   Time to wait for data to if there
 *                                      is none already in the buffer
 *
 * RETURNS:
 * bool_t: TRUE if data was read, data is left in *pu8Data,
 *         FALSE if no data was read
 *
 ****************************************************************************/
PUBLIC bool_t bUartReadWithTimeout(uint8 u8Uart, uint8 *pu8Data, uint32 u32TimeoutTime)
{

    uint32 u32Time;

    for(u32Time = 0; u32Time < u32TimeoutTime; u32Time++){

        if(bQueue_Read(&asUart_RxQueue[u8Uart], pu8Data))
        {
            return(TRUE);
        }

    }

    *pu8Data = 0;
    return(FALSE);
}
#endif

#ifdef UART_EXTRAS
/****************************************************************************
 *
 * NAME:       u8UartRead
 *
 * DESCRIPTION:
 * Reads 1 byte from the RX buffer. If there is no data in the
 * buffer, then it will wait until there is.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   UART to use, eg, E_AHI_UART_0
 *
 * RETURNS:
 * uint8: received data
 *
 ****************************************************************************/
PUBLIC uint8 u8UartRead(uint8 u8Uart)
{

    uint8 u8Data;

    while(!bQueue_Read(&asUart_RxQueue[u8Uart], &u8Data));

    return(u8Data);

}
#endif

/****************************************************************************
 *
 * NAME:       u8UartRead
 *
 * DESCRIPTION:
 * Reads 1 byte from the RX buffer. If there is no data in the
 * buffer, then it will wait until there is.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   UART to use, eg, E_AHI_UART_0
 *
 * RETURNS:
 * uint8: received data
 *
 ****************************************************************************/
PUBLIC bool bUartRead(uint8 u8Uart, uint8 *pu8Data)
{

    return(bQueue_Read(&asUart_RxQueue[u8Uart], pu8Data));

}

#ifdef UART_EXTRAS
/****************************************************************************
 *
 * NAME:       bUartTxInProgress
 *
 * DESCRIPTION:
 * Returns the state of data transmission
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   UART to use, eg, E_AHI_UART_0
 *
 * RETURNS:
 * bool_t: TRUE if data in buffer is being transmitted
 *         FALSE if all data in buffer has been transmitted by the UART
 *
 ****************************************************************************/
PUBLIC bool_t bUartTxInProgress(uint8 u8Uart)
{

    if(bQueue_IsEmpty(&asUart_TxQueue[u8Uart]))
    {

        if((u8AHI_UartReadLineStatus(u8Uart) & E_AHI_UART_LS_TEMT ) != 0)
        {
            return(FALSE);
        }

    }

    return(TRUE);

}
#endif

#ifdef UART_EXTRAS
/****************************************************************************
 *
 * NAME:       bUartRxDataAvailable
 *
 * DESCRIPTION:
 * Returns state of data reception
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   UART to use, eg, E_AHI_UART_0
 *
 * RETURNS:
 * bool_t: TRUE if there is received data in the buffer
 *         FALSE if there is no received data
 *
 ****************************************************************************/
PUBLIC bool_t bUartRxDataAvailable(uint8 u8Uart)
{

    return(!bQueue_IsEmpty(&asUart_RxQueue[u8Uart]));

}
#endif

#ifdef UART_EXTRAS
/****************************************************************************
 *
 * NAME:       u32UartWriteBinary
 *
 * DESCRIPTION:
 * Writes a specified number of bytes to the uart.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   UART to write to, eg. E_AHI_UART_0
 *                  pu8Ptr          W   Pointer to an area of memory that
 *                                      contains the data to send
 *                  u32Len          R   Number of bytes to send
 *
 * RETURNS:
 * Number of byte pushed into the FIFO.
 *
 ****************************************************************************/
PUBLIC uint32 u32UartWriteBinary(uint8 u8Uart, const uint8 *pu8Ptr, uint32 u32Len)
{
	uint32 u32NbWritten = bQueue_WriteBlock(&asUart_TxQueue[u8Uart], pu8Ptr, u32Len);
	
	// Check if we need to start the ISR pump
	if ((u8AHI_UartReadLineStatus(u8Uart) & (E_AHI_UART_LS_THRE|E_AHI_UART_LS_TEMT)) == (E_AHI_UART_LS_THRE|E_AHI_UART_LS_TEMT)){
		uint8 u8Data;
		if(bQueue_Read(&asUart_TxQueue[u8Uart], &u8Data)){
			vAHI_UartWriteData(u8Uart, u8Data);
		}
	}

	return u32NbWritten;
}
#endif

#ifdef UART_EXTRAS
/****************************************************************************
 *
 * NAME:       bUartReadBinary
 *
 * DESCRIPTION:
 * Writes a null terminated string to the specified UART
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   UART to write to, eg. E_AHI_UART_0
 *                  pu8String       W   Pointer to a null terminated string
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vUartWriteString(uint8 u8Uart, uint8 *pu8String)
{

    while(*pu8String != '\0'){
        vUartWrite(u8Uart, *pu8String);
        if(*pu8String++ == '\r'){
            vUartWrite(u8Uart, '\n');
        }
    }

}
#endif

/****************************************************************************
 *
 * NAME:       vUartWrite
 *
 * DESCRIPTION:
 * Writes one byte to the specified uart for transmission
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   UART to use, eg, E_AHI_UART_0
 *                  u8Data          R   data to transmit
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vUartWrite(uint8 u8Uart, uint8 u8Data)
{

    while(!bQueue_Write(&asUart_TxQueue[u8Uart], u8Data));


    /*
     * if there is already a tx in progress, we can expect a TX interrupt
     * some time in the future, in which case the data we wrote to the tx
     * buffer will get sent in due course to the UART in the interrupt
     * service routine.
     * if there is no tx in progress, there won't be a tx interrupt, and the
     * byte won't get read from the buffer in the ISR, so we must write it
     * to the UART tx FIFO here instead,
     */
    if ((u8AHI_UartReadLineStatus(u8Uart) & (E_AHI_UART_LS_THRE|E_AHI_UART_LS_TEMT)) == (E_AHI_UART_LS_THRE|E_AHI_UART_LS_TEMT))
    {

        if(bQueue_Read(&asUart_TxQueue[u8Uart], &u8Data))
        {

            vAHI_UartWriteData(u8Uart, u8Data);

        }

    }

}

#ifdef UART_EXTRAS
/****************************************************************************
 *
 * NAME:       vUartFlush
 *
 * DESCRIPTION:
 * Flushes the buffers of the specified UART
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   UART to disable, eg, E_AHI_UART_0
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vUartFlush(uint8 u8Uart)
{


    /* Disable TX Fifo empty and Rx data interrupts */
    vAHI_UartSetInterrupt(u8Uart, FALSE, FALSE, FALSE, FALSE, E_AHI_UART_FIFO_LEVEL_1);

    vQueue_Flush(&asUart_TxQueue[u8Uart]);
    vQueue_Flush(&asUart_RxQueue[u8Uart]);

    /* flush hardware buffer */
    vAHI_UartReset(u8Uart, TRUE, TRUE);
    vAHI_UartReset(u8Uart, FALSE, FALSE);

    /* Re-enable TX Fifo empty and Rx data interrupts */
    vAHI_UartSetInterrupt(u8Uart, FALSE, FALSE, TRUE, TRUE, E_AHI_UART_FIFO_LEVEL_1);

}
#endif

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME:       vUartISR
 *
 * DESCRIPTION:
 * Interrupt service callback for UART's
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32DeviceId     R   Device ID of whatever generated the
 *                                      interrupt
 *                  u32ItemBitmap   R   Which part of the device generated
 *                                      the interrupt
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vUartISR(uint32 u32DeviceId, uint32 u32ItemBitmap)
{

    uint8 u8Uart;


    switch(u32DeviceId){

    case E_AHI_DEVICE_UART0:
        u8Uart = 0;
        break;
    #if UART_NUM_UARTS == 2
    case E_AHI_DEVICE_UART1:
        u8Uart = 1;
        break;
    #endif
    default:
        return;
    }


    switch(u32ItemBitmap){

    case E_AHI_UART_INT_TX:
        vUartTxIsr(u8Uart);
        break;

    case E_AHI_UART_INT_RXDATA:
        vUartRxIsr(u8Uart);
        break;

    }

}


/****************************************************************************
 *
 * NAME:       vUartTxISR
 *
 * DESCRIPTION:
 * Interrupt service callback for UART data transmission. Checks the tx buffer
 * for any data waiting for transmission, and if any is available, will write
 * up to 16 bytes of it to the UART's hardware fifo, set the tx in progress
 * flag then exit.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   Uart to write to
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vUartTxIsr(uint8 u8Uart)
{

    uint8 u8Data;
    uint32 u32Bytes = 0;

    /*
     * if there is data in buffer waiting for tx and we've not filled the
     * hardware fifo up
     */

    while(u32Bytes++ < 16 && bQueue_Read(&asUart_TxQueue[u8Uart], &u8Data))
    {

        vAHI_UartWriteData(u8Uart, u8Data); /* write one byte to the UART */

    }

}


/****************************************************************************
 *
 * NAME:       vUartRxISR
 *
 * DESCRIPTION:
 * Interrupt service callback for UART data reception. Reads a received
 * byte from the UART and writes it to the reception buffer if it is not
 * full.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8Uart          R   Uart to read from
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vUartRxIsr(uint8 u8Uart)
{

    bQueue_Write(&asUart_RxQueue[u8Uart], u8AHI_UartReadData(u8Uart));

}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

