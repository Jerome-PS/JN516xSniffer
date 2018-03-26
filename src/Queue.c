/****************************************************************************
 *
 * MODULE:             Queue
 *
 */
/****************************************************************************
*
* This software is owned by NXP B.V. and/or its supplier and is protected
* under applicable copyright laws. All rights are reserved. We grant You,
* and any third parties, a license to use this software solely and
* exclusively on NXP products [NXP Microcontrollers such as JN5148, JN5142, JN5139].
* You, and any third parties must reproduce the copyright and warranty notice
* and any other legend of ownership on each copy or partial copy of the
* software.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.

* Copyright NXP B.V. 2012. All rights reserved
*
***************************************************************************/
/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include "Queue.h"
#include "utils.h"
#include <string.h>

#ifdef DEBUG_QUEUE
 #include "Printf.h"
#endif

#define COPY_UPCOUNT_BYTE

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME:       vQueue_Init
 *
 * DESCRIPTION:
 *
 * PARAMETERS: Name     RW  Usage
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vQueue_Init(tsQueue *psQueue, uint8 *pau8Buffer, uint32 u32Length)
{

	/* Initialise the event queue */
	psQueue->u32ReadPtr = 0;
	psQueue->u32WritePtr = 0;
	psQueue->u32Length = u32Length;
	psQueue->pau8Buffer = pau8Buffer;

}

/****************************************************************************
 *
 * NAME:       vQueue_Flush
 *
 * DESCRIPTION:
 *
 * PARAMETERS: Name     RW  Usage
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vQueue_Flush(tsQueue *psQueue)
{

	/* Initialise the event queue */
	psQueue->u32ReadPtr = 0;
	psQueue->u32WritePtr = 0;

}

/****************************************************************************
 *
 * NAME:       bPPP_QueueWrite
 *
 * DESCRIPTION:
 *
 * PARAMETERS: Name     RW  Usage
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC bool bQueue_Write(tsQueue *psQueue, uint8 u8Item)
{

	/* Make a copy of the write pointer */
	uint32 u32NewWritePtr = psQueue->u32WritePtr;

	u32NewWritePtr++;
	if(u32NewWritePtr == psQueue->u32Length)
	{
		u32NewWritePtr = 0;
	}

	/* If new incremented pointer is same as read pointer, queue is full */
	if(u32NewWritePtr == psQueue->u32ReadPtr)
	{
		return(FALSE);
	}

	psQueue->pau8Buffer[psQueue->u32WritePtr] = u8Item;	/* Add item to queue */
	psQueue->u32WritePtr = u32NewWritePtr;				/* Write new pointer */

	return(TRUE);
}

/****************************************************************************
 *
 * NAME:       bPPP_QueueWriteBlock
 *
 * DESCRIPTION: Write a block of data to the queue
 *
 * PARAMETERS: Name     RW  Usage
 *
 * RETURNS:
 * Number of bytes pushed into the FIFO.
 *
 ****************************************************************************/
PUBLIC uint32 bQueue_WriteBlock(tsQueue *psQueue, const uint8 * pu8Item, uint32 u32Len)
{
	uint32_t u32AlreadyCopied = 0;
	uint32_t u32LeftToCopy = u32Len;

	// Check how much contiguous space is available
	uint32_t u32ReadPtr = psQueue->u32ReadPtr;
	uint32_t u32WritPtr = psQueue->u32WritePtr;
	uint32_t u32Avail   = 0;
	if(u32ReadPtr > u32WritPtr){				/* Write up to read pointer (-1) */
		u32Avail = u32ReadPtr - 1 - u32WritPtr;
	}else{							/* Write up to the end, EXCEPT if ReadPtr = 0 ... */
		u32Avail = psQueue->u32Length - u32WritPtr;
		if(u32ReadPtr == 0 && u32Avail != 0){		/* Read pointer is at 0, we are not allowed to write up to the end */
			u32Avail -= 1;
		}
	}
//    vPrintf("u32ReadPtr=%d, u32WritPtr=%d, u32Avail=%d, u32LeftToCopy=%d\n", u32ReadPtr, u32WritPtr, u32Avail, u32LeftToCopy);
	uint32_t u32CanCopy = min(u32Avail, u32LeftToCopy);
	int cnt;(void)cnt;
#if defined(COPY_MEMCPY)
	memcpy(&psQueue->pau8Buffer[u32WritPtr], &pu8Item[0], u32CanCopy);
#elif defined(COPY_UPCOUNT_BYTE)
	for(cnt=0;cnt<u32CanCopy;cnt++){
		psQueue->pau8Buffer[u32WritPtr+cnt] = pu8Item[cnt];
	}
#elif defined(COPY_DOWNCOUNT_BYTE)
	for(cnt=u32CanCopy-1;cnt>=0;cnt--){
		psQueue->pau8Buffer[u32WritPtr+cnt] = pu8Item[cnt];
	}
#elif defined(COPY_UPCOUNT_BYTE_POINTER)
	uint8_t * pu8Src = pu8Item;
	uint8_t * pu8Dst = &psQueue->pau8Buffer[u32WritPtr];
	for(cnt=0;cnt<u32CanCopy;cnt++){
		*pu8Dst++ = *pu8Src++;
	}
#elif defined(COPY_DOWNCOUNT_BYTE_POINTER)
	uint8_t * pu8Src = pu8Item;
	uint8_t * pu8Dst = &psQueue->pau8Buffer[u32WritPtr];
	for(cnt=u32CanCopy-1;cnt>=0;cnt--){
		*pu8Dst++ = *pu8Src++;
	}
#else
#error("No copy method specified!");
#endif
	u32LeftToCopy -= u32CanCopy;
	u32WritPtr += u32CanCopy;
	u32AlreadyCopied += u32CanCopy;
//    vPrintf("u32CanCopy=%d, u32LeftToCopy=%d, u32WritPtr=%d, u32LeftToCopy=%d\n", u32CanCopy, u32LeftToCopy, u32WritPtr, u32LeftToCopy);
	if(u32WritPtr==psQueue->u32Length){u32WritPtr=0;}
	if(u32LeftToCopy && u32WritPtr==0){		// Not written everything yet, there is space only if we folded the write pointer
		u32Avail = u32ReadPtr - 1 - u32WritPtr;
		u32CanCopy = min(u32Avail, u32LeftToCopy);
		memcpy(&psQueue->pau8Buffer[u32WritPtr], &pu8Item[u32AlreadyCopied], u32CanCopy);
		u32LeftToCopy -= u32CanCopy;
		u32WritPtr += u32CanCopy;
		u32AlreadyCopied += u32CanCopy;
	}

	// Synchronize the pointer with the Queue
	psQueue->u32WritePtr = u32WritPtr;

	return u32AlreadyCopied;
}

/****************************************************************************
 *
 * NAME:       bPPP_QueueRead
 *
 * DESCRIPTION:
 *
 * PARAMETERS: Name     RW  Usage
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC bool bQueue_Read(tsQueue *psQueue, uint8 *pu8Item)
{

	/* If pointers are same, nothing in the queue */
	if(psQueue->u32ReadPtr == psQueue->u32WritePtr)
	{
		return(FALSE);
	}

	/* Read an event from the queue */
	*pu8Item = psQueue->pau8Buffer[psQueue->u32ReadPtr++];

	if(psQueue->u32ReadPtr == psQueue->u32Length)
	{
		psQueue->u32ReadPtr = 0;
	}

	return(TRUE);

}

/****************************************************************************
 *
 * NAME:       bQueue_IsEmpty
 *
 * DESCRIPTION:
 *
 * PARAMETERS: Name     RW  Usage
 *
 * RETURNS:
 * bool:	TRUE if the queue is empty
 *			FALSE if it contains data
 *
 ****************************************************************************/
PUBLIC bool bQueue_IsEmpty(tsQueue *psQueue)
{

	/* If pointers are same, nothing in the queue */
	if(psQueue->u32ReadPtr == psQueue->u32WritePtr)
	{
		return(TRUE);
	}
	else
	{
		return(FALSE);
	}

}


/****************************************************************************
 *
 * NAME:       bQueue_IsFull
 *
 * DESCRIPTION:
 *	Checks if the queue is full or not
 *
 * PARAMETERS: 	Name     		RW  Usage
 *
 * RETURNS:
 * bool:	TRUE if the queue is full
 *			FALSE if it contains data
 *
 ****************************************************************************/
PUBLIC bool bQueue_IsFull(tsQueue *psQueue)
{

	/* Queue can only ever hold u32Length -1 bytes max */
	if(u32Queue_GetDepth(psQueue) == psQueue->u32Length - 1)
	{
		return(TRUE);
	}
	else
	{
		return(FALSE);
	}

}


/****************************************************************************
 *
 * NAME:       u32Queue_GetDepth
 *
 * DESCRIPTION:
 *	Returns the number of bytes in the queue
 *
 * PARAMETERS: 	Name     		RW  Usage
 *
 * RETURNS:
 *	uint32:		Number of bytes in the queue
 *
 ****************************************************************************/
PUBLIC uint32 u32Queue_GetDepth(tsQueue *psQueue)
{

	/* If pointers are same, nothing in the queue */
	if(psQueue->u32ReadPtr == psQueue->u32WritePtr)
	{
		return(0);
	}


	if(psQueue->u32WritePtr > psQueue->u32ReadPtr)
	{
		return(psQueue->u32WritePtr - psQueue->u32ReadPtr);
	}
	else
	{
		return(psQueue->u32Length - (psQueue->u32ReadPtr - psQueue->u32WritePtr));
	}

}


/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
