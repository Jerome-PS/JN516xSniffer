/****************************************************************************
 *
 * MODULE:             Printf Function
 *
 * DESCRIPTION:
 * Code to provide a simple printf function
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

#include <stdarg.h>
#include <ctype.h>
#include "Printf.h"
#include <string.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/


/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

PRIVATE void vJPTNum2String(uint32 u32Number, uint8 u8Base, uint8 u8Digits, bool_t bPadWithZeros, bool_t bPrefixWithBase);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/* pointer to whatever putchar function the user gives us */
PRIVATE void (*vPutChar) (char c) = NULL;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/*
* Before using these functions, you must call vInitPrintf
* and give it a pointer to a function that will write the individual
* characters to whatever device you decide to use for displaying output.
*
* example below :-
*
*   #include "Printf.h"
*
*   void main(void)
*   {
*
*       vInitPrintf((void*)vPutC);  // pass the pointer to the putc function
*
*       vPrintf("\nHello World!");  // show some text !
*       while(1);
*   }
*
*   void vPutC(char c)
*   {
*       do something with c here, maybe write it to a uart
*   }
*
*/

/****************************************************************************
 *
 * NAME:       vInitPrintf
 *
 * DESCRIPTION:
 * Sets the putchar function that the printf function will use to output
 * its data.
 *
 * PARAMETERS:  Name    RW  Usage
 *              fp      R   Pointer to a function that accepts a character
 *                          as a parameter and returns void.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vInitPrintf(void (*fp)(char c))
{
    vPutChar = fp;
}


/****************************************************************************
 *
 * NAME:       vPrintf
 *
 * DESCRIPTION:
 * Outputs formatted text to the specified device
 *
 * PARAMETERS:  Name        RW  Usage
 *              fmt         R   pointer to string containing text and format
 *                              specifiers
 *              ...         R   extra values that were specified in the
 *                              format string
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vPrintf(const char *fmt, ...)
{

    char *bp = (char *)fmt;
    va_list ap;
    char c;
    char *p;
    int32 i = 0;
    uint64 u64BitVal;
    uint8 u8Digits;
    bool_t bLongValue;
    bool_t bPadWithZeros;
    va_start(ap, fmt);

    while ((c = *bp++)) {
        if (c != '%') {
            if (c == '\n'){
                vPutChar('\n');
                vPutChar('\r');
            } else {
                vPutChar(c);
            }
            continue;
        }

        /* read number of digits to display if specified */
        u8Digits = 0;
        bPadWithZeros = FALSE;
        while((c = *bp) >= '0' && (c=*bp) <= '9'){
            bp++;
            if(c == '0' && u8Digits == 0){
                bPadWithZeros = TRUE;
            }
            u8Digits *= 10;
            u8Digits += (c - '0');
        }
        c = *bp++;

        if(u8Digits < 1) bPadWithZeros = TRUE;

        /* see if its a long value */
        if(c == 'l'){
            bLongValue = TRUE;
            c = *bp++;
        } else {
            bLongValue = FALSE;
        }

        switch (c) {

        /* %d - show a decimal value */
        case 'd':
            vJPTNum2String(va_arg(ap, uint32), 10, u8Digits, bPadWithZeros, FALSE);
            break;

        /* %x - show a value in hex */
        case 'x':
            if(!bLongValue){
                vJPTNum2String(va_arg(ap, uint32), 16, u8Digits, bPadWithZeros, TRUE);
            } else {
                u64BitVal = va_arg(ap, uint64);
                vJPTNum2String((uint32)(u64BitVal >> 32), 16, u8Digits / 2, bPadWithZeros, TRUE);
                vJPTNum2String((uint32)(u64BitVal & 0xffffffff), 16, u8Digits / 2, TRUE, FALSE);
            }
            break;

        /* %b - show a value in binary */
        case 'b':
            vJPTNum2String(va_arg(ap, uint32), 2, u8Digits, bPadWithZeros, TRUE);
            break;

        /* %c - show a character */
        case 'c':
            vPutChar(va_arg(ap, int));
            break;

        case 'i':
            i = va_arg(ap, int32);
            if(i < 0){
                vPutChar('-');
                vJPTNum2String((~i)+1, 10,0, bPadWithZeros, FALSE);
            } else {
                vJPTNum2String(i, 10, u8Digits, bPadWithZeros, FALSE);
            }
            break;

        /* %s - show a string */
        case 's':
            p = va_arg(ap, char *);
            do {
                vPutChar(*p++);
            } while (*p);
            break;

        /* %% - show a % character */
        case '%':
            vPutChar('%');
            break;

        /* %something else not handled ! */
        default:
            vPutChar('?');
            break;

        }

    }

    va_end(ap);

}


/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME:       vNum2String
 *
 * DESCRIPTION:
 * Converts a number in a specified base to a string representation.
 *
 * PARAMETERS:  Name            RW  Usage
 *              u32Number       R   Number to be converted into a string
 *              u8Base          R   Base of number to be converted
 *              u8Digits        R   Minimum number of digits to display
 *                                  (adds leading zeros)
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vJPTNum2String(uint32 u32Number, uint8 u8Base, uint8 u8Digits, bool_t bPadWithZeros, bool_t bPrefixWithBase)
{

    char acBuffer[36] = {0};
    char *ptr1 = acBuffer + 35;

    /* ptr2 will point to the start of the numbers, not the preceding padding if any */
    char *ptr2 = ptr1;

    uint32 c;
    uint32 u32Length = 0;

    /* terminate the string */
    *--ptr1 = '\0';

    do {

        /* get next digit to display */
        c = u32Number % u8Base;

        /* figure out what ascii character to use */
        if (c < 10) {
            if(u32Number || bPadWithZeros || !u32Length){
                *--ptr1 = '0' + c;
                ptr2 = ptr1;
            } else {
                *--ptr1 = ' ';
            }
        } else {
            *--ptr1 = 'a' + (c - 10);
            ptr2 = ptr1;
        }

        /* next digit...*/
        u32Number /= u8Base;

        if(u8Digits) u8Digits--;
        u32Length++;
    } while (u32Number != 0 || u8Digits != 0);

    /*
     Now we prefix the base , ie, 0x if required. This goes at the start
     of the alphanumeric digits, not any preceding padding spaces. If we
     add a prefix to a space padded number, we add 2 more spaces to the start
     */
    if(bPrefixWithBase){

        switch(u8Base){

        case 2:
            *--ptr2 = 'b';
            break;

        case 16:
            *--ptr2 = 'x';
            break;
        default:
            *--ptr2 = '?';
            break;

        }

        *--ptr2 = '0';
        if(!bPadWithZeros){
            *--ptr1 = ' ';
            *--ptr1 = ' ';
        }

        if(ptr2 < ptr1) ptr1 = ptr2;

    }

    /* write completed string to output device */
    while (*ptr1){
        vPutChar(*ptr1);
        ptr1++;
    }

}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
