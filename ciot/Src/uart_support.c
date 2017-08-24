/********************************************************************************/
/*!
	@file			uart_support.c
	@author         Nemui Trinomius (http://nemuisan.blog.bai.ne.jp)
    @version        11.00
    @date           2016.02.14
	@brief          Based on ST Microelectronics Sample Thanks!

    @section HISTORY
		2011.06.12	V1.00	Start Here.
		2011.09.14	V2.00	Expand fifo buffer uint8_t to uint16_t
		2012.01.31	V3.00	UART IRQ Routine moved from stm32f4xx_it.c.
		2014.05.01	V4.00	Fixed Suitable Interrupt-Preemption level.
		2014.06.28	V5.00	Adopted to STM32 HAL driver.
		2014.07.11	V6.00	Simplified some functions.
		2014.07.19	V7.00	Added Struct Clear on Init.
		2015.01.11	V8.00	Added buffered UART information.
		2015.09.18	V9.00	Fixed Wrong Expression.
		2016.01.10 V10.00	Added selectable receive procedure on polling mode.
		2016.02.14 V11.00	Fixed wrong interrupt handlings.
		2017.08.20 V11.10   Change conio_init and USARTx_IRQHandler by spiralray

    @section LICENSE
		BSD License. See Copyright.txt
*/
/********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "uart_support.h"
/* check header file version for fool proof */
#if __UART_SUPPORT_H!= 0x1110
#error "header file version is not correspond!"
#endif

/* Defines -------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
static USART_Buffer_t* pUSART_Buf;
USART_Buffer_t USARTx_Buf;

/* Function prototypes -------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**************************************************************************/
/*!
    Initialize UART.
    !!!! Notice !!!!
	Default code is capable one UART port only(for reduce flash memory usage).
	If you want multiple uart port,you should change source codes
    for multiple UART ports definitions.
*/
/**************************************************************************/
/* Initialize serial console */
void conio_init()
{

#if defined(UART_INTERRUPT_MODE)
	/* Enable the USART1 Interrupt */
	HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);
#endif

	/* Init Ring Buffer */
	pUSART_Buf = &USARTx_Buf;
	USARTx_Buf.RX_Tail = 0;
	USARTx_Buf.RX_Head = 0;
	USARTx_Buf.TX_Tail = 0;
	USARTx_Buf.TX_Head = 0;

	/* Enable UART Receive interrupts */
	UARTx->CR1 |= USART_CR1_RXNEIE;
}


/**************************************************************************/
/*!
    Check UART TX Buffer Empty.
*/
/**************************************************************************/
bool USART_TXBuffer_FreeSpace(USART_Buffer_t* USART_buf)
{
	/* Make copies to make sure that volatile access is specified. */
	unsigned int tempHead = (USART_buf->TX_Head + 1) & (UART_BUFSIZE-1);
	unsigned int tempTail = USART_buf->TX_Tail;

	/* There are data left in the buffer unless Head and Tail are equal. */
	return (tempHead != tempTail);
}

/**************************************************************************/
/*!
    Put Bytedata with Buffering.
*/
/**************************************************************************/
bool USART_TXBuffer_PutByte(USART_Buffer_t* USART_buf, uint8_t data)
{

	unsigned int tempTX_Head;
	bool TXBuffer_FreeSpace;

	TXBuffer_FreeSpace = USART_TXBuffer_FreeSpace(USART_buf);


	if(TXBuffer_FreeSpace)
	{
	  	tempTX_Head = USART_buf->TX_Head;

		__disable_irq();
	  	USART_buf->TX[tempTX_Head]= data;
		/* Advance buffer head. */
		USART_buf->TX_Head = (tempTX_Head + 1) & (UART_BUFSIZE-1);
		__enable_irq();

		/* Enable TXE interrupt. */
		UARTx->CR1 |= USART_CR1_TXEIE;
	}
	return TXBuffer_FreeSpace;
}

/**************************************************************************/
/*!
    Check UART RX Buffer Empty.
*/
/**************************************************************************/
bool USART_RXBufferData_Available(USART_Buffer_t* USART_buf)
{
	/* Make copies to make sure that volatile access is specified. */
	unsigned int tempHead = pUSART_Buf->RX_Head;
	unsigned int tempTail = pUSART_Buf->RX_Tail;

	/* There are data left in the buffer unless Head and Tail are equal. */
	return (tempHead != tempTail);
}

/**************************************************************************/
/*!
    Get Bytedata with Buffering.
*/
/**************************************************************************/
uint8_t USART_RXBuffer_GetByte(USART_Buffer_t* USART_buf)
{
	uint8_t ans;

	__disable_irq();
	ans = (pUSART_Buf->RX[pUSART_Buf->RX_Tail]);

	/* Advance buffer tail. */
	pUSART_Buf->RX_Tail = (pUSART_Buf->RX_Tail + 1) & (UART_BUFSIZE-1);

	__enable_irq();

	return ans;
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Send 1 character */
inline void putch(uint8_t data)
{
#if defined(UART_INTERRUPT_MODE)
	/* Interrupt Version */
	while(!USART_TXBuffer_FreeSpace(pUSART_Buf));
	USART_TXBuffer_PutByte(pUSART_Buf,data);
#else
	/* Polling version */
	while (!(UARTx->SR & USART_SR_TXE));
	UARTx->DR = data;
#endif
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Receive 1 character */
uint8_t getch(void)
{
#if defined(UART_INTERRUPT_MODE)
	if (USART_RXBufferData_Available(pUSART_Buf))  return USART_RXBuffer_GetByte(pUSART_Buf);
	else										   return false;
#else
	/* Polling version */
#if (UART_NOBLOCK_RECV == 1)
	if ((UARTx->SR & USART_SR_RXNE))			   return (uint8_t)(UARTx->DR);
	else										   return false;
#else
	while (!(UARTx->SR & USART_SR_RXNE));
	return (uint8_t)(UARTx->DR);
#endif
#endif
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Return 1 if key pressed */
uint8_t keypressed(void)
{
#if defined(UART_INTERRUPT_MODE)
	return (USART_RXBufferData_Available(pUSART_Buf));
#else
	return (UARTx->SR & USART_SR_RXNE);
#endif
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Send a string */
void cputs(char *s)
{
	while (*s)
    putch(*s++);
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Receive a string, with rudimentary line editing */
void cgets(char *s, int bufsize)
{
	char *p;
	int c;

	memset(s, 0, bufsize);

	p = s;

	for (p = s; p < s + bufsize-1;)
	{
		/* 20090521Nemui */
		do{
			c = getch();
		}while(c == false);
		/* 20090521Nemui */
		switch (c)
		{
			case '\r' :
			case '\n' :
				putch('\r');
				putch('\n');
				*p = '\n';
			return;

			case '\b' :
				if (p > s)
				{
				  *p-- = 0;
				  putch('\b');
				  putch(' ');
				  putch('\b');
				}
			break;

			default :
				putch(c);
				*p++ = c;
			break;
		}
	}
	return;
}

#if defined(UART_INTERRUPT_MODE)
/**************************************************************************/
/*!
    @brief	Handles UARTx global interrupt request.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void USARTx_IRQHandler(void)
{
    uint32_t IntStat = UARTx->ISR;

    if(IntStat & USART_ISR_RXNE)
    {
        /* Advance buffer head. */
        unsigned int tempRX_Head = ((&USARTx_Buf)->RX_Head + 1) & (UART_BUFSIZE-1);

        /* Check for overflow. */
        unsigned int tempRX_Tail = (&USARTx_Buf)->RX_Tail;
        uint8_t data =  UARTx->RDR;

        if (tempRX_Head == tempRX_Tail) {
            /* Overflow MAX size Situation */
            /* Disable the UART Receive interrupt */
            //UARTx->CR1 &= ~(USART_CR1_RXNEIE);
            //(&USARTx_Buf)->RX_Tail = 0;
            //(&USARTx_Buf)->RX_Head = 0;
        }else{
            (&USARTx_Buf)->RX[(&USARTx_Buf)->RX_Head] = data;
            (&USARTx_Buf)->RX_Head = tempRX_Head;
        }
    }

    if(IntStat & USART_ISR_TXE)
    {
        /* Check if all data is transmitted. */
        unsigned int tempTX_Tail = (&USARTx_Buf)->TX_Tail;
        if ((&USARTx_Buf)->TX_Head == tempTX_Tail){
            /* Overflow MAX size Situation */
            /* Disable the UART Transmit interrupt */
            UARTx->CR1 &= ~(USART_CR1_TXEIE);
        }else{
            /* Start transmitting. */
            uint8_t data = (&USARTx_Buf)->TX[(&USARTx_Buf)->TX_Tail];
            UARTx->TDR = data;

            /* Advance buffer tail. */
            (&USARTx_Buf)->TX_Tail = ((&USARTx_Buf)->TX_Tail + 1) & (UART_BUFSIZE-1);
        }
    }
}
#endif

/* End Of File ---------------------------------------------------------------*/
