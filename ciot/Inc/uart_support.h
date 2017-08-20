/********************************************************************************/
/*!
	@file			uart_support.h
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
#ifndef __UART_SUPPORT_H
#define __UART_SUPPORT_H	0x1110

#ifdef __cplusplus
 extern "C" {
#endif

/* General Inclusion */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>

/* uC Specific Inclusion */
#include "stm32l0xx.h"
#include "stm32l0xx_hal_uart.h"

/* USART Definition */
#define UART_BUFSIZE			512			/* Buffer size MUST Takes power of 2(64,128,256,512...) */
#define UART_INTERRUPT_MODE					/* If u want polling mode, uncomment this */
#define UART_NOBLOCK_RECV		0			/* Set 1 to non-blocking receive on polling mode */


#define UART_DEFAULT_NUM        1
#define UARTx                  USART1
#define USARTx_IRQn            USART1_IRQn
#define USARTx_IRQHandler      USART1_IRQHandler

/* General Definition */
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Funcion Prototypes */
extern void conio_init();
extern void putch(uint8_t c);
extern uint8_t getch(void);
extern uint8_t keypressed(void);
extern void cputs(char *s);
extern void cgets(char *s, int bufsize);

/* Structs of UART(This is Based on AVRX uC Sample!!!) */
/* @brief USART transmit and receive ring buffer. */
typedef struct USART_Buffer
{
	/* @brief Receive buffer. */
	volatile uint8_t RX[UART_BUFSIZE];
	/* @brief Transmit buffer. */
	volatile uint8_t TX[UART_BUFSIZE];
	/* @brief Receive buffer head. */
	volatile unsigned int RX_Head;
	/* @brief Receive buffer tail. */
	volatile unsigned int RX_Tail;
	/* @brief Transmit buffer head. */
	volatile unsigned int TX_Head;
	/* @brief Transmit buffer tail. */
	volatile unsigned int TX_Tail;
} USART_Buffer_t;

/* Externs */
extern USART_Buffer_t USARTx_Buf;


#ifdef __cplusplus
}
#endif

#endif	/* __UART_SUPPORT_H */
