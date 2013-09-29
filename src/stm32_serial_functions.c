/*
 * stm32_serial_functions.c
 *
 *  Created on: Mar 20, 2013
 *      Author: franz
 */

#include <stm32_serial_functions.h>


void USART_send(USART_TypeDef *usartNr, const char *msgPtr)
{
	while(*msgPtr)
	{
		USART_SendData(usartNr, *msgPtr++);
		while (USART_GetFlagStatus(usartNr, USART_FLAG_TC) != SET);
	}
}


void USART_sendSuspend(USART_TypeDef *usartNr, const char *msgPtr, xTaskHandle taskHandle)
{
	while(*msgPtr)
	{
		/* Enable Interrupt for USART TX. USART2 MUST NOT be used by another function or task!!!*/
		USART_ITConfig(usartNr, USART_IT_TC, ENABLE);
		USART_SendData(usartNr, *msgPtr++);
		vTaskSuspend( taskHandle );
	}
}


void USART_debug(USART_TypeDef *usartNr, const char *s,...)
{
    va_list va;
    va_start(va,s);

    char buffer[USART_MAX_STR_LEN];

    //sprintf(max,"error: "); vsprintf(max,s,va); sprintf(max,"\n");

    va_end(va);
    vsnprintf(buffer,USART_MAX_STR_LEN, s, va);
	USART_send(usartNr, buffer);
}


