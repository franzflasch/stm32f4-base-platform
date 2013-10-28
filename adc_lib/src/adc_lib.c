/*
 * adc_lib.c
 *
 *  Created on: Oct 26, 2013
 *      Author: franz
 */

#include <adc_lib.h>

#include <FreeRTOS.h>
#include <task.h>

void adcCallback(void* pParam, void* pParam2)
{
	adcWorkArea_t *pWa = pParam;

	pWa->frameCountIn = *(uint16_t *)pParam2;
	/* Resume the FreeRTOS task */
	//xTaskResumeFromISR(pWa->optionalParameter);
	adcDoProcessing(pWa);
}

void adcInit(adcWorkArea_t *pWa, uint16_t *pPingPongBuf, uint16_t bufSize, uint32_t samplingRate, void *pOptParameter)
{
	/* bufSize has to be at least 2 otherwise the pingPong is sensless*/
	if(pPingPongBuf != NULL && bufSize >= 2)
	{
		pWa->pAdcBufferA = &pPingPongBuf[0];
		pWa->pAdcBufferB = &pPingPongBuf[bufSize/2];
		pWa->bufSize = bufSize;
		pWa->pActBuffer = pWa->pAdcBufferB;
		pWa->bufOverWriteCnt = 0;
		pWa->adcSamplingRate = samplingRate;
		pWa->frameCountIn = 0;
		pWa->frameCountAct = 0;
		pWa->optionalParameter = pOptParameter;
	}
}

void adcDoProcessing(adcWorkArea_t *pWa)
{
	int i = 0;

	if((pWa->frameCountIn%2) == 1)
	{
		pWa->pActBuffer = pWa->pAdcBufferA;
	}
	else
	{
		pWa->pActBuffer = pWa->pAdcBufferB;
	}

	for(i=0;i<(pWa->bufSize/2);i++)
	{
		/*TODO Just for timing test, put here real code */
		pWa->pActBuffer[i] = i;
	}

	if(pWa->frameCountIn-pWa->frameCountAct > 1)
	{
		pWa->bufOverWriteCnt++;
		//USART_debug(USART2, "!!!!%d!\n\r", pWa->frameCountAct-pWa->frameCountIn);
	}
	pWa->frameCountAct++;
}

