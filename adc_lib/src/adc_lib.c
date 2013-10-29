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
	//adcDoProcessing(pWa);
}

void adcInit(adcWorkArea_t *pWa, uint16_t *pPingPongBuf,
			 uint16_t bufSize, uint32_t samplingRate,
			 void *pOptParameter, uint16_t *hostBuf,
			 uint16_t bufSizeHost)
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
		pWa->bufSizeHost = bufSizeHost;
		pWa->bufferToHost = hostBuf;
	}
}

void adcDoProcessing(adcWorkArea_t *pWa)
{
	int i = 0;
	static int hostBufCnt = 0;

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

	/* Copy to host buffer */
	memcpy(&pWa->bufferToHost[hostBufCnt],pWa->pActBuffer,2*(pWa->bufSize/2));
	hostBufCnt += pWa->bufSize/2;


	/* We didnt manage to store the last buffer */
	if((pWa->hostBufState & ADC_HOST_BUF_FIRST_HALF_READY) ||
	   (pWa->hostBufState & ADC_HOST_BUF_SCND_HALF_READY))
	{
		pWa->hostBufState |= ADC_HOST_BUF_OVF;
	}

	if(hostBufCnt==(pWa->bufSizeHost/2))
	{
		pWa->hostBufState |= ADC_HOST_BUF_FIRST_HALF_READY;
	}
	else if(hostBufCnt==pWa->bufSizeHost)
	{
		pWa->hostBufState |= ADC_HOST_BUF_SCND_HALF_READY;
	}

	if(hostBufCnt >= pWa->bufSizeHost)
	{
		hostBufCnt = 0;
	}

	if(pWa->frameCountIn-pWa->frameCountAct > 1)
	{
		pWa->bufOverWriteCnt++;
		//USART_debug(USART2, "!!!!%d!\n\r", pWa->frameCountAct-pWa->frameCountIn);
	}
	pWa->frameCountAct++;
}

