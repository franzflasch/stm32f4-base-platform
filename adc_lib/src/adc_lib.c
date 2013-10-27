/*
 * adc_lib.c
 *
 *  Created on: Oct 26, 2013
 *      Author: franz
 */

#include <adc_lib.h>

#include <FreeRTOS.h>
#include <task.h>

void adcCallback(void* adcTaskHandle)
{
	/* Resume the FreeRTOS task */
	xTaskResumeFromISR((xTaskHandle)adcTaskHandle);
}

void adcInit(adcWorkArea_t *pWa, uint16_t *pPingPongBuf, uint16_t bufSize, uint32_t samplingRate)
{
	/* bufSize has to be at least 2 otherwise the pingPong is sensless*/
	if(pPingPongBuf != NULL && bufSize >= 2)
	{
		pWa->pAdcBufferA = &pPingPongBuf[0];
		pWa->pAdcBufferB = &pPingPongBuf[bufSize/2];
		pWa->pActBuffer = pWa->pAdcBufferB;
		pWa->bufOverWriteCnt = 0;
		pWa->adcSamplingRate = samplingRate;
	}
}

void adcDoProcessing(adcWorkArea_t *pWa)
{

}

