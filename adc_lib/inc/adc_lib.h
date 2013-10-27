/*
 * adc_lib.h
 *
 *  Created on: Oct 26, 2013
 *      Author: franz
 */

#include <FreeRTOS.h>
#include <task.h>

#define ADC_BUF_SIZE 512

typedef struct adcWorkArea_s
{
	uint16_t *pActBuffer;		/* Pointer to the currently active buffer */
	uint16_t *pAdcBufferA;		/* Pointer to the adc ping membuffer */
	uint16_t *pAdcBufferB;		/* Pointer to the adc pong membuffer */
	uint16_t bufOverWriteCnt;	/* Counter which indicates that we could not calc'd the buffer in time */
	uint32_t adcSamplingRate;	/* Used sampling rate */

}adcWorkArea_t;

void adcCallback(void* adcTaskHandle);
void adcInit(adcWorkArea_t *pWa, uint16_t *pPingPongBuf, uint16_t bufSize, uint32_t samplingRate);
void adcDoProcessing(adcWorkArea_t *pWa);
