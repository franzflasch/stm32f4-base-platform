/*
 * adc_lib.h
 *
 *  Created on: Oct 26, 2013
 *      Author: franz
 */

#include <FreeRTOS.h>
#include <task.h>

#define ADC_HOST_BUF_FIRST_HALF_READY	0x0001
#define ADC_HOST_BUF_SCND_HALF_READY	0x0002
#define ADC_HOST_BUF_OVF				0x0004

typedef struct adcWorkArea_s
{
	uint16_t *pActBuffer;		/* Pointer to the currently active buffer */
	uint16_t *pAdcBufferA;		/* Pointer to the adc ping membuffer */
	uint16_t *pAdcBufferB;		/* Pointer to the adc pong membuffer */
	uint16_t bufSize;			/* PingPong buffer size */
	uint16_t bufOverWriteCnt;	/* Counter which indicates that we could not calc'd the buffer in time */
	uint32_t adcSamplingRate;	/* Used sampling rate */
	uint16_t frameCountIn;		/* Current frameCount from input stream */
	uint16_t frameCountAct;		/* Current frameCount from the actual stream */
	void *optionalParameter;	/* Optional parameter - could be used e.g for freertos task handle */
	uint16_t *bufferToHost;		/* Buffer to copy to host */
	uint16_t hostBufState;		/* Status of the host buffer use enum */
	uint16_t bufSizeHost;		/* Size of the Host buffer */
}adcWorkArea_t;

void adcCallback(void* adcTaskHandle, void* pAdcWa);
void adcInit(adcWorkArea_t *pWa, uint16_t *pPingPongBuf,
			 uint16_t bufSize, uint32_t samplingRate,
			 void *pOptParameter, uint16_t *hostBuf,
			 uint16_t bufSizeHost);
void adcDoProcessing(adcWorkArea_t *pWa);
