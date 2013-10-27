/*
 * adc.h
 *
 *  Created on: Oct 26, 2013
 *      Author: franz
 */

#ifndef ADC_H_
#define ADC_H_

/* IRQ Callback to be installed */
typedef void (*ADC_call_back) (void *);

typedef struct ADC_WA_s
{
	ADC_call_back adcCallBack;	/* Cb to be installed */
	void *adcCbParam;			/* Parameter which is handed to the callback functoin */
}ADC_WA_t;

void ADC_A_installCB(void *pCb, void *pCbParam);
void ADC_A_dmaConfiguration(__IO uint16_t *uhADCxConvertedValue, uint16_t bufSize);
uint32_t ADC_A_getSamplingRate(void);

#endif /* ADC_H_ */
