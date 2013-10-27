/*
 * adc.c
 *
 *  Created on: Oct 26, 2013
 *      Author: franz
 */

#include <stm32f4xx.h>
#include <stm32_configuration.h>
#include <adc.h>

#define ADC_A_CYCLETIME 3
#define ADC_A_RESOLUTION 12

/* Get the system clock */
extern uint32_t SystemCoreClock;

/* Global ADC Workarea structure
 * unfortunately this is needed for the IRQ */
ADC_WA_t ADC_wa;


void ADC_A_installCB(void *pCb, void *pCbParam)
{
	ADC_wa.adcCallBack = pCb;
	ADC_wa.adcCbParam = pCbParam;
}

void ADC_A_dmaConfiguration(__IO uint16_t *uhADCxConvertedValue, uint16_t bufSize)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADCx, DMA and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(ADC_A_DMA_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(ADC_A_CHANNEL_GPIO_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(ADC_A_CLK, ENABLE);

  /* DMA2 Stream0 channel2 configuration **************************************/
  DMA_InitStructure.DMA_Channel = ADC_A_DMA_CHANNELx;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC_A_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uhADCxConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = bufSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(ADC_A_DMA_STREAMx, &DMA_InitStructure);
  DMA_Cmd(ADC_A_DMA_STREAMx, ENABLE);

  /* DMA-IRQ enable */
  DMA_ITConfig(ADC_A_DMA_STREAMx, DMA_IT_TC, ENABLE);

  /* Configure ADC3 Channel7 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = ADC_A_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(ADC_A_GPIO_PORT, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/

  //INFO: When changing this also change the define ADC_A_RESOLUTION otherwise the sampling time calculation is not correct!
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC_A, &ADC_InitStructure);

  /* ADC3 regular channel7 configuration *************************************/

  //INFO: When changing this also change the define ADC_A_CYCLETIME otherwise the sampling time calculation is not correct!
  ADC_RegularChannelConfig(ADC_A, ADC_A_CHANNEL, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC_A, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC_A, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC_A, ENABLE);
}

uint32_t ADC_A_getSamplingRate(void)
{
	/* Just return the system clock */
	uint32_t APB2_Clock;
	uint32_t ADC_Clock;

	/* Calc the sampling rate */
	/*
	 * APB2 = SystemClock/2
	 * ADC clock = APB2/2
	 * Sample Time = (ADC_SampleTime_Cycles+ADC_Resolution)/ADC_Clock
	 */
	APB2_Clock = SystemCoreClock/2;
	ADC_Clock = APB2_Clock/2;

	return ADC_Clock/(ADC_A_CYCLETIME+ADC_A_RESOLUTION);
}

void ADC_A_DMA_Stream_IRQHandler(void)
{
	DMA_ClearITPendingBit( ADC_A_DMA_STREAMx, DMA_IT_TCIF0);

	/* Call the CB function */
	ADC_wa.adcCallBack(ADC_wa.adcCbParam);

	GPIOD->ODR ^= BLUE_LED;
}
