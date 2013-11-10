/*
 * tim.c
 *
 *  Created on: Nov 10, 2013
 *      Author: franz
 */

#include <tim.h>
#include <stm32f4xx.h>



void TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 50000 - 1; // 1 MHz down to 1 KHz (1 ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 20 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/* TIM IT enable */
	//TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

