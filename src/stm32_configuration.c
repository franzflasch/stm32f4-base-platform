#include <stm32_configuration.h>
#include <stm32f4xx_conf.h>

ErrorStatus HSEStartUpStatus;

void RCC_Configuration(void)
{
	//RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	//Periph-Clock:
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA |
							RCC_AHB1Periph_GPIOB |
							RCC_AHB1Periph_GPIOC |
							RCC_AHB1Periph_GPIOD |
							RCC_AHB1Periph_GPIOE |
							RCC_AHB1Periph_DMA1	|
							RCC_AHB1Periph_DMA2, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_SPI2, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG |
						   RCC_APB2Periph_ADC1 |
						   RCC_APB2Periph_ADC2 |
						   RCC_APB2Periph_ADC3, ENABLE);
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Configure USART2 Tx (PA.02) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/******************SPI GPIO Configuration **********************************/

	/* CS */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect SPI pins to AF5 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* SPI  MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/****************** END SPI GPIO Configuration **********************************/


	/*!< Configure SD_SPI_DETECT_PIN pin: SD Card detect pin */
	GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);

	/* PD13 (oranges LED), PD7(CLK), PD3(GND), PD1(VIO) */
	GPIO_InitStructure.GPIO_Pin = GREEN_LED | ORANGE_LED | RED_LED | BLUE_LED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);


	/***************** ADC Configuration*********************************************/
	GPIO_InitStructure.GPIO_Pin = ADC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(ADC_PORT, &GPIO_InitStructure);
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQ_DMA_STREAM;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//	/* Configure the SPI interrupt priority */
//	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
}


void USART_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

  	USART_InitStructure.USART_BaudRate = 115200;
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;
  	USART_InitStructure.USART_Parity = USART_Parity_No;
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	/* Configure USART1 */
	USART_Init(USART2, &USART_InitStructure);    
			 
	  /* Enable the USART1 */
	USART_Cmd(USART2, ENABLE);	
}

void SPI_Configuration(void)
{
	SPI_InitTypeDef  SPI_InitStructure;

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI2);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_CalculateCRC(SPI2, DISABLE);
	/* Enable the SPI peripheral */
	SPI_Cmd(SPI2, ENABLE);
}

void SPI_Configuration_FullSpeed(void)
{
	SPI_InitTypeDef  SPI_InitStructure;

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI2);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

	SPI_Init(SPI2, &SPI_InitStructure);

	/* Enable the SPI peripheral */
	SPI_Cmd(SPI2, ENABLE);
}

/**
  * @brief  ADC configuration
  * @note   This function Configure the ADC peripheral
            1) Enable peripheral clocks
            2) Configure ADC Channel 12 pin as analog input
            3) DMA2_Stream0 channel2 configuration
            4) Configure ADC1 Channel 12
            5) Configure ADC2 Channel 12
            6) Configure ADC3 Channel 12
  * @param  None
  * @retval None
  */
void ADC_Triple_Interleaved_Configuration(__IO uint32_t *pADC_TripleConvertedVale, uint32_t bufSize)
{
  DMA_InitTypeDef        DMA_InitStructure;
  ADC_InitTypeDef        ADC_InitStructure;
  ADC_CommonInitTypeDef  ADC_CommonInitStructure;

  DMA_DeInit(ADC_DMA_STREAM);
  ADC_DeInit();

  /* DMA2 Stream0 channel0 configuration **************************************/
  DMA_InitStructure.DMA_Channel = ADC_DMA_CHANNEL;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC_CDR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pADC_TripleConvertedVale;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = bufSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(ADC_DMA_STREAM, &DMA_InitStructure);

  /* DMA2_Stream0 enable */
  DMA_Cmd(ADC_DMA_STREAM, ENABLE);
  DMA_ITConfig(ADC_DMA_STREAM, DMA_IT_HT, ENABLE);

  /* ADC Common configuration *************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_Interl;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_2;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 regular channel 12 configuration ************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_3Cycles);
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* ADC2 regular channel 12 configuration ************************************/
  ADC_Init(ADC2, &ADC_InitStructure);
  /* ADC2 regular channel12 configuration */
  ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_3Cycles);

  /* ADC3 regular channel 12 configuration ************************************/
  ADC_Init(ADC3, &ADC_InitStructure);
  /* ADC3 regular channel12 configuration */
  ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_3Cycles);

  /* Enable DMA request after last transfer (multi-ADC mode) ******************/
  ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

  /* Enable ADC1 **************************************************************/
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC2 **************************************************************/
  ADC_Cmd(ADC2, ENABLE);

  /* Enable ADC3 **************************************************************/
  ADC_Cmd(ADC3, ENABLE);
}


