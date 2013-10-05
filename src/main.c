#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <stm32_configuration.h>
#include <stm32_serial_functions.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
#include <croutine.h>

#include <commands.h>
#include <main.h>

#include <stdio.h>
#include <string.h>

#include <ff.h>
#include <diskio.h>
#include <usbd_core.h>
#include <usbd_hid_core.h>
#include <usbd_desc.h>
#include <usbd_usr.h>

#define ADC_BUFSIZE 1024


__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

uint8_t SPIRxBuffer[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
//FIXME: Don't know why the TX buffersize has to 9 here? The DMA does not send the
//		 first byte. It begins with the second one. Maybe something is not correctly setup,
//		 or maybe I have to look in the errata sheet of the STM32F4.
uint8_t SPITxBuffer[] = {0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41};

/* ADC Array to store values */
__IO uint32_t aADCTripleConvertedValue[ADC_BUFSIZE];
uint32_t aADCTripleConvertedValue_cached[ADC_BUFSIZE];
volatile uint8_t adcStatus = 0;

static void SD_TimerTask( void *pvParameters );
static void SD_CardTestTask( void *pvParameters );
static void BlinkyTask( void *pvParameters );
static void ADCTask( void *pvParameters );
static void CANTestTask( void *pvParameters );


int main()
{
	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	USART_Configuration();
	SPI_Configuration();
	//ADC_Triple_Interleaved_Configuration(&aADCTripleConvertedValue[0], ADC_BUFSIZE);

	/* Disable MMC/SD SPI CS Pin (active low) */
	MMC_CS_HIGH();

	FreeRTOS_CLIRegisterCommand( &xHelloCLI );
	FreeRTOS_CLIRegisterCommand( &xSimpleParamCLI );
	FreeRTOS_CLIRegisterCommand( &xStoreInFlash );
	FreeRTOS_CLIRegisterCommand( &xgetFromFlash );

	usartControl.xUsartRxQueue = xQueueCreate( 1, sizeof( char * ) );

	//xTaskCreate( SD_TimerTask, ( signed char * ) "SD_Timer_Task", configMINIMAL_STACK_SIZE, NULL, 4, NULL );
	//xTaskCreate( SD_CardTestTask, ( signed char * ) "SDCARD Test Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
	xTaskCreate( BlinkyTask, ( signed char * ) "BlinkyTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
	//xTaskCreate( ADCTask, ( signed char * ) "ADCTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
	//xTaskCreate( CANTestTask, ( signed char * ) "CANTestTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
	xTaskCreate( prvUARTCommandConsoleTask, ( signed char * ) "CLI", configMINIMAL_STACK_SIZE, usartControl.xUsartRxQueue, 4, &usartControl.commandlineHandle );

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	while(1);
}

static void SD_TimerTask( void *pvParameters )
{
	portTickType xNextWakeTime;

	xNextWakeTime = xTaskGetTickCount();

	while(1)
	{
		GPIOD->ODR ^= RED_LED;
		disk_timerproc();
		vTaskDelayUntil( &xNextWakeTime, 10);
	}
}


static void SD_CardTestTask( void *pvParameters )
{
	portTickType xNextWakeTime;

	/* FATFS Variables*/
	FRESULT res;
	FILINFO fno;
	FIL fil;
	DIR dir;
	FATFS fs32;
	char* path;

	memset(&fs32, 0, sizeof(FATFS));

	res = f_mount(0, &fs32);

	if (res != FR_OK)
		USART_debug(DEBUG_USART, "res = %d f_mount\n\r", res);

	memset(&fil, 0, sizeof(FIL));

	res = f_open(&fil, "MESSAGE.TXT", FA_READ);

	if (res != FR_OK)
		USART_debug(DEBUG_USART,"res = %d f_open MESSAGE.TXT\n\r", res);

	if (res == FR_OK)
	{
		UINT Total = 0;

		while(1)
		{
			BYTE Buffer[512];
			UINT BytesRead;
			UINT i;

			res = f_read(&fil, Buffer, sizeof(Buffer), &BytesRead);

			if (res != FR_OK)
				USART_debug(DEBUG_USART,"res = %d f_read MESSAGE.TXT\n\r", res);

			if (res != FR_OK)
				break;

			Total += BytesRead;

			for(i=0; i<BytesRead; i++)
				putchar(Buffer[i]);

			if (BytesRead < sizeof(Buffer))
				break;
		}

		res = f_close(&fil); // MESSAGE.TXT

		if (res != FR_OK)
			USART_debug(DEBUG_USART,"res = %d f_close MESSAGE.TXT\n\r", res);
	}

	res = f_open(&fil, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);

	if (res != FR_OK)
		USART_debug(DEBUG_USART,"res = %d f_open test.txt\n\r", res);
	else
	{
		/* Move to end of the file to append data */
		f_lseek(&fil, f_size(&fil));
		int i = 0;
		for(i=0;i<100;i++)
		{
			f_printf(&fil, "Test: Test test test dsadasdsa %d\n\r", 3);
		}

		res = f_close(&fil);

		if (res != FR_OK)
			USART_debug(DEBUG_USART,"res = %d f_close test.TX\n\r", res);
	}

	xNextWakeTime = xTaskGetTickCount();

	while(1)
	{
		//GPIOD->ODR ^= GPIO_Pin_12;
		vTaskDelayUntil( &xNextWakeTime, 1000);
	}
}


static void BlinkyTask( void *pvParameters )
{
	portTickType xNextWakeTime;
	uint8_t testState = 0;

	uint8_t i = 0;

	static uint8_t testBuffer[3] =
	{
			1,
			2,
			3
	};

	USBD_Init(&USB_OTG_dev,
	#ifdef USE_USB_OTG_HS
			USB_OTG_HS_CORE_ID,
	#else
			USB_OTG_FS_CORE_ID,
	#endif
			&USR_desc,
			&USBD_HID_cb,
			&USR_cb);

	xNextWakeTime = xTaskGetTickCount();
	while(1)
	{
		if(USBD_STATUS == 1)
		{
			/* We are connected send one status message*/
			if(testState == 0)
			{
				USART_debug(USART2, "Connected!\n\r");
				testState = 1;
			}
			DCD_EP_Tx(&USB_OTG_dev, 0x81, &testBuffer[0], 1);
		}
		else
		{
			if(testState == 1)
			{
				USART_debug(USART2, "Disconnected!\n\r");
				testState = 0;
			}
		}
		GPIOD->ODR ^= GREEN_LED;
		vTaskDelayUntil( &xNextWakeTime, 1000);
	}
}

static void ADCTask( void *pvParameters )
{
	uint32_t i = 0;
	uint32_t temp = 0;
	portTickType xNextWakeTime;

	xNextWakeTime = xTaskGetTickCount();

	/* Start ADC Software Conversion */
	ADC_SoftwareStartConv(ADC1);

	while(1)
	{
		if(adcStatus >=2 )
		{
			for(i=0;i<ADC_BUFSIZE;i++)
			{
				temp = ((aADCTripleConvertedValue_cached[i] & 0xFFF0000) >> 4) | (aADCTripleConvertedValue_cached[i] & 0x0000FFF);
				USART_debug(USART2, "%d\n",temp);
			}
			adcStatus = 0;
		}
		/* Start ADC1 Software Conversion */
		//USART_debug(USART2, "test\n\r");
		//USART_debug(USART2,"%d\n\r",((RCC->CFGR & RCC_CFGR_HPRE) >> 4));
		//USART_debug(USART2, "%d,%d,%d\n\r", aADCTripleConvertedValue[0], aADCTripleConvertedValue[1], aADCTripleConvertedValue[2]);
		vTaskDelayUntil( &xNextWakeTime, 200);
	}
}


static void CANTestTask( void *pvParameters )
{
	portTickType xNextWakeTime;
	xNextWakeTime = xTaskGetTickCount();

	float result = 0.1;
	CanTxMsg TxMessage = {0};

	/* Transmit Structure preparation */
	  /* transmit 1 message */
	TxMessage.StdId = 0x100;
	TxMessage.ExtId = 0x01;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 8;
    TxMessage.Data[0] = 97;
    TxMessage.Data[1] = 98;
    TxMessage.Data[2] = 99;
    TxMessage.Data[3] = 100;
    TxMessage.Data[4] = 101;
    TxMessage.Data[5] = 102;
    TxMessage.Data[7] = 103;
	CAN_Transmit(CAN1, &TxMessage);

	while(1)
	{
		result += 0.1;
		//printf("Hello! I am STM32F3 Discovery!\n\r");
//	    while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
//	    str[0] = USART_ReceiveData(USART2);
		//str[0] = getchar();
		printf("CAN\n");
        CAN_Transmit(CAN1, &TxMessage);

		GPIOE->ODR ^= GPIO_Pin_8;
		//GPIO_SetBits(GPIOE, GPIO_Pin_9);
		vTaskDelayUntil( &xNextWakeTime, 500 );
	}
}


void CAN_sendBuffer(uint8_t *msgBuffer)
{
	CanTxMsg TxMessage = {0};

	/* Transmit Structure preparation */
	/* transmit 1 message */
	TxMessage.StdId = 0x100;
	TxMessage.ExtId = 0x01;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 8;

	memcpy(TxMessage.Data, msgBuffer, 8);

	CAN_Transmit(CAN1, &TxMessage);
}


void USART2_IRQHandler (void)
{
	static int8_t cRxedChar;
	portBASE_TYPE xYieldRequired = pdFALSE;

	if(USART_GetITStatus(USART2, USART_IT_TC) == SET)
	{
		USART_ITConfig(USART2, USART_IT_TC, DISABLE);
	     // Resume the suspended task.
	     xYieldRequired = xTaskResumeFromISR( usartControl.commandlineHandle );
	}
	else if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
    	cRxedChar = USART_ReceiveData(USART2);
    	xQueueSendFromISR( usartControl.xUsartRxQueue, &cRxedChar, &xYieldRequired );
    	xTaskResumeFromISR( usartControl.commandlineHandle );
    }

	USART_ClearITPendingBit(USART2, USART_IT_TC);
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);

    if( xYieldRequired == pdTRUE )
    {
        // We should switch context so the ISR returns to a different task.
        // NOTE:  How this is done depends on the port you are using.  Check
        // the documentation and examples for your port.
   	 taskYIELD();
    }
}


void DMA2_Stream0_IRQHandler(void)
{
	static uint8_t currentState = 0;
	DMA_ClearITPendingBit( ADC_DMA_STREAM, DMA_IT_HTIF0);

	currentState++;

	/* Synchronize with the print task */
	if(!(((currentState % 2) == 1) && adcStatus == 0))
	{
		if(adcStatus < 2)
		{
			/* Copy the Pingbuffer */
			if((currentState % 2) == 0)
			{
				memcpy(&aADCTripleConvertedValue_cached, &aADCTripleConvertedValue, sizeof(aADCTripleConvertedValue)/2);
				adcStatus++;
			}
			/* Copy the Pongbuffer */
			if((currentState % 2) == 1)
			{
				memcpy(&aADCTripleConvertedValue_cached[ADC_BUFSIZE/2], &aADCTripleConvertedValue, sizeof(aADCTripleConvertedValue)/2);
				adcStatus++;
			}
		}
	}
	GPIOD->ODR ^= ORANGE_LED;
}





void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line5) != RESET)
  {
	/* Toggle LED1 */
	GPIO_SetBits(GPIOD, GPIO_Pin_13);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI2);
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_Cmd(SPI2, ENABLE);

    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line5);
  }
}

void SPI2_IRQHandler(void)
{
  /* SPI in Slave Receiver mode--------------------------------------- */
  if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) == SET)
  {
	printf("%d\n\r", SPI_I2S_ReceiveData(SPI2));
  }
}

void DMA1_Stream3_IRQHandler(void)
{
	uint8_t i = 0;

	GPIO_SetBits(GPIOD, GPIO_Pin_14);
	printf("RXBuf:");
	for(i = 0;i<sizeof(SPIRxBuffer);i++)
	{
		printf(" %0x", SPIRxBuffer[i]);
	}
	printf("\n");

	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);

	CAN_sendBuffer(SPIRxBuffer);
}


void CAN1_RX0_IRQHandler(void)
{
	int i = 0;
	CanRxMsg RxMessage = {0};
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	printf("Got Message:");

	for(i=0;i<8;i++)
	{
		printf(" %d", RxMessage.Data[i]);
	}
	printf("\n");

	if ((RxMessage.StdId == 0x00))
	{
	//KeyNumber = RxMessage.Data[0];
	}
}


void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{

	/* This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amout of FreeRTOS heap that
	remains unallocated. */
	for(;;)
	{
		//FUCKING CoRoutine!! needed 3 hours of debugging to find this
		//vCoRoutineSchedule();
	}
}


void vApplicationTickHook( void )
{


}

