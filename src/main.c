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

#include <usbd_core.h>
#include <usbd_hid_core.h>
#include <usbd_desc.h>
#include <usbd_usr.h>

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

static void TestTask( void *pvParameters );
static void UsbComTask( void *pvParameters );

int main()
{
	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	USART_Configuration();

	FreeRTOS_CLIRegisterCommand( &xHelloCLI );
	FreeRTOS_CLIRegisterCommand( &xSimpleParamCLI );
	FreeRTOS_CLIRegisterCommand( &xStoreInFlash );
	FreeRTOS_CLIRegisterCommand( &xgetFromFlash );

	usartControl.xUsartRxQueue = xQueueCreate( 1, sizeof( char * ) );

	xTaskCreate( TestTask, ( signed char * ) "TestTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
	xTaskCreate( UsbComTask, ( signed char * ) "BlinkyTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
	xTaskCreate( prvUARTCommandConsoleTask, ( signed char * ) "CLI", configMINIMAL_STACK_SIZE, usartControl.xUsartRxQueue, 4, &usartControl.commandlineHandle );

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	while(1);
}


static void TestTask( void *pvParameters )
{
	portTickType xNextWakeTime;


	xNextWakeTime = xTaskGetTickCount();
	while(1)
	{
		GPIOD->ODR ^= RED_LED;
		vTaskDelayUntil( &xNextWakeTime, 50);
	}

}




static void UsbComTask( void *pvParameters )
{
	portTickType xNextWakeTime;
	uint8_t i = 0;
	uint8_t USB_connectState = 0;

	/* Declaration of the receive buffer.
	 * only the 2nd dimension will be declared via the HID_IN_PACKET define
	 * because the first one involves further changes in the USB-device descriptor.
	 * Maybe this can be automated later.
	 * */
	uint8_t USB_RxBufList[2][HID_OUT_PACKET];

	static uint8_t testBuffer[HID_IN_PACKET] =
	{
			1,
			2,
			3,
			4,
			5,
			6,
			7,
			8
	};

	USBD_Init(&USB_OTG_dev,
	#ifdef USE_USB_OTG_HS
			USB_OTG_HS_CORE_ID,
	#else
			USB_OTG_FS_CORE_ID,
	#endif
			&USR_desc,
			&USBD_HID_cb,
			&USR_cb,
			&USB_RxBufList[0][0]);

	xNextWakeTime = xTaskGetTickCount();
	while(1)
	{
		if(USBD_STATUS == 1)
		{
			/* We are connected send one status message*/
			if(USB_connectState == 0)
			{
				USART_debug(USART2, "Connected!\n\r");
				USB_connectState = 1;
			}

			/* Only update TX fifo if there is no other message pending */
			if(!(USB_OTG_dev.usbUsrDevStatus & USB_USR_MSG_PENDING))
			{
				DCD_EP_Tx(&USB_OTG_dev, HID_IN_EP, &testBuffer[0], 8);
			}
			if((USB_OTG_dev.usbUsrDevStatus & USB_USR_RX_MSG_READY))
			{
				USART_debug(USART2, "RxMessage:");
				for(i=0; i<HID_OUT_PACKET; i++)
				{
					USART_debug(USART2, "%d ", USB_RxBufList[0][i]);
				}
				for(i=0; i<HID_OUT_PACKET; i++)
				{
					USART_debug(USART2, "%d ", USB_RxBufList[1][i]);
				}
				/* Clear the MSG_READY flag */
				USB_OTG_dev.usbUsrDevStatus &= ~USB_USR_RX_MSG_READY;
				USART_debug(USART2, "\n\r");
			}
		}
		else
		{
			if(USB_connectState == 1)
			{
				USART_debug(USART2, "Disconnected!\n\r");
				USB_connectState = 0;
			}
		}
		GPIOD->ODR ^= GREEN_LED;
		vTaskDelayUntil( &xNextWakeTime, 1000);
	}
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

