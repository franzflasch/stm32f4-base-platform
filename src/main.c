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

#include <stdio.h>
#include <string.h>

static void BlinkyTask( void *pvParameters );

int main()
{
	RCC_Configuration();
	NVIC_Configuration();
	LED_Configuration();

	xTaskCreate( BlinkyTask, ( signed char * ) "BlinkyTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL );

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	while(1);
}

static void BlinkyTask( void *pvParameters )
{
	portTickType xNextWakeTime;
	xNextWakeTime = xTaskGetTickCount();
	while(1)
	{
		GPIOD->ODR ^= GREEN_LED;
		vTaskDelayUntil( &xNextWakeTime, 1000);
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

