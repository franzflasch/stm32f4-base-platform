/*
 * main.h
 *
 *  Created on: Mar 23, 2013
 *      Author: franz
 */

#ifndef MAIN_H_
#define MAIN_H_

typedef struct _usartCtrl_
{
	xTaskHandle commandlineHandle;
	xQueueHandle xUsartRxQueue;
} usartCtrl;


xTaskHandle adcTaskHandle;

/*Global System Variables*/
usartCtrl usartControl;

#endif /* MAIN_H_ */
