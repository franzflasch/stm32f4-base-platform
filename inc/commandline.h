/*
 * commandline.h
 *
 *  Created on: Mar 23, 2013
 *      Author: franz
 */

#ifndef COMMANDLINE_H_
#define COMMANDLINE_H_

#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>

#define cmdMAX_INPUT_SIZE 80
#define configCOMMAND_INT_MAX_OUTPUT_SIZE 80

/*
 * Commandline Task:
 */
void prvUARTCommandConsoleTask( void *pvParameters );


#endif /* COMMANDLINE_H_ */
