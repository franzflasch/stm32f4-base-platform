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

/*
 * Commandline Task:
 */
void prvUARTCommandConsoleTask( void *pvParameters );


#endif /* COMMANDLINE_H_ */
