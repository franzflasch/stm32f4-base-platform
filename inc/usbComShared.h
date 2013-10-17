/*
 * usbComShared.h
 *
 *  Created on: Oct 14, 2013
 *      Author: franz
 *
 * Shared header:
 * Linux-Host <-> Microcontroller
 */

#ifndef USBCOMSHARED_H_
#define USBCOMSHARED_H_


/* Test message header */
typedef struct _testUSBComMsg_
{
	uint8_t ctrlData;
	uint16_t data;
}testUSBComMsg;


#endif /* USBCOMSHARED_H_ */
