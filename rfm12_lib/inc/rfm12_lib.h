/*
 * rfm12_lib.h
 *
 *  Created on: Nov 1, 2013
 *      Author: franz
 */

#include <stm32f4xx.h>

#ifndef RFM12_LIB_H_
#define RFM12_LIB_H_

#define RFM12_WAIT_SDO_HIGH() while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2)!=1)

/* macro for calculating frequency value out of frequency in MHz */
#define RFM12FREQ(freq)	((freq-430.0)/0.0025)

uint16_t RFM12_trans(uint16_t value);

void RFM12_init(void);											// initialize module
void RFM12_setfreq(unsigned short freq);							// set center frequency
void RFM12_setbaud(unsigned short baud);							// set baudrate
void RFM12_setpower(unsigned char power, unsigned char mod);		// set transmission settings
void RFM12_setbandwidth(unsigned char bandwidth, unsigned char gain, unsigned char drssi);	// set receiver settings
void RFM12_txdata(unsigned char *data, unsigned char number);		// transmit number of bytes from array
void RFM12_rxdata(unsigned char *data, unsigned char number);		// receive number of bytes into array
void RFM12_ready(void);											// wait until FIFO ready (to transmit/read data)

#endif /* RFM12_LIB_H_ */
