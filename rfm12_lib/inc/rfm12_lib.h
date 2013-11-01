/*
 * rfm12_lib.h
 *
 *  Created on: Nov 1, 2013
 *      Author: franz
 */

#include <stm32f4xx.h>

#ifndef RFM12_LIB_H_
#define RFM12_LIB_H_

#define RF12FREQ(freq)	((freq-430.0)/0.0025)

void RFM12_init_tx(void);
void RFM12_init_rx(void);
void rf12_setfreq(uint16_t freq);
void rf12_setbaud(uint16_t baud);
void rf12_setpower(unsigned char power, unsigned char mod);		// set transmission settings
void rf12_setbandwidth(unsigned char bandwidth, unsigned char gain, unsigned char drssi);	// set receiver settings
void rf12_txdata(unsigned char *data, unsigned char number);		// transmit number of bytes from array
void rf12_rxdata(uint16_t *data, unsigned char number);


#endif /* RFM12_LIB_H_ */
