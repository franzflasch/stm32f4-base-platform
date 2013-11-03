/*
 * rfm12_lib.c
 *
 *  Created on: Nov 1, 2013
 *      Author: franz
 */

#include <rfm12_lib.h>
#include <spi.h>
#include <stm32_configuration.h>

uint16_t rf12_trans(uint16_t value)
{
	uint16_t retVal = 0;

	RFM12_CS_LOW();

	//Delay(1);

	/* Send a Byte through the SPI peripheral */
	SPI_I2S_SendData(RFM12_SPI, value);

	/* Loop while DR register in not empty */
	while (SPI_I2S_GetFlagStatus(RFM12_SPI, SPI_I2S_FLAG_TXE) == RESET)
	{
	}

	/* Wait to receive a Byte */
	while (SPI_I2S_GetFlagStatus(RFM12_SPI, SPI_I2S_FLAG_RXNE) == RESET)
	{
	}

	/* Return the Byte read from the SPI bus */
	retVal = SPI_I2S_ReceiveData(RFM12_SPI);

	RFM12_CS_HIGH();

	return retVal;
}

//unsigned short rf12_trans(unsigned short wert)
//{	unsigned short werti=0;
//	unsigned char i;
//
//	cbi(RF_PORT, CS);
//	for (i=0; i<16; i++)
//	{	if (wert&32768)
//			sbi(RF_PORT, SDI);
//		else
//			cbi(RF_PORT, SDI);
//		werti<<=1;
//		if (RF_PIN&(1<<SDO))
//			werti|=1;
//		sbi(RF_PORT, SCK);
//		wert<<=1;
//		_delay_us(0.3);
//		cbi(RF_PORT, SCK);
//	}
//	sbi(RF_PORT, CS);
//	return werti;
//}

void rf12_init(void)
{
	rf12_trans(0xC0E0);			// AVR CLK: 10MHz
	rf12_trans(0x80D7);			// Enable FIFO
	rf12_trans(0xC2AB);			// Data Filter: internal
	rf12_trans(0xCA81);			// Set FIFO mode
	rf12_trans(0xE000);			// disable wakeuptimer
	rf12_trans(0xC800);			// disable low duty cycle
	rf12_trans(0xC4F7);			// AFC settings: autotuning: -10kHz...+7,5kHz
}

void rf12_setbandwidth(unsigned char bandwidth, unsigned char gain, unsigned char drssi)
{
	rf12_trans(0x9400|((bandwidth&7)<<5)|((gain&3)<<3)|(drssi&7));
}

void rf12_setfreq(unsigned short freq)
{	if (freq<96)				// 430,2400MHz
		freq=96;
	else if (freq>3903)			// 439,7575MHz
		freq=3903;
	rf12_trans(0xA000|freq);
}

void rf12_setbaud(unsigned short baud)
{
	if (baud<663)
		return;
	if (baud<5400)					// Baudrate= 344827,58621/(R+1)/(1+CS*7)
		rf12_trans(0xC680|((43104/baud)-1));
	else
		rf12_trans(0xC600|((344828UL/baud)-1));
}

void rf12_setpower(unsigned char power, unsigned char mod)
{
	rf12_trans(0x9800|(power&7)|((mod&15)<<4));
}

void rf12_ready(void)
{
	RFM12_CS_LOW();
	//Delay(2);

	while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2)!=1);
	//cbi(RF_PORT, CS);
	//while (!(RF_PIN&(1<<SDO))); // wait until FIFO ready
}

void rf12_txdata(unsigned char *data, unsigned char number)
{	unsigned char i;
	rf12_trans(0x8238);			// TX on
	rf12_ready();
	rf12_trans(0xB8AA);
	rf12_ready();
	rf12_trans(0xB8AA);
	rf12_ready();
	rf12_trans(0xB8AA);
	rf12_ready();
	rf12_trans(0xB82D);
	rf12_ready();
	rf12_trans(0xB8D4);
	for (i=0; i<number; i++)
	{		rf12_ready();
		rf12_trans(0xB800|(*data++));
	}
	rf12_ready();
	rf12_trans(0x8208);			// TX off
}

void rf12_rxdata(unsigned char *data, unsigned char number)
{	unsigned char i;
	rf12_trans(0x82C8);			// RX on
	rf12_trans(0xCA81);			// set FIFO mode
	rf12_trans(0xCA83);			// enable FIFO
	//USART_debug(USART2, "%d\n\r", rf12_trans(0x0000));
	for (i=0; i<number; i++)
	{	rf12_ready();
		*data++=rf12_trans(0xB000);
	}
	rf12_trans(0x8208);			// RX off
}







