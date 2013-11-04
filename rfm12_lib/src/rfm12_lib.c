/*
 * rfm12_lib.c
 *
 *  Created on: Nov 1, 2013
 *      Author: franz
 */

#include <rfm12_lib.h>
#include <spi.h>
#include <stm32_configuration.h>

uint16_t RFM12_trans(uint16_t value)
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

//unsigned short RFM12_trans(unsigned short wert)
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

void RFM12_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Configure PA0 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = RFM12_SDO_CHECK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(RFM12_SDO_CHECK_PORT, &GPIO_InitStructure);

	/* CS */
	GPIO_InitStructure.GPIO_Pin = RFM12_NSEL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(RFM12_NSEL_PORT, &GPIO_InitStructure);

	RFM12_CS_HIGH();

	/* Set up the RFM12 module */
	RFM12_trans(0xC0E0);			// AVR CLK: 10MHz
	RFM12_trans(0x80D7);			// Enable FIFO
	RFM12_trans(0xC2AB);			// Data Filter: internal
	RFM12_trans(0xCA81);			// Set FIFO mode
	RFM12_trans(0xE000);			// disable wakeuptimer
	RFM12_trans(0xC800);			// disable low duty cycle
	RFM12_trans(0xC4F7);			// AFC settings: autotuning: -10kHz...+7,5kHz
}

void RFM12_setbandwidth(unsigned char bandwidth, unsigned char gain, unsigned char drssi)
{
	RFM12_trans(0x9400|((bandwidth&7)<<5)|((gain&3)<<3)|(drssi&7));
}

void RFM12_setfreq(unsigned short freq)
{	if (freq<96)				// 430,2400MHz
		freq=96;
	else if (freq>3903)			// 439,7575MHz
		freq=3903;
	RFM12_trans(0xA000|freq);
}

void RFM12_setbaud(unsigned short baud)
{
	if (baud<663)
		return;
	if (baud<5400)					// Baudrate= 344827,58621/(R+1)/(1+CS*7)
		RFM12_trans(0xC680|((43104/baud)-1));
	else
		RFM12_trans(0xC600|((344828UL/baud)-1));
}

void RFM12_setpower(unsigned char power, unsigned char mod)
{
	RFM12_trans(0x9800|(power&7)|((mod&15)<<4));
}

void RFM12_ready(void)
{
	RFM12_CS_LOW();
	//Delay(2);

	RFM12_WAIT_SDO_HIGH();
	//while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2)!=1);
	//cbi(RF_PORT, CS);
	//while (!(RF_PIN&(1<<SDO))); // wait until FIFO ready
}

void RFM12_txdata(unsigned char *data, unsigned char number)
{
	unsigned char i;

	RFM12_trans(0x8238);			// TX on
	RFM12_ready();
	RFM12_trans(0xB8AA);
	RFM12_ready();
	RFM12_trans(0xB8AA);
	RFM12_ready();
	RFM12_trans(0xB8AA);
	RFM12_ready();
	RFM12_trans(0xB82D);
	RFM12_ready();
	RFM12_trans(0xB8D4);

	for (i=0; i<number; i++)
	{		RFM12_ready();
		RFM12_trans(0xB800|(*data++));
	}

	RFM12_ready();
	RFM12_trans(0x8208);			// TX off
}

void RFM12_rxdata(unsigned char *data, unsigned char number)
{
	unsigned char i;

	RFM12_trans(0x82C8);			// RX on
	RFM12_trans(0xCA81);			// set FIFO mode
	RFM12_trans(0xCA83);			// enable FIFO
	//USART_debug(USART2, "%d\n\r", RFM12_trans(0x0000));
	for (i=0; i<number; i++)
	{	RFM12_ready();
		*data++=RFM12_trans(0xB000);
	}
	RFM12_trans(0x8208);			// RX off
}

