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

	Delay(2);

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

	Delay(1000);

	return retVal;
}



void RFM12_init_tx(void)
{

	SPI1_init();

	rf12_trans(0xFF00); //reset RFM12

//	rf12_trans(0xC0E0);			// AVR CLK: 10MHz
//	rf12_trans(0x80D7);			// Enable FIFO
//	rf12_trans(0xC2AB);			// Data Filter: internal
//	rf12_trans(0xCA81);			// Set FIFO mode
//	rf12_trans(0xE000);			// disable wakeuptimer
//	rf12_trans(0xC800);			// disable low duty cycle
//	rf12_trans(0xC4F7);			// AFC settings: autotuning: -10kHz...+7,5kHz

//	rf12_trans(0xCA01); // FiFo and resetmode command ; FIFO fill disabeld
//	rf12_trans(0x8017); // disable FIFO, 433MHz, 12.0pF
//	rf12_trans(0x8209); // synth on, PLL on, enable xtal, enable CLKOUT pin, disable Batt
//	rf12_trans(0xA620); //   0xA620 = 433.92MHz
//	rf12_trans(0xC647); // c647 4.8Kbps (38.4: 8, 19.2: 11, 9.6: 23, 4.8: 47)
//	rf12_trans(0x9489); // VDI,FAST,BW200kHz,-6dBm,DRSSI -97dbm
//	rf12_trans(0xC220); // datafiltercommand ; ** not documented command **
//	rf12_trans(0xC4c3); // enable AFC ;enable frequency offset
//	rf12_trans(0xCC67); //
//	rf12_trans(0xC000); // clock output 1.00MHz, can be used to see if SPI works
//	rf12_trans(0xE000); // disable wakeuptimer
//	rf12_trans(0xC800); // disable low duty cycle

	rf12_trans(0x80D7); //EL,EF,868band,12.0pF
	rf12_trans(0x8239); //!er,!ebb,ET,ES,EX,!eb,!ew,DC
	rf12_trans(0xA640); //frequency select
	rf12_trans(0xC647); //4.8kbps
	rf12_trans(0x94A0); //VDI,FAST,134kHz,0dBm,-103dBm
	rf12_trans(0xC2AC); //AL,!ml,DIG,DQD4
	rf12_trans(0xCA81); //FIFO8,SYNC,!ff,DR
  	rf12_trans(0xC483); //@PWR,NO RSTRIC,!st,!fi,OE,EN
  	rf12_trans(0x9850); //!mp,90kHz,MAX OUT
  	rf12_trans(0xE000); //NOT USE
  	rf12_trans(0xC800); //NOT USE
  	rf12_trans(0xC400); //1.66MHz,2.2V
}

void RFM12_init_rx(void)
{

	SPI1_init();

	rf12_trans(0xFF00); //reset RFM12

//	rf12_trans(0xC0E0);			// AVR CLK: 10MHz
//	rf12_trans(0x80D7);			// Enable FIFO
//	rf12_trans(0xC2AB);			// Data Filter: internal
//	rf12_trans(0xCA81);			// Set FIFO mode
//	rf12_trans(0xE000);			// disable wakeuptimer
//	rf12_trans(0xC800);			// disable low duty cycle
//	rf12_trans(0xC4F7);			// AFC settings: autotuning: -10kHz...+7,5kHz

//	rf12_trans(0xCA01); // FiFo and resetmode command ; FIFO fill disabeld
//	rf12_trans(0x8017); // disable FIFO, 433MHz, 12.0pF
//	rf12_trans(0x8209); // synth on, PLL on, enable xtal, enable CLKOUT pin, disable Batt
//	rf12_trans(0xA620); //   0xA620 = 433.92MHz
//	rf12_trans(0xC647); // c647 4.8Kbps (38.4: 8, 19.2: 11, 9.6: 23, 4.8: 47)
//	rf12_trans(0x9489); // VDI,FAST,BW200kHz,-6dBm,DRSSI -97dbm
//	rf12_trans(0xC220); // datafiltercommand ; ** not documented command **
//	rf12_trans(0xC4c3); // enable AFC ;enable frequency offset
//	rf12_trans(0xCC67); //
//	rf12_trans(0xC000); // clock output 1.00MHz, can be used to see if SPI works
//	rf12_trans(0xE000); // disable wakeuptimer
//	rf12_trans(0xC800); // disable low duty cycle

	rf12_trans(0x80D7); //EL,EF,868band,12.0pF
	rf12_trans(0x82D9); //!er,!ebb,ET,ES,EX,!eb,!ew,DC
	rf12_trans(0xA640); //frequency select
	rf12_trans(0xC647); //4.8kbps
	rf12_trans(0x94A0); //VDI,FAST,134kHz,0dBm,-103dBm
	rf12_trans(0xC2AC); //AL,!ml,DIG,DQD4
	rf12_trans(0xCA81); //FIFO8,SYNC,!ff,DR
  	rf12_trans(0xC483); //@PWR,NO RSTRIC,!st,!fi,OE,EN
  	rf12_trans(0x9850); //!mp,90kHz,MAX OUT
  	rf12_trans(0xE000); //NOT USE
  	rf12_trans(0xC800); //NOT USE
  	rf12_trans(0xC400); //1.66MHz,2.2V
  	rf12_trans(0xCA81); //FIFO8,SYNC,!ff,DR
  	//rf12_trans(0xCA83); //FIFO8,SYNC,!ff,DR
}



void rf12_setbandwidth(unsigned char bandwidth, unsigned char gain, unsigned char drssi)
{
	rf12_trans(0x9400|((bandwidth&7)<<5)|((gain&3)<<3)|(drssi&7));
}

void rf12_setfreq(uint16_t freq)
{	if (freq<96)				// 430,2400MHz
		freq=96;
	else if (freq>3903)			// 439,7575MHz
		freq=3903;
	rf12_trans(0xA000|freq);
}

void rf12_setbaud(uint16_t baud)
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

void RFM12_Ready()
{
	Delay(1);
}

void rf12_txdata(unsigned char *data, unsigned char number)
{
	uint8_t i;

	rf12_trans(0x0000);

	/* enable TX */
	rf12_trans(0x8239);
	RFM12_Ready();

	/* send preamble (0xAA) */
	rf12_trans(0xB8AA);
	RFM12_Ready();
	rf12_trans(0xB8AA);
	RFM12_Ready();
	rf12_trans(0xB8AA);
	RFM12_Ready();

	/* send sync word 0x2DD4 */
	rf12_trans(0xB82D);
	RFM12_Ready();
	rf12_trans(0xB8D4);

	/* send data buffer */
	for (i=0; i < number; i++)
	{   RFM12_Ready();
		rf12_trans(0xB800|(*data++));
	}
	RFM12_Ready();

	/* transmit 2 dummy bytes to avoid that last bytes of real payload don't */
	/* get transmitted properly (due to transmitter disabled to early) */
	rf12_trans(0xB801);
	RFM12_Ready();
	//rf12_trans(0xB800);
	//RFM12_Ready();

	/* disable TX */
	//rf12_trans(0x8208);
}

void rf12_rxdata(uint16_t *data, unsigned char number)
{
	uint8_t i;

	/* enable RX */
	//rf12_trans(0x82C8);

	/* set FIFO mode */
	rf12_trans(0xCA81);

	/* enable FIFO */
	rf12_trans(0xCA83);

	USART_debug(USART2, "Status: %u\n\r", rf12_trans(0x0000));

	//RFM12_ClrSDI();
	//delay_us(3);
	for (i=0; i < number; i++)
	{
		RFM12_Ready();
		*data++=rf12_trans(0xB000);
	}

	/* disable RX */
	rf12_trans(0xCA81);
}


