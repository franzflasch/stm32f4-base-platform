/*
 * rfm12.c
 *
 *  Created on: Nov 1, 2013
 *      Author: franz
 */

#include <stm32f4xx.h>
#include <stm32_configuration.h>
#include <spi.h>

void SPI2_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* Connect SPI pins to AF5 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	GPIO_PinAFConfig(SPI2_GPIO_PORT, SPI2_SCK_PIN_SRC, GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI2_GPIO_PORT, SPI2_MISO_PIN_SRC, GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI2_GPIO_PORT, SPI2_MOSI_PIN_SRC, GPIO_AF_SPI2);

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = SPI2_SCK_PIN;
	GPIO_Init(SPI2_GPIO_PORT, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  SPI2_MOSI_PIN;
	GPIO_Init(SPI2_GPIO_PORT, &GPIO_InitStructure);

	/* SPI  MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  SPI2_MISO_PIN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(SPI2_GPIO_PORT, &GPIO_InitStructure);

	SPI_InitTypeDef  SPI_InitStructure;

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI2);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_CalculateCRC(SPI2, DISABLE);
	/* Enable the SPI peripheral */
	SPI_Cmd(SPI2, ENABLE);

	/* drain SPI */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) { ; }
	//SPI_I2S_ReceiveData(SPI2);
}


