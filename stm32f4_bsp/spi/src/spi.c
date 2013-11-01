/*
 * rfm12.c
 *
 *  Created on: Nov 1, 2013
 *      Author: franz
 */

#include <stm32f4xx.h>
#include <stm32_configuration.h>
#include <spi.h>

void SPI1_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;

	/* SPI GPIO Configuration --------------------------------------------------*/

	/* CS */
	GPIO_InitStructure.GPIO_Pin = RFM12_NSEL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(RFM12_NSEL_PORT, &GPIO_InitStructure);

	/* Connect SPI pins to AF5 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	GPIO_PinAFConfig(SPI1_GPIO_PORT, SPI1_SCK_PIN_SRC, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI1_GPIO_PORT, SPI1_MISO_PIN_SRC, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI1_GPIO_PORT, SPI1_MOSI_PIN_SRC, GPIO_AF_SPI1);

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN;
	GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  SPI1_MOSI_PIN;
	GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStructure);

	/* SPI  MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  SPI1_MISO_PIN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStructure);

	SPI_InitTypeDef  SPI_InitStructure;

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_CalculateCRC(SPI1, DISABLE);
	/* Enable the SPI peripheral */
	SPI_Cmd(SPI1, ENABLE);

	/* drain SPI */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) { ; }
	SPI_I2S_ReceiveData(SPI1);
}


