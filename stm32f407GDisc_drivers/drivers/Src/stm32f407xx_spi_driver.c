/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Mar 19, 2025
 *      Author: anhhu
 */
#include <stm32f407xx_spi_driver.h>

/*
 * SPI Clock Controller
 */
/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function is enable/disable SPI clock
 *
 * @param[in]         - none
 * @param[in]         - none
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (ENABLE == EnorDi)
	{
		if (SPI1 == pSPIx)
		{
			SPI1_PCLK_EN();
		}
		else if (SPI2 == pSPIx)
		{
			SPI2_PCLK_EN();
		}
		else if (SPI3 == pSPIx)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if (SPI1 == pSPIx)
		{
			SPI1_PCLK_DI();
		}
		else if (SPI2 == pSPIx)
		{
			SPI2_PCLK_DI();
		}
		else if (SPI3 == pSPIx)
		{
			SPI3_PCLK_DI();
		}
	}

}

/*
 * SPI Init
 */
/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function is to initialize the SPI driver
 *
 * @param[in]         - none
 * @param[in]         - none
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t reg_value = 0;
	// Device mode
	reg_value |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);
	// Bus config
	if (SPI_BUS_CONFIG_FULLDUPLEX == pSPIHandle->SPIConfig.SPI_BusConfig)
	{
		reg_value &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (SPI_BUS_CONFIG_HALFDUPLEX == pSPIHandle->SPIConfig.SPI_BusConfig)
	{
		reg_value |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (SPI_BUS_CONFIG_SIMPLEX_RX == pSPIHandle->SPIConfig.SPI_BusConfig)
	{
		reg_value &= ~(1 << SPI_CR1_BIDIMODE);
		reg_value |= (1 << SPI_CR1_RXONLY);
	}
	// Baudrate config
	reg_value |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_SCLK_SPEED_DIV16;
	// DFF configure
	reg_value |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
	// CPOL
	reg_value |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	// CPHA
	reg_value |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	// Write setting into reg CR1
	pSPIHandle->pSPIx->CR1 = reg_value;
}

/*
 * SPI DeInit
 */
/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function is to de init
 *
 * @param[in]         - none
 * @param[in]         - none
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_DeInit(SPI_Handle_t *pSPIHandle)
{

}

/*
 * SPI Send data
 */
/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function is to send data buffer
 *
 * @param[in]         - none
 * @param[in]         - none
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_SendData(SPI_Handle_t *pSPIHandle, void *pTxBuffer, uint32_t Length)
{
	while (Length > 0)
	{
		while (SPI_GETFLAGSTATUS(pSPIHandle->pSPIx->SR, SPI_TXE_FLAG) == SPI_RESET);
		if (SPI_DFF_8BITS == SPI_GETDFFTYPE(pSPIHandle->SPIConfig.SPI_DFF)) // 8 Bit data load
		{
			pSPIHandle->pSPIx->DR = *(uint8_t *)pTxBuffer;
			Length -= 1;
		}
		else // 16 Bit data load
		{
			pSPIHandle->pSPIx->DR = *(uint16_t *)pTxBuffer;
			Length -= 2;
		}
	}
}

/*
 * SPI Data reception
 */
/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function is to receive data
 *
 * @param[in]         - none
 * @param[in]         - none
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length)
{

}

/*
 * SPI Interrupt configuration
 */
/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             - This function is to configure spi interrupt
 *
 * @param[in]         - none
 * @param[in]         - none
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
/*
 * SPI Interrupt priority
 */
/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - This function is to set interrupt priority for SPI
 *
 * @param[in]         - none
 * @param[in]         - none
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority)
{

}
/*
 * SPI IRQ
 */
/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - SPI IRQ handler
 *
 * @param[in]         - none
 * @param[in]         - none
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}
