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
 * @param[in]         - pSPIx: SPI peripheral base address
 * @param[in]         - EnorDi: ENABLE or DISABLE
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
 * @fn      		  - SPI_HWInit
 *
 * @brief             - This function is to initialize the HW SPI driver
 *
 * @param[in]         - pSPIHandle: SPI handle structure
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_HWInit(SPI_Handle_t *pSPIHandle)
{
	uint32_t reg_value = 0;

	// Enable clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

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
	// SSM
	reg_value |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;
	// Write setting into reg CR1
	pSPIHandle->pSPIx->CR1 = reg_value;
}

/*
 * SPI DeInit
 */
/*********************************************************************
 * @fn      		  - SPI_HWDeInit
 *
 * @brief             - This function is to de-init HW
 *
 * @param[in]         - pSPIHandle: SPI handle structure
 *
 * @return            -  none
 *
 * @Note              -  corresponding RCC reset bits must be set before clearing

 */
void SPI_HWDeInit(SPI_Handle_t *pSPIHandle)
{
    if (pSPIHandle->pSPIx == SPI1)
    {
        // Reset SPI1 using RCC
        RCC->APB2RSTR |= (1 << 12);
        RCC->APB2RSTR &= ~(1 << 12);
    }
    else if (pSPIHandle->pSPIx == SPI2)
    {
        // Reset SPI2 using RCC
        RCC->APB1RSTR |= (1 << 14);
        RCC->APB1RSTR &= ~(1 << 14);
    }
    else if (pSPIHandle->pSPIx == SPI3)
    {
        // Reset SPI3 using RCC
        RCC->APB1RSTR |= (1 << 15);
        RCC->APB1RSTR &= ~(1 << 15);
    }
}

/*
 * SPI Send data
 */
/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function is to send data buffer
 *
 * @param[in]         - pSPIHandle: SPI handle structure
 * @param[in]         - pTxBuffer: Transmit buffer pointer
 * @param[in]         - Length: Length of data to be sent
 *
 * @return            -  none
 *
 * @Note              -  This is a blocking call, polling technique

 */
void SPI_SendData(SPI_Handle_t *pSPIHandle, void *pTxBuffer, uint32_t Length)
{
	while (Length > 0)
	{
		// Wait until TXE is set (transmit buffer empty)
		while (SPI_GETFLAGSTATUS(pSPIHandle->pSPIx->SR, SPI_TXE_FLAG) == SPI_RESET);

		if (SPI_DFF_8BITS == SPI_GETDFFTYPE(pSPIHandle->SPIConfig.SPI_DFF)) // 8 Bit data loaded
		{
			pSPIHandle->pSPIx->DR = *(uint8_t *)pTxBuffer; // Read data from local buffer
			Length -= 1; // Decrement length
			pTxBuffer++; // Increment buffer pointer
		}
		else // 16 Bit data loaded
		{
			pSPIHandle->pSPIx->DR = *(uint16_t *)pTxBuffer; // Read data from local buffer
			Length -= 2; // Decrement length
			pTxBuffer += 2; // Increment buffer pointer
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
 * @param[in]         - pSPIx: SPI peripheral base address
 * @param[in]         - pRxBuffer: Pointer to the receive buffer
 * @param[in]         - Length: Length of data to be received
 *
 * @return            -  none
 *
 * @Note              -  This is a blocking call, polling technique

 */
void SPI_ReceiveData(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length)
{
	while (Length > 0)
	{
		// Wait until RXNE is set (data received)
		while (SPI_GETFLAGSTATUS(pSPIHandle->pSPIx->SR, SPI_RXNE_FLAG) == SPI_RESET);

		if (SPI_GETDFFTYPE((pSPIHandle->pSPIx->CR1 >> SPI_CR1_DFF) & 0x1) == SPI_DFF_8BITS)
		{
			// 8-bit data loaded
			*pRxBuffer = (uint8_t)pSPIHandle->pSPIx->DR; // Read data from SPI data register
			pRxBuffer++; // Increment buffer pointer
			Length--; // Decrement length
		}
		else
		{
			// 16-bit data loaded
			*((uint16_t*)pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR; // Read data from SPI data register
			pRxBuffer += 2; // Increment buffer pointer
			Length -= 2; // Decrement length
		}
	}
}

/*
 * SPI Interrupt configuration
 */
/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             - This function is to configure SPI interrupt
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

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - Enables or disables the SPI peripheral.
 *
 * @param[in]         - pSPIHandle: Pointer to the SPI handle structure.
 * @param[in]         - EnorDi: ENABLE or DISABLE macro.
 *
 * @return            - None
 *
 * @Note              - This function sets or clears the SPE (SPI Enable) bit in the SPI_CR1 register,
 *                      allowing you to turn the SPI peripheral on or off as needed.
 *********************************************************************/
void SPI_PeripheralControl(SPI_Handle_t *pSPIHandle, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
		pSPIHandle->pSPIx->CR1 |= (uint32_t)(1 << SPI_CR1_SSI);

        pSPIHandle->pSPIx->CR1 |= (uint32_t)(1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIHandle->pSPIx->CR1 &= ~(uint32_t)(1 << SPI_CR1_SPE);
    }
}


/*********************************************************************
 * @fn       - SPI_SSOEConfig
 *
 * @brief    - Configures the SSOE (Slave Select Output Enable) setting for the SPI peripheral.
 *
 * @param[in]  - pSPIHandle: Pointer to the SPI handle structure.
 * @param[in]  - EnorDi: ENABLE or DISABLE macro.
 *
 * @return   - none
 *
 * @Note     - When SSOE is enabled and the SPI is in master mode, the NSS pin is managed automatically by hardware.
 *********************************************************************/
void SPI_SSOEConfig(SPI_Handle_t *pSPIHandle, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        pSPIHandle->pSPIx->CR2 |= (uint32_t)(1 << SPI_CR2_SSOE);
    }
    else
    {
        pSPIHandle->pSPIx->CR2 &= ~(uint32_t)(1 << SPI_CR2_SSOE);
    }
}