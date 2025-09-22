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
	reg_value |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;
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
 * @brief             - This function is to configure SPI interrupt mode
 *
 * @param[in]         - IRQNumber: SPI IRQ number
 * @param[in]         - EnorDi: ENABLE or DISABLE macro
 * @param[in]         - none
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (ENABLE == EnorDi)
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
		else
		{

		}
	}
	else // Disable
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
		else
		{

		}
	}
}
/*
 * SPI Interrupt priority
 */
/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - This function is to set interrupt priority for SPI
 *
 * @param[in]         - IRQNumber: SPI IRQ number
 * @param[in]         - Priority: Interrupt priority level
 * @param[in]         - none
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  Priority value should be between 0 and 15.
 * 						 The lower the number, the higher the priority.

 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( Priority << shift_amount );
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

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - Configures the SSI (Internal Slave Select) bit for the SPI peripheral.
 *
 * @param[in]         - pSPIHandle: Pointer to the SPI handle structure.
 * @param[in]         - EnOrDi: ENABLE or DISABLE macro.
 *
 * @return            - None
 *
 * @Note              - When Software Slave Management (SSM) is enabled, the SSI bit simulates the NSS signal.
 *                      In master mode, SSI must be set (ENABLE) to allow SPI operation; otherwise, the SPI peripheral will disable itself.
 *********************************************************************/
void  SPI_SSIConfig(SPI_Handle_t *pSPIHandle, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pSPIHandle->pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
    }else
    {
        pSPIHandle->pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
    }
}

/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             - This function is to send data using interrupt mode.
 *
 * @param[in]         - pSPIHandle: Pointer to the SPI handle structure.
 * @param[in]         - pTxBuffer: Pointer to the data buffer to be transmitted.
 * @param[in]         - Length: Length of the data to be transmitted.
 *
 * @return            - Status of the transmission request (e.g: SUCCESS, BUSY, OR FAILURE...).
 *
 * @Note              - This function is intended for non-blocking data transmission using interrupts.
 *********************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, void *pTxBuffer, uint32_t Length)
{
	uint8_t state = pSPIHandle->TxState;
	// Check if SPI is already busy in transmission
	if (state != SPI_BUSY_IN_TX)
	{
		// 1. Save Tx buffer address and length information in the SPI handle structure
		pSPIHandle->pTxBuffer = (uint8_t *)pTxBuffer;
		pSPIHandle->TxLen = Length;
		// 2. Mark the SPI state as busy in transmission so that no other code can take over the SPI peripheral until transmission is complete
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		// 3. Enable TXE interrupt
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
		// 4. Hand over the SPI peripheral to the IRQ handler
	}
	return state;
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             - This function is to receive data using interrupt mode.
 *
 * @param[in]         - pSPIHandle: Pointer to the SPI handle structure.
 * @param[in]         - pRxBuffer: Pointer to the data buffer to be received.
 * @param[in]         - Length: Length of the data to be received.
 *
 * @return            - Status of the transmission request (e.g: SUCCESS, BUSY, OR FAILURE...).
 *
 * @Note              - This function is intended for non-blocking data transmission using interrupts.
 *********************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, void *pRxBuffer, uint32_t Length)
{
	uint8_t state = pSPIHandle->RxState;
	// Check if SPI is already busy in reception
	if (state != SPI_BUSY_IN_RX)
	{
		// 1. Save Rx buffer address and length information in the SPI handle structure
		pSPIHandle->pRxBuffer = (uint8_t *)pRxBuffer;
		pSPIHandle->RxLen = Length;
		// 2. Mark the SPI state as busy in reception so that no other code can take over the SPI peripheral until reception is complete
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		// 3. Enable RXNE interrupt
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
		// 4. Hand over the SPI peripheral to the IRQ handler
	}
	return state;
}