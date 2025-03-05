#include <stm32f407xx_gpio_driver.h>

/*
 * Mapping Port To Numerical
 */
/*********************************************************************
 * @fn      		  - MappingPortToNumerical
 *
 * @brief             - This function is to translate a configured port to a numerical
 *
 * @param[in]         - none
 * @param[in]         - none
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
static GPIO_e MappingPortToNumerical(GPIO_Handle_t *pGPIOHandle)
{

	if (pGPIOHandle->pGPIOx == GPIOA)
	{
		return GPIOA_e;
	}
	else if (pGPIOHandle->pGPIOx == GPIOB)
	{
		return GPIOB_e;
	}
	else if (pGPIOHandle->pGPIOx == GPIOC)
	{
		return GPIOC_e;
	}
	else if (pGPIOHandle->pGPIOx == GPIOD)
	{
		return GPIOD_e;
	}
	else if (pGPIOHandle->pGPIOx == GPIOE)
	{
		return GPIOE_e;
	}
	else if (pGPIOHandle->pGPIOx == GPIOF)
	{
		return GPIOF_e;
	}
	else if (pGPIOHandle->pGPIOx == GPIOG)
	{
		return GPIOG_e;
	}
	else if (pGPIOHandle->pGPIOx == GPIOH)
	{
		return GPIOH_e;
	}
	else if (pGPIOHandle->pGPIOx == GPIOI)
	{
		return GPIOI_e;
	}
	else
	{
		return GPIOMAX_e;
	}

}
/*
 * SYSCFG interrupt configuration
 */
/*********************************************************************
 * @fn      		  - SYSCFG_EnableInterrupt
 *
 * @brief             - This function enables or disables interrupt line for a respective GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
static void SYSCFG_EnableInterrupt(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t PortResNum = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
	uint8_t PortPinNum = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
	GPIO_e PortName = MappingPortToNumerical(pGPIOHandle);

	// Enable clock system
	SYSCFG_PCLK_EN();
	// Enable the interrupt needed pin
	SYSCFG->EXTICR[PortResNum] = PortName << (PortPinNum * 4);
}

/*
 * Peripheral Clock setup
 */
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (GPIOA == pGPIOx)
		{
			GPIOA_PCLK_EN();
		}
		else if (GPIOB == pGPIOx)
		{
			GPIOB_PCLK_EN();
		}
		else if (GPIOC == pGPIOx)
		{
			GPIOC_PCLK_EN();
		}
		else if (GPIOD == pGPIOx)
		{
			GPIOD_PCLK_EN();
		}
		else if (GPIOE == pGPIOx)
		{
			GPIOE_PCLK_EN();
		}
		else if (GPIOF == pGPIOx)
		{
			GPIOF_PCLK_EN();
		}
		else if (GPIOG == pGPIOx)
		{
			GPIOG_PCLK_EN();
		}
		else if (GPIOH == pGPIOx)
		{
			GPIOH_PCLK_EN();
		}
		else if (GPIOI == pGPIOx)
		{
			GPIOI_PCLK_EN();
		}
		else
		{

		}
	}
	else if (EnorDi == ENABLE)
	{
		if (GPIOA == pGPIOx)
		{
			GPIOA_PCLK_DI();
		}
		else if (GPIOB == pGPIOx)
		{
			GPIOB_PCLK_DI();
		}
		else if (GPIOC == pGPIOx)
		{
			GPIOC_PCLK_DI();
		}
		else if (GPIOD == pGPIOx)
		{
			GPIOD_PCLK_DI();
		}
		else if (GPIOE == pGPIOx)
		{
			GPIOE_PCLK_DI();
		}
		else if (GPIOF == pGPIOx)
		{
			GPIOF_PCLK_DI();
		}
		else if (GPIOG == pGPIOx)
		{
			GPIOG_PCLK_DI();
		}
		else if (GPIOH == pGPIOx)
		{
			GPIOH_PCLK_DI();
		}
		else if (GPIOI == pGPIOx)
		{
			GPIOI_PCLK_DI();
		}
		else
		{

		}
	}
	else
	{
		return;
	}
}

/*
 * Init and De-init
 */
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function is to initialize the a specific PIN of a GPIO port
 *
 * @param[in]         - pointer to the GPIOx handler
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t Config1Bit = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	uint32_t Config2Bit = 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	uint8_t AltResNum = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
	uint8_t AltPinNum = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;

	// Mode setting
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) // Non interrupt mode
	{
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << Config1Bit);
		pGPIOHandle->pGPIOx->MODER |= pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << Config2Bit;
	}
	else // IRQ mode
	{
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) // falling edge Interrupt mode
		{
			EXTI->FTSR |= (1 << Config1Bit); // set falling
			EXTI->RTSR &= ~(1 << Config1Bit);// clear rising
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) // rising edge Interrupt mode
		{
			EXTI->RTSR |= (1 << Config1Bit); // set rising
			EXTI->FTSR &= ~(1 << Config1Bit);// clear falling
		}
		else // both falling and rising edges
		{
			EXTI->RTSR |= (1 << Config1Bit); // set rising
			EXTI->FTSR |= (1 << Config1Bit);// set falling
		}
		// Enable SYSCFG EXTICR for GPIO port
		SYSCFG_EnableInterrupt(pGPIOHandle);
		// Enable EXTI
		EXTI->IMR |= (1 << Config1Bit);

	}

	// Alternate Func
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) //  mode ALT
	{
		pGPIOHandle->pGPIOx->AFR[AltResNum] &= ~(0xf << 4*AltPinNum);
		pGPIOHandle->pGPIOx->AFR[AltResNum] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << 4*AltPinNum);
	}

	// Speed
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << Config1Bit);
	pGPIOHandle->pGPIOx->OSPEEDR |= pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << Config2Bit;
	// Pupd
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << Config1Bit);
	pGPIOHandle->pGPIOx->PUPDR |= pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << Config2Bit;
	// Output type
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << Config1Bit);
	pGPIOHandle->pGPIOx->OTYPER |= pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << Config1Bit;


}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function is to deinitialize the GPIO port
 *
 * @param[in]         - pointer to the GPIOx base address
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (GPIOA == pGPIOx)
	{
		GPIOA_REG_RESET();
	}
	else if (GPIOB == pGPIOx)
	{
		GPIOB_REG_RESET();
	}
	else if (GPIOC == pGPIOx)
	{
		GPIOC_REG_RESET();
	}
	else if (GPIOD == pGPIOx)
	{
		GPIOD_REG_RESET();
	}
	else if (GPIOE == pGPIOx)
	{
		GPIOE_REG_RESET();
	}
	else if (GPIOF == pGPIOx)
	{
		GPIOF_REG_RESET();
	}
	else if (GPIOG == pGPIOx)
	{
		GPIOG_REG_RESET();
	}
	else if (GPIOH == pGPIOx)
	{
		GPIOH_REG_RESET();
	}
	else if (GPIOI == pGPIOx)
	{
		GPIOI_REG_RESET();
	}
	else
	{

	}
}


/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	return (uint8_t)((pGPIOx->IDR >> PinNumber) & (uint32_t) 1);
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)(pGPIOx->IDR);

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if (ENABLE == value)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	pGPIOx->ODR ^= (1 << PinNumber);

}


/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint8_t Priority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( Priority << shift_amount );
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the EXTI->PR pending int
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}

/*
 * Configuration
 */
void GPIO_Config(GPIO_Handle_t *pGPIOHandle, GPIO_RegDef_t *port, uint8_t pin, uint8_t mode, uint8_t speed, uint8_t pupd, uint8_t optype, uint8_t altmode)
{
	pGPIOHandle->pGPIOx = port;
	pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber = pin;
	pGPIOHandle->GPIO_PinConfig.GPIO_PinMode = mode;
	pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed = speed;
	pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType = pupd;
	pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl = optype;
	pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode = altmode;
}
