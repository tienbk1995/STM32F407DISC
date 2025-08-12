/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Mar 19, 2025
 *      Author: anhhu
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include <stm32f407xx.h>
/********************************************** Macro definition *************************************************/
/* SPI Mode */
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE 			0

/* SPI Bus */
#define SPI_BUS_CONFIG_FULLDUPLEX		1
#define SPI_BUS_CONFIG_HALFDUPLEX		2
#define SPI_BUS_CONFIG_SIMPLEX_RX		3

/* SPI Speed */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/* SPI DFF */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/* CPOL */
#define SPI_CPOL_HIGH 					1
#define SPI_CPOL_LOW 					0

/* CPHA */
#define SPI_CPHA_HIGH 					1
#define SPI_CPHA_LOW 					0

/* SPI_SSM */
#define SPI_SSM_EN     					1
#define SPI_SSM_DI     					0

/* Flag */
#define SPI_SET     					1
#define SPI_RESET     					0
/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)

/*
 * SPI macro functions
 */
#define SPI_GETFLAGSTATUS(SPIx_SR, flag_name) ((SPIx_SR & (1 << flag_name)) ? SPI_SET : SPI_RESET)
#define SPI_GETDFFTYPE(SPIx_DFF_TYPE) ((SPIx_DFF_TYPE == SPI_DFF_8BITS) ? SPI_DFF_8BITS : SPI_DFF_16BITS)

/********************************************** Typedef definition ***********************************************/
/* Configuration SPI */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

/* SPI handler */
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
} SPI_Handle_t;

/********************************************** Function definition ***********************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Handle_t *pSPIHandle);

/*
 * Communication
 */
void SPI_SendData(SPI_Handle_t *pSPIHandle, void *pTxBuffer, uint32_t Length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
