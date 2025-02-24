
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
/* Bus state */
#define SPI_BUSY_IN_RX     1
#define SPI_BUSY_IN_TX     2
#define SPI_READY          0
/*
 * SPI macro functions
 */
#define SPI_GETFLAGSTATUS(SPIx_SR, flag_name) ((SPIx_SR & flag_name) ? SPI_SET : SPI_RESET)
#define SPI_GETDFFTYPE(SPIx_DFF_TYPE) ((SPIx_DFF_TYPE == SPI_DFF_8BITS) ? SPI_DFF_8BITS : SPI_DFF_16BITS)
#define SPI_READ_BIT(SPIx_reg, bit_name) ((SPIx_reg & (1 << bit_name)) ? SPI_SET : SPI_RESET)
#define SPI_CLEAR_BIT(SPIx_reg, bit_name) (SPIx_reg &= ~(1 << bit_name))
#define SPI_SET_BIT(SPIx_reg, bit_name) (SPIx_reg |= (1 << bit_name))

/* Callback completion flag */
#define SPI_EVENT_TX_CMPLT  1
#define SPI_EVENT_RX_CMPLT  2
#define SPI_EVENT_OVR_ERR   3
#define SPI_EVENT_CRC_ERR   4

/********************************************** Typedef definition ***********************************************/
/* Configuration SPI */
typedef struct
{
	uint8_t SPI_DeviceMode;   /* Specifies master or slave mode. Use SPI_DEVICE_MODE_MASTER or SPI_DEVICE_MODE_SLAVE */
	uint8_t SPI_BusConfig;    /* Specifies bus configuration: full-duplex, half-duplex, or simplex RX. Use SPI_BUS_CONFIG_* macros */
	uint8_t SPI_SclkSpeed;    /* Specifies SPI serial clock speed. Use SPI_SCLK_SPEED_DIV* macros */
	uint8_t SPI_DFF;          /* Specifies data frame format: 8 or 16 bits. Use SPI_DFF_* macros */
	uint8_t SPI_CPOL;         /* Specifies clock polarity. Use SPI_CPOL_LOW or SPI_CPOL_HIGH */
	uint8_t SPI_CPHA;         /* Specifies clock phase. Use SPI_CPHA_LOW or SPI_CPHA_HIGH */
	uint8_t SPI_SSM;          /* Specifies software slave management: enabled or disabled. Use SPI_SSM_EN or SPI_SSM_DI */
} SPI_Config_t;

/* SPI handler */
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer; // To store the app. Tx buffer address
	uint8_t *pRxBuffer; // To store the app. Rx buffer address
	uint32_t TxLen;     // To store Tx length
	uint32_t RxLen;     // To store Rx length
	uint8_t TxState;    // To store Tx state
	uint8_t RxState;    // To store Rx state
} SPI_Handle_t;

/********************************************** Function definition ***********************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * HW Init and De-init
 */
void SPI_HWInit(SPI_Handle_t *pSPIHandle);
void SPI_HWDeInit(SPI_Handle_t *pSPIHandle);

/*
 * Communication
 */
void SPI_SendData(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length);
void SPI_ReceiveData(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

/*
 * Peripheral Control
 */
void SPI_PeripheralControl(SPI_Handle_t *pSPIHandle, uint8_t EnorDi);

/*
 * SSOE (Slave Select Output Enable) configuration
 */
void SPI_SSOEConfig(SPI_Handle_t *pSPIHandle, uint8_t EnorDi);

/*
 * SSOE (Slave Select Output Enable) configuration
 */
void SPI_SSIConfig(SPI_Handle_t *pSPIHandle, uint8_t EnOrDi);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
