/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 13-Nov-2020
 *      Author: bhavi
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/*
 * SPI Configuration Structure
 */
typedef struct
{
	uint8_t SPI_DeviceMode;  //master or slave
	uint8_t SPI_BusConfig;
	uint8_t SPI_SCLKSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * SPI Handle structure
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/*
 * @CPOL
 */
#define SPI_CPOL_HIGH 					1
#define SPI_CPOL_LOW					0

/*
 * @CPHA
 */
#define SPI_CPHA_HIGH 					1
#define SPI_CPHA_LOW					0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN						1
#define SPI_SSM_DI						0



/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)

/**************************************************************************************************************
 * 									APIs Supported by this driver
 * 				For more information about the APIs clock the function defintion
 */

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);		//To enable or disable the clock.
/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);					/*Initialize given port pin*/
void SPI_DeInit(SPI_RegDef_t *pSPIx);					/*This is to de-Initialize : That means it restores the value back to default state*/

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);
/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);							/*To process the interrupt*/

/*
 * Other Peripheral Control APIs
 */

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */