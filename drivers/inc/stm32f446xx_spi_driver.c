/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 13-Nov-2020
 *      Author: bhavi
 */

#include "stm32f446xx_spi_driver.h"
/*
 * Peripheral Clock Setup
 */
/******************************************************************************************************************
 * @fn 							-	SPI_periclockcontrol
 *
 * @brief						-	This function enables or disables peripheral clock for this given GPIO port
 *
 * @param[in]					-	Base address of the spi peripheral
 * @param[in]					-	ENABLE or DISABLE macros
 * @param[in]					-
 *
 * @return						-	none
 *
 * @Note						-	none

 *********************************************************************************************************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	//To enable or disable the clock.
	if(EnorDi == ENABLE){
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
			else if(pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}

		}
		else{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
			else if(pSPIx == SPI4)
			{
				SPI4_PCLK_DI();
			}

		}
}
/*
 * Init and De-init
 */
/******************************************************************************************************************
 * @fn 							-	SPI_Init
 *
 * @brief						-	This function Initializes the given GPIO port pin
 *
 * @param[in]					-
 * @param[in]					-
 * @param[in]					-
 *
 * @return						-	none

 * @Note						-	none

 *********************************************************************************************************************/

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/*first lets configure the SPI_CR1 register*/
	uint32_t tempreg = 0;
	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. configure the device mode
		tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ; //2
	//2. Configure the bus config
		if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
		{
			//bidi mode should be cleared
			tempreg &= ~(1<<SPI_CR1_BIDI);
		}
		else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
		{
			//bidi mode should be set
			tempreg |= (1<<SPI_CR1_BIDI);
		}
		else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
		{
			//BIDI mode should be cleared
			tempreg &= ~(1<< SPI_CR1_BIDI); // 15
			//RXONLY bit must be set
			tempreg |= (1<< SPI_CR1_RXONLY); //10
		}
		//3. Configure the spi serial clock speed (baud rate)
		tempreg |= pSPIHandle->SPIConfig.SPI_SCLKSpeed << SPI_CR1_BR; //3

		//4. Configure the DFF
		tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF; // 11

		//5. Configure the CPOL
		tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;	//1

		//6. Configure the CPHA
		tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;	// 0

		pSPIHandle->pSPIx->CR1 = tempreg;
}
/******************************************************************************************************************
 * @fn 							-	SPI_DeInit
 *
 * @brief						-	This function reset the value of given GPIO port
 *
 * @param[in]					-	Base address of the gpio port
 * @param[in]					-
 * @param[in]					-
 *
 * @return						-	none
 *
 * @Note						-	none

 *********************************************************************************************************************/

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	/*This is to de-Initialize : That means it restores the value back to default state*/
	//todo
}
uint32_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return	FLAG_RESET;
}
/*
 * Data Send and Receive
 */
/******************************************************************************************************************
 * @fn 							-	SPI_SendData
 *
 * @brief						-	This function is called blocking data , function call will wait untill all the bytes are transmitted
 *
 * @param[in]					-	base address of spi peripheral
 * @param[in]					-	pointer to the data
 * @param[in]					-	length of data
 *
 * @return						-	none
 *
 * @Note						-	this is blocking call

 *********************************************************************************************************************/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	//blocking API
	while(Len > 0)
	{
		//1.wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1<< SPI_CR1_DFF) )
		{
			//16 BIT DFF
			//1. load the data in to DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;// Note: As it is 2 bytes of data, we decrease 2 times.
			(uint16_t*)pTxBuffer++;

		}
		else
		{
			//8 BIT DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
/******************************************************************************************************************
 * @fn 							-	SPI_ReceiveData
 *
 * @brief						-	This function read from given GPIO pin
 *
 * @param[in]					-	Base address of the gpio port
 * @param[in]					-	pinNumber of given gpio port
 * @param[in]					-
 *
 * @return						-	0 or 1;
 *
 * @Note						-	none

 *********************************************************************************************************************/

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{

}

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{

}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	/*To process the interrupt*/
}
