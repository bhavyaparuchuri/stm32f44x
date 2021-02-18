/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 13-Nov-2020
 *      Author: bhavi
 */

#include "../inc/stm32f446xx_spi_driver.h"
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

	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
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
			tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		}
		else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
		{
			//bidi mode should be set
			tempreg |= (1<<SPI_CR1_BIDIMODE);
		}
		else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
		{
			//BIDI mode should be cleared
			tempreg &= ~(1<< SPI_CR1_BIDIMODE); // 15
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
				if(pSPIx == SPI1)
				{
					SPI1_REG_RESET();
				}
				else if(pSPIx == SPI2)
				{
					SPI2_REG_RESET();
				}
				else if(pSPIx == SPI3)
				{
					SPI3_REG_RESET();
				}
				else if(pSPIx == SPI4)
				{
					SPI4_REG_RESET();
				}

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

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	//blocking API
		while(Len > 0)
		{
			//1.wait until RXNE is set
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);

			//2. check the DFF bit in CR1
			if(pSPIx->CR1 & (1<< SPI_CR1_DFF) )
			{
				//16 BIT DFF
				//1. load the data in from DR to Rxbuffer address
				*((uint16_t*)pRxBuffer) = pSPIx->DR ;
				Len--;
				Len--;// Note: As it is 2 bytes of data, we decrease 2 times.
				(uint16_t*)pRxBuffer++;

			}
			else
			{
				//8 BIT DFF
				*pRxBuffer=pSPIx->DR ;
				Len--;
				pRxBuffer++;
			}
		}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = *pSPIHandle->pTxState;

		if(state != SPI_BUSY_IN_TX)
		{
		//1. Save the TX Buffer address and Len information in some global variables
			pSPIHandle->pTxBuffer = pTxBuffer;
			pSPIHandle->TxLen = Len;
		//2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
			*pSPIHandle->pTxState = (1 << SPI_BUSY_IN_TX);
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
			pSPIHandle->pSPIx->CR2 |=(1 << SPI_CR2_TXEIE);
		//4. Data transmission will be handled by the ISR code (will implement later)
		}
		return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = *pSPIHandle->pRxState;

	if(state != SPI_BUSY_IN_RX)
	{
	//1. Save the RX Buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
	//2. Mark the SPI state as busy in transmission so that
	// no other code can take over same SPI peripheral until transmission is over
		*pSPIHandle->pRxState = (1 << SPI_BUSY_IN_RX);
	//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |=(1<<SPI_CR2_RXNEIE);
	//4. Data transmission will be handled by the ISR code (will implement later)
	}
	return state;
}
/******************************************************************************************************************
 * @fn 							-	SPI_PeripheralControl
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
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}
		else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}

}

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				*NVIC_ISER0 |= (1 << IRQNumber );
			}
			else if(IRQNumber > 31 && IRQNumber < 64)
			{
				//program ISER1 register
				*NVIC_ISER1 |= (1 << (IRQNumber % 32));
			}
			else if(IRQNumber > 64 && IRQNumber < 96)
			{
				//program ISER2 register
				*NVIC_ISER2 |= (1 << (IRQNumber % 64));

			}
		}
		else
		{

			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= (1 << IRQNumber );
			}
			else if(IRQNumber > 31 && IRQNumber < 64)
			{
				//program ICER1 register
				*NVIC_ICER1 |= (1 << (IRQNumber % 64));
			}
			else if(IRQNumber > 64 && IRQNumber < 96)
			{
				//program ICER2 register
				*NVIC_ICER2 |= (1 << (IRQNumber % 96));
			}

		}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	// 1. first lets find out the ipr register
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section = IRQNumber % 4;
		uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
		*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= ( IRQPriority << (8 * shift_amount));
}
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	/*To process the interrupt*/

	uint8_t temp1,temp2;
	//first lets checks for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_rxne_interrupt_handle(pHandle);
	}
	//first lets checks for ovr flag
		temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
		temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

		if(temp1 && temp2)
			{
				//handle TXE
				spi_ovr_err_interrupt_handle(pHandle);
			}


}

//some helper functions

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1<< SPI_CR1_DFF) )
			{
				//16 BIT DFF
				//1. load the data in to DR
				pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->TxLen--;// Note: As it is 2 bytes of data, we decrease 2 times.
				(uint16_t*)pSPIHandle->pTxBuffer++;

			}
			else
			{
				//8 BIT DFF
				pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}
			if(! pSPIHandle->TxLen)
			{
				//TxLen is zero, so close the spi transmission and inform the application that
				//Tx is over
				// this prevents interrupt from setting up of TXE flag
				SPI_CloseTrasmission(pSPIHandle);
				SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

			}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1<< SPI_CR1_DFF) )
				{
					//16 BIT DFF
					//1. load the data in to DR
					 *((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
					 pSPIHandle->RxLen -=2;
					pSPIHandle->pRxBuffer--;
					pSPIHandle->pRxBuffer--;

				}
				else
				{
					//8 BIT DFF
					*pSPIHandle->pTxBuffer = (uint8_t)pSPIHandle->pSPIx->DR;
					pSPIHandle->RxLen--;
					pSPIHandle->pRxBuffer--;
				}
				if(! pSPIHandle->RxLen)
				{
					//TxLen is zero, so close the spi transmission and inform the application that
					//Tx is over
					// this prevents interrupt from setting up of TXE flag
					SPI_CloseReception(pSPIHandle);
					SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

				}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//clear the ovr flag
	uint8_t temp;
	if(*pSPIHandle->pTxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}


void SPI_CloseTrasmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->pTxState = SPI_READY;

}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->pRxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle, uint8_t AppEv)
{
	//This is weak implementation. the applicastion may override this function
}
