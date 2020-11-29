/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: 30-Nov-2020
 *      Author: bhavya
 */
#include "stm32f446xx_usart_driver.h"
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	//To enable or disable the clock.
		if(EnorDi == ENABLE)
		{
				if(pUSARTx == USART1)
				{
					USART1_PCLK_EN();
				}
				else if(pUSARTx == USART2)
				{
					USART2_PCLK_EN();
				}
				else if(pUSARTx == USART3)
				{
					USART3_PCLK_EN();
				}
				else if(pUSARTx == UART4)
				{
					UART4_PCLK_EN();
				}
				else if(pUSARTx == UART5)
				{
					UART5_PCLK_EN();
				}
				else if(pUSARTx == USART6)
				{
					USART6_PCLK_EN();
				}
			}

		else
		{
			if(pUSARTx == USART1)
			{
				USART1_PCLK_DI();
			}
			else if(pUSARTx == USART2)
			{
				USART2_PCLK_DI();
			}
			else if(pUSARTx == USART3)
			{
				USART3_PCLK_DI();
			}
			else if(pUSARTx == UART4)
			{
				UART4_PCLK_DI();
			}
			else if(pUSARTx == UART5)
			{
				UART5_PCLK_DI();
			}
			else if(pUSARTx == USART6)
			{
				USART6_PCLK_DI();
			}
	}
}

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{

}
void USART_DeInit(USART_RegDef_t *pUSARTx)
{

}


/*
 * Data Send and Receive
 */
void USART_SendData(USART_RegDef_t *pUSARTx,uint8_t *pTxBuffer, uint32_t Len)
{

}
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len)
{

}
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{

}
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{

}

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}
void USART_IRQHandling(USART_Handle_t *pHandle)
{

}

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{

}
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{

}
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{

}

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{

}
