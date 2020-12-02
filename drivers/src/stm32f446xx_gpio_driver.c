/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 06-Nov-2020
 *      Author: bhavi
 */
#include "stm32f446xx_gpio_driver.h"
/*
 * Peripheral Clock Setup
 */
/******************************************************************************************************************
 * @fn 							-	GPIO_periclockcontrol
 *
 * @brief						-	This function enables or disables peripheral clock for this given GPIO port
 *
 * @param[in]					-	Base address of the gpio peripheral
 * @param[in]					-	ENABLE or DISABLE macros
 * @param[in]					-
 *
 * @return						-	none
 *
 * @Note						-	none

 *********************************************************************************************************************/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	/*	else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}*/
	}
	else{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		/*else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}*/
	}

}
/*
 * Init and De-init
 */
/******************************************************************************************************************
 * @fn 							-	GPIO_Init
 *
 * @brief						-	This function Initializes the given GPIO port pin
 *
 * @param[in]					-
 * @param[in]					-
 * @param[in]					-
 *
 * @return						-	none
 *
 * @Note						-	none

 *********************************************************************************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
/*Initialize given port pin*/
{
	uint32_t temp=0;//temp register
	//enable the peripheral clock
		GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non-interrupt modes
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx-> MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);// clearing
		pGPIOHandle->pGPIOx-> MODER |= temp;//setting
	}
	else
	{
		//interrupt modes
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
		{
			//1.configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
		{
			//1.configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Clear the corresponding RTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			//1.configure both FTSR & RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
						//Clear the corresponding RTSR bit
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}
		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2*4);

		//3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

	}
	temp = 0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx-> OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);// clearing
	pGPIOHandle->pGPIOx-> OSPEEDR |= temp;//setting

	temp = 0;
	//3. configure the pull up and pull down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx-> PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);// clearing
	pGPIOHandle->pGPIOx-> PUPDR |= temp;//setting

	temp = 0;
	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx-> OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);// clearing
	pGPIOHandle->pGPIOx-> OTYPER |= temp;

	temp = 0;
	//5. configure the alternate functionality
	uint8_t temp1,temp2;

	temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
	temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
	pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));//clearing
	pGPIOHandle->pGPIOx->AFR[temp1] |=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));//setting
}

/******************************************************************************************************************
 * @fn 							-	GPIO_DeInit
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

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
/*This is to de-Initialize : That means it restores the value back to default state*/
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}
/*	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}*/
/*
 * Data read and write
 */
/******************************************************************************************************************
 * @fn 							-	GPIO_ReadFromInputPin
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

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
/*To read from the pin*/
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) && 0x00000001);

	return value;
}
/******************************************************************************************************************
 * @fn 							-	GPIO_ReadFromInputPort
 *
 * @brief						-	This function read from given GPIO port
 *
 * @param[in]					-	Base address of the gpio port
 * @param[in]					-
 * @param[in]					-
 *
 * @return						-
 *
 * @Note						-	none

 *********************************************************************************************************************/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
/*To read from the port*/
{
	uint16_t value;
	value = (pGPIOx->IDR);

	return value;
}
/******************************************************************************************************************
 * @fn 							-	GPIO_WritetooutputPin
 *
 * @brief						-	This function write to the given GPIO pin
 *
 * @param[in]					-	Base address of the gpio port
 * @param[in]					-	pinNumber of given gpio port
 * @param[in]					-	value to write into respected port
 *
 * @return						-	0 or 1;
 *
 * @Note						-	none

 *********************************************************************************************************************/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
/*To write to the output pin*/
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the corresponding to the pin number.
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);

	}
}
/******************************************************************************************************************
 * @fn 							-	GPIO_writetooutputPort
 *
 * @brief						-	This function write to the given GPIO port
 *
 * @param[in]					-	Base address of the gpio port
 * @param[in]					-	Value to write into given gpio port
 * @param[in]					-
 *
 * @return						-
 *
 * @Note						-	none

 *********************************************************************************************************************/

void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
/*To write to the output port*/
{
	pGPIOx->ODR = Value;
}
/******************************************************************************************************************
 * @fn 							-	GPIO_ToggleOutputPin
 *
 * @brief						-	This function is to toggle the GPIO output pin
 *
 * @param[in]					-	Base address of the gpio port
 * @param[in]					-	pinNumber of given gpio port
 * @param[in]					-
 *
 * @return						-	0 or 1;
 *
 * @Note						-	none

 *********************************************************************************************************************/

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
/*To toggle the gpio output pin*/
{
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR handling
 */
/******************************************************************************************************************
 * @fn 							-	GPIO_IRQInterruptConfig
 *
 * @brief						-	This function is to manage the interrupt
 *
 * @param[in]					-	IRQ NUMBER of
 * @param[in]					-	enable or diable
 * @param[in]					-
 *
 * @return						-	none
 *
 * @Note						-	none

 *********************************************************************************************************************/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
/*Api to manage the Interrupt : Enabling,settingup priority*/
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
/******************************************************************************************************************
 * @fn 							-	GPIO_ToggleOutputPin
 *
 * @brief						-	This function
 *
 * @param[in]					-	Priority of IRQInterrupt
 * @param[in]					-
 * @param[in]					-
 *
 * @return						-	none
 *
 * @Note						-	none

 *********************************************************************************************************************/

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// 1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= ( IRQPriority << (8 * shift_amount));
}
/******************************************************************************************************************
 * @fn 							-	GPIO_IRQHandling
 *
 * @brief						-	This function is to process the interrupt
 *
 * @param[in]					-
 * @param[in]					-	pinNumber of given gpio port
 * @param[in]					-
 *
 * @return						-	none
 *
 * @Note						-	none

 *********************************************************************************************************************/

void GPIO_IRQHandling(uint8_t PinNumber)
/*To process the interrupt*/
{
	//clear the EXTI pr register corresponding to the pin number
	if(EXTI->PR &(1 << PinNumber))
	{
		//clear
		EXTI->PR |=(1 << PinNumber);
	}
}


