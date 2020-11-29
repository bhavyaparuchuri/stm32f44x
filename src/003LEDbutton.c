/*
 * 003LEDbutton.c
 *
 *  Created on: 23-Nov-2020
 *      Author: bhavya
 */
/*
 * Write a program to toggle the on board LED whenever the on board button is pressed.
 *
 * If you are using discovery board, When you press the button, Pin will be high
 * when you release the button, Pin will be low
 *
 * If you are using nucleo board, when you press the button, Pin will be low (pulled to GND)
 * when you release the button, Pin will be High (Pulled to VCC)
 */
/*
 * 001LEDtoggle.c
 *
 *  Created on: 10-Nov-2020
 *      Author: bhavi
 */
#include "stm32f446xx.h"

#define LOW 0
#define HIGH 1

#define BTN_PRESSED LOW

void delay(void)
{
	for(uint32_t i=0 ; i < 500000 ; i++ );
}
int main()
{
	GPIO_Handle_t GpioLed, GPIOBtn;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOBtn);
	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) == BTN_PRESSED)
		{
			delay();// with delay there will be button debouncing
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5 );
		}
	}
}



