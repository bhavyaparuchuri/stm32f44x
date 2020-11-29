/*
 *  * 002LEDtoggle.c
 *
 *  Created on: 23-Nov-2020
 *      Author: bhavya
 */
/*
 * 001LEDtoggle.c
 *
 *  Created on: 10-Nov-2020
 *      Author: bhavi
 */
//Here using open drain.
//In this case led is toggling, but intensity is very very small, when using internal
//pullup resistor. so give an external resistor of value 320 connected to vcc,this
//makes LED to toggle like normal
//To give an external pullup, connect the jumper wire from PA5 to VCC5 through resistor
// 320 or some value
#include "stm32f446xx.h"

void delay(void)
{
	for(uint32_t i=0 ; i < 500000 ; i++ );
}
int main()
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	//GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5 );
		delay();
	}
}





