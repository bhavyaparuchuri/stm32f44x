/*
 * 006i2c_master_tx_testing.c
 *
 *  Created on: 01-Dec-2020
 *      Author: bhavya
 */
#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

#define SLAVE_ADDR 		0x68
#define MY_ADDR 		0x61
void delay(void)
{
	for(uint32_t i=0 ; i < 500000 ; i++ );
}
I2C_Handle_t I2C1Handle;

//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";

/*
 * PBG-> SCL
 * PB9-> SDA
 */


void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
		GPIOBtn.pGPIOx = GPIOC;
		GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
		GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_Init(&GPIOBtn);

		//this is led gpio configuration
		GpioLed.pGPIOx = GPIOA;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
		GpioLed.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

		GPIO_PeriClockControl(GPIOA, ENABLE);

		GPIO_Init(&GpioLed);
}

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

		I2CPins.pGPIOx = GPIOB;
		I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
		I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
		I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
		I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
		I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

		//SCL
		I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
		GPIO_Init(&I2CPins);

		//SDA
		I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
		GPIO_Init(&I2CPins);

}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle= I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

int main()
{
	GPIO_ButtonInit();
	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);
	while(1)
	{
		//wait till button is pressed
		while( !GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		//to avoid button de-bouncing  related issues 200ms of delay
		delay();

		//Send some data to the slave
		I2C_MasterSendData(&I2C1Handle,some_data,strlen((char*)some_data),SLAVE_ADDR);
	}
}
