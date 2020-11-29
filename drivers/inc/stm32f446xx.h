/*
 * stm32f446xx.h
 *
 *  Created on: Nov 5, 2020
 *      Author: bhavi
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include<stdint.h>
#include<stddef.h>

#define __vo volatile

/*****************************************************START:Processor Specific Details********************************************/
/*
 * ARM Cortex Mx processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0				((__vo uint32_t*)0XE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0XE000E104)
#define NVIC_ISER2				((__vo uint32_t*)0XE000E108)
#define NVIC_ISER3				((__vo uint32_t*)0XE000E10C)

/*
 * ARM Cortex Mx processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0				((__vo uint32_t*)0XE000E100)
#define NVIC_ICER1				((__vo uint32_t*)0XE000E104)
#define NVIC_ICER2				((__vo uint32_t*)0XE000E108)
#define NVIC_ICER3				((__vo uint32_t*)0XE000E10C)

/*
 * ARM Cortex Mx processor Priority register Addresses
 */
#define NVIC_PR_BASE_ADDR				((__vo uint32_t*)0XE000E400)

#define NO_PR_BITS_IMPLEMENTED			4
/*
 * base address of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U			/* Flash base address */
#define SRAM1_BASEADDR			0x20000000U			/*SRAM1 base address */
#define SRAM2_BASEADDR			0x20001C00U			/*SRAM2 base address */
#define ROM_BASEADDR			0x1FFF0000U			/*ROM base address */
#define SRAM 					SRAM1_BASEADDR		/*The main internal ram is SRAM1*/

/*
 * AHBx and APBx Bus peripheral base address
 */

#define PERIPH_BASE				0x40000000			/*peripheral base address*/
#define APB1PERIPH_BASEADDR  	PERIPH_BASE			/*APB1 peripheral bus address is Initial address of peripherial base address*/
#define APB2PERIPH_BASEADDR		0x40010000U			/*APB2 peripheral base address*/
#define AHB1PERIPH_BASEADDR 	0x40020000U			/*AHB1 peripheral base address*/
#define AHB2PERIPH_BASEADDR 	0X50000000U			/*AHB2 peripheral base address*/
/*
 * Base Addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0000)			/*GPIO PORT-A base address */
#define GPIOB_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0400)			/*GPIO PORT-B base address */
#define GPIOC_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0800)			/*GPIO PORT-C base address */
#define GPIOD_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0C00)			/*GPIO PORT-D base address */
#define GPIOE_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1000)			/*GPIO PORT-E base address */
#define GPIOF_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1400)			/*GPIO PORT-F base address */
#define GPIOG_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1800)			/*GPIO PORT-G base address */
#define GPIOH_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1C00)			/*GPIO PORT-H base address */
//#define GPIOI_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x2000)			/*GPIO PORT-I base address */
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)          	/*RCC base address*/
/*
 * base Addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR          	(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)


#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)

#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)

/*
*base addresses of peripherals which are hanging on APB2 Bus
*/
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)

#define SPI1_BASEADDR		    (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR		    (APB2PERIPH_BASEADDR + 0x3400)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)


/*********************************peripheral register definition structure ****************************/

/*
 * NOTE : Registers of a peripherals are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different (more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your device RM
 */

typedef struct
{
	__vo uint32_t MODER;		/*GPIO port mode register 						Address offset : 0x00 */
	__vo uint32_t OTYPER;		/*GPIO port output type register 					Address offset : 0x04 */
	__vo uint32_t OSPEEDR;		/*GPIO port output speed register 					Address offset : 0x08 */
	__vo uint32_t PUPDR;		/*GPIO port pull-up/pull-down register 					Address offset : 0x0C */
	__vo uint32_t IDR;		/*GPIO port input data  register 					Address offset : 0x10*/
	__vo uint32_t ODR;		/*GPIO port output data register 					Address offset : 0x14 */
	__vo uint32_t BSRR;		/*GPIO port bit set/reset register 					Address offset : 0x18 */
	__vo uint32_t LCKR;		/*GPIO port configuration lock register 				Address offset : 0x1C */
	__vo uint32_t AFR[2];		/*GPIO port alternate function low AFR[0] & high AFR[1] registers 	Address offset : 0x20&0X24*/
}GPIO_RegDef_t;

/*
 * Peripheral register definition Structure for RCC
 */
typedef struct
{
	__vo uint32_t CR;					/*RCC clock control register 															Address offset :0x00 */
	__vo uint32_t PLLCFGR;					/*RCC PLL configuration register 														Address offset :0x04*/
	__vo uint32_t CFGR;			                /*RCC clock configuration register 														Address offset : 0x08*/
	__vo uint32_t CIR;					/*RCC clock interrupt register 															Address offset : 0x0C*/
	__vo uint32_t AHB1RSTR;					/*RCC AHB1 peripheral reset register 														Address offset : 0x10*/
	__vo uint32_t AHB2RSTR;					/*RCC AHB2 peripheral reset register 														Address offset : 0x14*/
	__vo uint32_t AHB3RSTR;					/*RCC AHB3 peripheral reset register 														Address offset : 0x18*/
	uint32_t RESERVED0;					/*RCC reversed register 															Address offset : 0x1C*/
	__vo uint32_t APB1RSTR;					/*RCC APB1 peripheral reset register 														Address offset : 0x20*/
	__vo uint32_t APB2RSTR;					/*RCC APB2 peripheral reset register 														Address offset : 0x24*/
	uint32_t RESERVED1[2];					/*RCC reversed register 															Address offset : 0x28*/
	__vo uint32_t AHB1ENR;					/*RCC AHB1 peripheral clock enable register 													Address offset : 0x30*/
	__vo uint32_t AHB2ENR;					/*RCC AHB2 peripheral clock enable register 													Address offset : 0x34*/
	__vo uint32_t AHB3ENR;					/*RCC AHB3 peripheral clock enable register 													Address offset : 0x38*/
	uint32_t RESERVED2;					/*RCC resersed register 															Address offset : 0x3C*/
	__vo uint32_t APB1ENR;					/*RCC APB1 peripheral clock enable register 													Address offset : 0x40*/
	__vo uint32_t APB2ENR;					/*RCC APB2 peripheral clock enable register 													Address offset : 0x44*/
	uint32_t RESERVED3[2];					/*RCC reserved register 															Address offset : 0x4C*/
	__vo uint32_t AHB1LPENR;				/*RCC AHB1 peripheral clock enable in low power moderegister 											Address offset : 0x50*/
	__vo uint32_t AHB2LPENR;				/*RCC AHB2 peripheral clock enable in low power moderegister 											Address offset : 0x54*/
	__vo uint32_t AHB3LPENR;				/*RCC AHB3 peripheral clock enable in low power moderegister 											Address offset : 0x58*/
	uint32_t RESERVED4;					/*RCC reserved register 															Address offset : 0x5C*/
	__vo uint32_t APB1LPENR;				/*RCC APB1 peripheral clock enable in low power moderegister 											Address offset : 0x60*/
	__vo uint32_t APB2LPENR;				/*RCC APB2 peripheral clock enable in low power moderegister 											Address offset : 0x64*/
	uint32_t RESERVED5[2];					/*RCC reserved register 															Address offset : 0x68*/
	__vo uint32_t BDCR;					/*RCC Backup domain control register 														Address offset : 0x74*/
	__vo uint32_t CSR;					/*RCC clock control & status register 														Address offset : 0x78*/
	uint32_t RESERVED6[2];					/*RCC reserved register 															Address offset : 0x7C*/
	__vo uint32_t SSCGR;					/*RCC  spread spectrum clock generation register 												Address offset : 0x80*/
	__vo uint32_t PLLI2SCFGR;				/*RCC PLLI2S configuration register 														Address offset :0x84*/
	__vo uint32_t PLLSAICFGR;				/*RCC PLL configuration register 														Address offset : 0x88*/
	__vo uint32_t DCKCFGR;					/*RCC Dedicated Clock Configuration register 													Address offset : 0x8C*/
	__vo uint32_t CKGATENR;					/*RCC clocks gated enable register 														Address offset : 0x90*/
	__vo uint32_t DCKCFGR2;					/*RCC dedicated clocks configuration register													Address offset : 0x94*/
}RCC_RegDef_t;


/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;		/*					Address offset : 0x00 */
	__vo uint32_t EMR;		/* 					Address offset : 0x04 */
	__vo uint32_t RTSR;		/* 					Address offset : 0x08 */
	__vo uint32_t FTSR;		/* 					Address offset : 0x0C */
	__vo uint32_t SWIER;	/* 					Address offset : 0x10*/
	__vo uint32_t PR;		/* 					Address offset : 0x14 */
}EXTI_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t MEMRMP;		/*					Address offset : 0x00 */
	__vo uint32_t PCM;		/* 					Address offset : 0x04 */
	__vo uint32_t EXTICR[4];		/* 					Address offset : 0x08 */
	__vo uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;		/* 					Address offset : 0x0C */
	__vo uint32_t RESERVED2[2];
	__vo uint32_t CFGR;	/* 					Address offset : 0x10*/
}SYSCFG_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;		/* 					Address offset : 0x00 */
	__vo uint32_t CR2;		/* 					Address offset : 0x04 */
	__vo uint32_t SR;		/* 					Address offset : 0x08 */
	__vo uint32_t DR;		/* 					Address offset : 0x0C */
	__vo uint32_t CRCPR;		/* 				Address offset : 0x10*/
	__vo uint32_t RXCRCR;		/* 				Address offset : 0x14 */
	__vo uint32_t TXCRCR;		/* 				Address offset : 0x18 */
	__vo uint32_t I2SCFGR;		/* 				Address offset : 0x1C */
	__vo uint32_t I2SPR;		/* 	Address offset : 0x20&0X24*/
	}SPI_RegDef_t;

/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
	__vo uint32_t CR1;		/* 					Address offset : 0x00 */
	__vo uint32_t CR2;		/* 					Address offset : 0x04 */
	__vo uint32_t OAR1;		/* 					Address offset : 0x08 */
	__vo uint32_t OAR2;		/* 					Address offset : 0x0C */
	__vo uint32_t DR;		/* 					Address offset : 0x10*/
	__vo uint32_t SR1;		/* 					Address offset : 0x14 */
	__vo uint32_t SR2;		/* 					Address offset : 0x18 */
	__vo uint32_t CCR;		/* 					Address offset : 0x */
	__vo uint32_t TRISE;	/*					Address offset : */
	__vo uint32_t FLTR;		/*					Address offset : */
}I2C_RegDef_t;

typedef struct
{
		__vo uint32_t SR;		/* 		Address offset : 0x04 */
		__vo uint32_t DR;		/* 		Address offset : 0x08 */
		__vo uint32_t BRR;		/* 		Address offset : 0x0C */
		__vo uint32_t CR1;		/* 		Address offset : 0x10 */
		__vo uint32_t CR2;		/* 		Address offset : 0x14 */
		__vo uint32_t CR3;		/* 		Address offset : 0x18 */
		__vo uint32_t GTPR;		/* 		Address offset : 0x1C */
}USART_RegDef_t;

/*
 * peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA 	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 	((GPIO_RegDef_t*)GPIOH_BASEADDR)
//#define GPIOI 	((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC   	((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3	((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4	((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1	((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2	((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3	((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1	((USART_RegDef_t*)USART1_BASEADDR)
#define USART2	((USART_RegDef_t*)USART2_BASEADDR)
#define USART3	((USART_RegDef_t*)USART3_BASEADDR)
#define UART4	((USART_RegDef_t*)UART4_BASEADDR)
#define UART5	((USART_RegDef_t*)UART5_BASEADDR)
#define USART6	((USART_RegDef_t*)USART6_BASEADDR)


/*
 * Clock Enable Macro for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 0 ))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 1 ))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 2 ))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 3 ))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 4 ))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 5 ))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 6 ))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 7 ))
//#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 8 ))


/*
 * Clock Enable Macro for I2Cx peripherals
 */

#define I2C1_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 21 ))
#define I2C2_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 22 ))
#define I2C3_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 23 ))

/*
 * Clock Enable Macro for SPIx peripherals
 */

#define SPI1_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 12 ))
#define SPI2_PCLK_EN()	(RCC->APB1ENR |= ( 1 << 14 ))
#define SPI3_PCLK_EN()	(RCC->APB1ENR |= ( 1 << 15 ))
#define SPI4_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 13 ))

/*
 * Clock Enable Macro for USARTx peripherals
 */

#define USART1_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 4  ))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= ( 1 << 17 ))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= ( 1 << 18 ))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 5  ))


/*
 * Clock Enable Macro for UARTx peripherals
 */

#define UART4_PCLK_EN()	(RCC->APB1ENR |= ( 1 << 19 ))
#define UART5_PCLK_EN()	(RCC->APB1ENR |= ( 1 << 30 ))


/*
 * Clock Enable Macro for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 14  ))

/*
 * Clock Disable Macro for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 0 ))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 1 ))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 2 ))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 3 ))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 4 ))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 5 ))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 6 ))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 7 ))
//#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 8 ))

/*
 * Clock Disable Macro for I2Cx peripherals
 */

#define I2C1_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 21 ))
#define I2C2_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 22 ))
#define I2C3_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 23 ))

/*
 * Clock Disable Macro for SPIx peripherals
 */

#define SPI1_PCLK_DI()	(RCC->APB2ENR &= ~( 1 << 12 ))
#define SPI2_PCLK_DI()	(RCC->APB1ENR &= ~( 1 << 14 ))
#define SPI3_PCLK_DI()	(RCC->APB1ENR &= ~( 1 << 15 ))
#define SPI4_PCLK_DI()	(RCC->APB2ENR &= ~( 1 << 13 ))

/*
 * Clock Disable Macro for USARTx peripherals
 */

#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~( 1 << 4  ))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~( 1 << 17 ))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~( 1 << 18 ))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~( 1 << 5  ))


/*
 * Clock Disable Macro for UARTx peripherals
 */

#define UART4_PCLK_DI()	(RCC->APB1ENR &= ~( 1 << 19 ))
#define UART5_PCLK_DI()	(RCC->APB1ENR &= ~( 1 << 30 ))


/*
 * Clock Disable Macro for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~( 1 << 14  ))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1<<0));  (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1<<0));  (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOC_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1<<0));  (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOD_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1<<0));  (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOE_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1<<0));  (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOF_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1<<0));  (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOG_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1<<0));  (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOH_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1<<0));  (RCC->AHB1RSTR &= ~(1<<0));}while(0)
//#define GPIOI_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1<<0));  (RCC->AHB1RSTR &= ~(1<<0));}while(0)

#define GPIO_BASEADDR_TO_CODE(x)	   ((x == GPIOA) ? 0:\
										(x == GPIOB) ? 1:\
										(x == GPIOC) ? 2:\
										(x == GPIOD) ? 3:\
										(x == GPIOE) ? 4:\
										(x == GPIOF) ? 5:\
										(x == GPIOG) ? 6:\
										(x == GPIOH) ? 7:0	)

/*
 * IRQ(Interrupt Request) Numbers of STM32F44x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO : You may complete this list for other peripherals
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV
#define IRQ_NO_I2C2_ER
#define IRQ_NO_I2C3_EV
#define IRQ_NO_I2C3_ER



/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI15			15

//some generic macros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET 			SET
#define GPIO_PIN_RESET 			RESET
#define FLAG_RESET				RESET
#define FLAG_SET				SET

/**********************************************************************************************
 * Bit position definition of SPI peripheral
 **********************************************************************************************/
//CR1
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

//CR2
#define	SPI_CR2_TXEIE 		7
#define	SPI_CR2_RXNEIE 		6
#define	SPI_CR2_ERRIE 		5
#define	SPI_CR2_FRF 		4
#define	SPI_CR2_SSOE 		2
#define	SPI_CR2_TXDMAEN		1
#define SPI_CR2_RXDAMEN		0

//SR
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_CRCERR		3
#define SPI_SR_MODF			4
#define SPI_SR_OVR			5
#define SPI_SR_BSY			6
#define SPI_SR_FRE			7


/**********************************************************************************************
 * Bit position definition of I2C peripheral
 **********************************************************************************************/
//Bit position definition CR1
#define I2C_CR1_PE				0
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_SWRST			15

//Bit position definition CR2
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10

//Bit Position definition SR1
#define	I2C_SR1_SB 				0
#define	I2C_SR1_ADDR 			1
#define	I2C_SR1_BTF 			2
#define	I2C_SR1_ADD10 			3
#define	I2C_SR1_STOPF			4
#define	I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define	I2C_SR1_BERR 			8
#define	I2C_SR1_ARLO 			9
#define	I2C_SR1_AF				10
#define	I2C_SR1_OVR				11
#define I2C_SR1_TIMEOUT			14

//Bit Position definition SR2

#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_DUALF			7

//Bit Position definition I2C_CCR

#define I2C_SR_CCR_CCR			0
#define I2C_SR_CCR_DUTY			14
#define I2C_SR_CCR_FS			15

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"

#endif /* INC_STM32F446XX_H_ */

