/*
 * stm32f407xx.h
 *
 *  Created on: Sep 29, 2020
 *      Author: Training
 */

#include "STM32F407XX_gpio_driver.h"
#include<stdint.h> //Header file for uint32_t

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_



#define  FLASH_BASE_ADDR		0x08000000UL //Base address of the memory
#define SRAM1_BASE_ADDR 		0x20000000UL //BASE address of SRAM1
#define SRAM2_BASE_ADDR 		0x2001C000UL //BASE address of SRAM2
#define SRAM 					SRAM1_BASE_ADDR
#define ROM_BASE_ADDR			0x1FFF0000UL

		//Base address of the Bus
#define APB1_BASEADDR			0x40000000UL //Base address of the APB 1
#define APB2_BASEADDR			0x40000000UL //Base address of APB 2
#define AHB1_BASEADDR			0x40020000UL //Base address of AHB 1
#define AHB2_BASEADDR			0x50000000UL //Base address of AHB 2

		//Base address of the Peripherals hanging on AHB1

#define GPIOA_BASEADDR 			(AHB1_BASEADDR + 0x00UL) //Base + OFFSET
#define GPIOB_BASEADDR 			(AHB1_BASEADDR + 0x0400UL) //Base+ OFFSET
#define GPIOC_BASEADDR			(AHB1_BASEADDR + 0x0800UL) //Base+ OFFSET
#define GPIOD_BASEADDR 			(AHB1_BASEADDR + 0xC00UL)
#define GPIOE_BASEADDR 			(AHB1_BASEADDR + 0x1000UL)
#define GPIOF_BASEADDR          (AHB1_BASEADDR + 0x1400UL)
#define GPIOG_BASEADDR          (AHB1_BASEADDR + 0x1800UL)
#define GPIOH_BASEADDR          (AHB1_BASEADDR + 0x1C00UL)
#define GPIOI_BASEADDR          (AHB1_BASEADDR + 0x2000UL)
#define RCC_BASE_ADDR  			(AHB1_BASEADDR + 0x3800UL)

		//Base address of the Peripherals hanging on APB1
#define I2C1_BASEADDR   (APB1_BASEADDR + 0x5400UL)
#define I2C2_BASEADDR (APB1_BASEADDR + 0x5800UL)
#define I2C3_BASEADDR (APB1_BASEADDR + 0x5C00UL)
#define SPI2_BASEADDR (APB1_BASEADDR + 0x3800UL)
#define SPI3_BASEADDR (APB1_BASEADDR + 0x3C00UL)
#define USART2_BASEADDR (APB1_BASEADDR + 0x4400UL)
#define USART3_BASEADDR (APB1_BASEADDR + 0x4800UL)
#define USART4_BASEADDR (APB1_BASEADDR + 0x4C00UL)
#define USART5_BASEADDR (APB1_BASEADDR + 0x5000UL)


		//Base address of the Peripherals hanging on APB2
#define SPI1_BASEADDR (APB2_BASEADDR + 0x3000U)
#define USART1_BASEADDR (APB2_BASEADDR + 0x1000U)
#define USART6_BASEADDR (APB2_BASEADDR + 0x1400U)
#define EXTI_BASEADDR (APB2_BASEADDR + 0x3C00U)
#define SYSCFG_BASEADDR (APB2_BASEADDR + 0x3800U)


		// Register structure of GPIO peripherals
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
}GPIO_REGDEF_t;

#define GPIOA ((GPIO_REGDEF_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_REGDEF_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_REGDEF_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_REGDEF_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_REGDEF_t*)GPIOE_BASEADDR)
#define GPIOF ((GPIO_REGDEF_t*)GPIOF_BASEADDR)
#define GPIOG ((GPIO_REGDEF_t*)GPIOG_BASEADDR)
#define GPIOH ((GPIO_REGDEF_t*)GPIOH_BASEADDR)
#define GPIOI ((GPIO_REGDEF_t*)GPIOI_BASEADDR)
//Register structure of RCC
typedef struct
{

	volatile uint32_t 	CR;
	volatile uint32_t  	PLLCFGR;
	volatile uint32_t  	CFGR;
	volatile uint32_t  	CIR;
	volatile uint32_t  	AHB1RSTR;
	volatile uint32_t  	AHB2RSTR;
	volatile uint32_t  	AHB3RSTR;
	volatile uint32_t 	Reserved0;
	volatile uint32_t 	APB1RSTR;
	volatile uint32_t 	APB2RSTR;
	volatile uint32_t 	Reserved1[2];
	volatile uint32_t 	AHB1ENR;
	volatile uint32_t 	AHB2ENR;
	volatile uint32_t 	AHB3ENR;
	volatile uint32_t 	Reserved2;
	volatile uint32_t 	APB1ENR;
	volatile uint32_t  	APB2ENR;
	volatile uint32_t 	Reserved3[2];
	volatile uint32_t 	AHB1LPENR;
	volatile uint32_t 	AHB2LPENR;
	volatile uint32_t 	AHB3LPENR;
	volatile uint32_t 	Reserved4;
	volatile uint32_t 	APB1LPENR;
	volatile uint32_t 	APB2LPENR;
	volatile uint32_t 	Reserved5[2];
	volatile uint32_t 	BDCR;
	volatile uint32_t 	CSR;
	volatile uint32_t 	Reserved6[2];
	volatile uint32_t 	SSCGR;
	volatile uint32_t 	PLLI2SCFGR;
//volatile unint32_t 	PLLSAICFGR;
//volatile unint32_t 	DCKCFGR;
}RCC_REGDEF_t;


#define RCC	((RCC_REGDEF_t*) RCC_BASE_ADDR)


//Clock enable macros for GPIOx

#define GPIOA_PCLK_EN()		((RCC->AHB1ENR |=(1<<0)))
#define GPIOB_PCLK_EN()		((RCC->AHB1ENR |=(1<<1)))
#define GPIOC_PCLK_EN()		((RCC->AHB1ENR |=(1<<2)))
#define GPIOD_PCLK_EN()		((RCC->AHB1ENR |=(1<<3)))
#define GPIOE_PCLK_EN()		((RCC->AHB1ENR |=(1<<4)))
#define GPIOF_PCLK_EN()		((RCC->AHB1ENR |=(1<<5)))
#define GPIOG_PCLK_EN()		((RCC->AHB1ENR |=(1<<6)))
#define GPIOH_PCLK_EN()		((RCC->AHB1ENR |=(1<<7)))
#define GPIOI_PCLK_EN()		((RCC->AHB1ENR |=(1<<8)))

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1<<14))
//Clock disable macros for GPIOx

#define GPIOA_PCLK_DIS()	((RCC->AHB1ENR &=~(1<<0)))
#define GPIOB_PCLK_DIS()	((RCC->AHB1ENR &=~(1<<1)))
#define GPIOC_PCLK_DIS()	((RCC->AHB1ENR &=~(1<<2)))
#define GPIOD_PCLK_DIS()	((RCC->AHB1ENR &=~(1<<3)))
#define GPIOE_PCLK_DIS()	((RCC->AHB1ENR &=~(1<<4)))
#define GPIOF_PCLK_DIS()	((RCC->AHB1ENR &=~(1<<5)))
#define GPIOG_PCLK_DIS()	((RCC->AHB1ENR &=~(1<<6)))
#define GPIOH_PCLK_DIS()	((RCC->AHB1ENR &=~(1<<7)))
#define GPIOI_PCLK_DIS()	((RCC->AHB1ENR &=~(1<<8)))

/*
 * Some important Macros
 */

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_Pin_Set 	SET
#define GPIO_Pin_Reset 	REST

//GPIO reseting 1-> reset 0->No reset
#define GPIOA_REG_RESET() 	do{(RCC->AHB1RSTR|=(1<<0));  	(RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET() 	do{(RCC->AHB1RSTR|=(1<<1));		(RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET() 	do{(RCC->AHB1RSTR|=(1<<2));		(RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET() 	do{(RCC->AHB1RSTR|=(1<<3));		(RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET() 	do{(RCC->AHB1RSTR|=(1<<4));		(RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET() 	do{(RCC->AHB1RSTR|=(1<<5));		(RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET() 	do{(RCC->AHB1RSTR|=(1<<6));		(RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET() 	do{(RCC->AHB1RSTR|=(1<<7));		(RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET() 	do{(RCC->AHB1RSTR|=(1<<8));		(RCC->AHB1RSTR &= ~(1<<8));}while(0)


#endif /* INC_STM32F407XX_H_ */
