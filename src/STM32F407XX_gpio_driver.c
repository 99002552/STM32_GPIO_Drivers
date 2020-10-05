/*
 * STM32F407XX_gpio_driver.c
 *
 *  Created on: 30-Sep-2020
 *      Author: Training
 */
#include "STM32F407XX_gpio_driver.h"

/*
 * GPIO APIs
 * @Brief description
 * @Function-
 *
 * @Param1-
 *
 *
 * @param2-
 *
 * @param3-
 *
 * @Definition-
 *
 * @Design by-
 *
 * @Date and Time-
 *
 */
void GPIO_Peri_Clk_Ctrl(GPIO_REGDEF_t *pGPIOx, uint8_t EorDi)
{
	if(EorDi == ENABLE)
	{
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
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
			//TO DO
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DIS();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DIS();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DIS();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DIS();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DIS();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DIS();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DIS();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DIS();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DIS();
		}
	}
}

/*
 * @Brief description
 * @Function-
 *
 * @Param1-
 *
 *
 * @param2-
 *
 * @param3-
 *
 * @Definition-
 *
 * @Design by-
 *
 * @Date and Time-
 *
 */

void GPIO_Init(GPIO_Handle_t *GPIOHandle) 													//Initialization of GPIO (A,B,C .... I)
{
	//1. Initializing MODE
	uint32_t temp=0;
	temp=(GPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	(GPIOHandle->pGPIOx->MODER) &=~(0x3<<(2*GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	GPIOHandle->pGPIOx->MODER|=temp;
	temp=0;

	//2. Configuration Speed
	temp = (GPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	(GPIOHandle->pGPIOx->OSPEEDR) &=~(0x3<<(2*GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	GPIOHandle->pGPIOx->OSPEEDR|=temp;
	temp=0;

	//3. Configuration Pull up or pull down
	temp = (GPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	(GPIOHandle->pGPIOx->PUPDR) &=~(0x3<<(2*GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	GPIOHandle->pGPIOx->PUPDR|=temp;
	temp=0;

	//4. Configuration Output type
	temp = (GPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<(GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	(GPIOHandle->pGPIOx->OTYPER) &=~(0x1<<GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	GPIOHandle->pGPIOx->OTYPER|=temp;
	temp=0;

	//5. Configuration  Alternate Function
	//Method 1
	/*if(GPIOHandle->GPIO_PinConfig.GPIO_PinNumber<=7)
	{
		temp = (GPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		GPIOHandle->pGPIOx->AFR[2] &= (0xF<<(4*GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		GPIOHandle->pGPIOx->AFR[2]|=temp;
		temp=0;
	}
	else
	{
		temp = (GPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		GPIOHandle->pGPIOx->AFR[1] &= (0xF<<(4*GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		GPIOHandle->pGPIOx->AFR[1]|=temp;
		temp=0;
	}*/

	//Method 2
	 if(GPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_Pin_Mode_ALTFUN)
	 {
		 uint32_t temp1=0;
		 uint32_t temp2=0;
		 temp1 = (GPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/8;
		 temp2 = (GPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%8;
		 GPIOHandle->pGPIOx->AFR[temp1] &=~(0xF<<(4*temp2));
		 GPIOHandle->pGPIOx->AFR[temp1]|= (GPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2));
	}
}

/*
 * @Brief description
 * @Function-
 *
 * @Param1-
 *
 *
 * @param2-
 *
 * @param3-
 *
 * @Definition-
 *
 * @Design by-
 *
 * @Date and Time-
 *
 */


void GPIO_DInit(GPIO_REGDEF_t *pGPIOx)													//De-initialization
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
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}


/*
 * @Brief description
 * @Function-
 *
 * @Param1-
 *
 *
 * @param2-
 *
 * @param3-
 *
 * @Definition-
 *
 * @Design by-
 *
 * @Date and Time-
 *
 */

uint8_t GPIO_Read_Frm_In_Pin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNo)						// Read from Input pin
{
	uint8_t val=0;
	val = (uint8_t)(pGPIOx->IDR>>PinNo) & (0X00000001);  //Masking
	return val;
}

/*
 * @Brief description
 * @Function-
 *
 * @Param1-
 *
 *
 * @param2-
 *
 * @param3-
 *
 * @Definition-
 *
 * @Design by-
 *
 * @Date and Time-
 *
 */

uint16_t GPIO_Read_Frm_In_Port(GPIO_REGDEF_t *pGPIOx)										// Read from Input port
{
	uint16_t val=0;
	val =(uint16_t)(pGPIOx->IDR);
	return val;
}

/*
 * @Brief description
 * @Function-
 *
 * @Param1-
 *
 *
 * @param2-
 *
 * @param3-
 *
 * @Definition-
 *
 * @Design by-
 *
 * @Date and Time-
 *
 */

void GPIO_Write_To_Out_Pin(GPIO_REGDEF_t *pGPIOx ,  uint8_t PinNo , uint8_t value)			// Write to Output pin
{
	if(value == GPIO_Pin_Set)
	{
		pGPIOx->ODR |= (1<<PinNo);
	}
	else
	{
		pGPIOx->ODR &=~(1<<PinNo);
	}
}

/*
 * @Brief description
 * @Function-
 *
 * @Param1-
 *
 *
 * @param2-
 *
 * @param3-
 *
 * @Definition-
 *
 * @Design by-
 *
 * @Date and Time-
 *
 */

void GPIO_Write_To_Out_Port(GPIO_REGDEF_t *pGPIOx , uint16_t value)						// Write to Output port
{
	pGPIOx->ODR = value;
}

/*
 * @Brief description
 * @Function-
 *
 * @Param1-
 *
 *
 * @param2-
 *
 * @param3-
 *
 * @Definition-
 *
 * @Design by-
 *
 * @Date and Time-
 *
 */

void GPIO_Toggle_Out_Pin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNo)
{
	pGPIOx->ODR ^= (1<<PinNo);
}
