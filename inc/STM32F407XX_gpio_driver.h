/*
 * STM32F407XX_gpio_driver.h
 *
 *  Created on: 30-Sep-2020
 *      Author: Training
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_
#include "stm32f407xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;
/*
 * Handle structure for GPIO pin
 */
typedef struct
{
	GPIO_REGDEF_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * GPIO_PinNumber of GPIOx
 */
#define GPIO_Pin_No_0 			0
#define GPIO_Pin_No_1 			1
#define GPIO_Pin_No_2 			2
#define GPIO_Pin_No_3 			3
#define GPIO_Pin_No_4 			4
#define GPIO_Pin_No_5 			5
#define GPIO_Pin_No_6 			6
#define GPIO_Pin_No_7 			7
#define GPIO_Pin_No_8 			8
#define GPIO_Pin_No_9 			9
#define GPIO_Pin_No_10 			10
#define GPIO_Pin_No_11			11
#define GPIO_Pin_No_12 			12
#define GPIO_Pin_No_13 			13
#define GPIO_Pin_No_14 			14
#define GPIO_Pin_No_15 			15

/*
 * GPIO_PinMode
 */

#define GPIO_Pin_Mode_IN		0 //Input
#define GPIO_Pin_Mode_OUT		1 //General_Purpose_Output_mode
#define GPIO_Pin_Mode_ALTFUN	2 //Alternate_Function_Mode
#define GPIO_Pin_Mode_ANGM		3 //Analog_Mode

/*
 * GPIO_PinSpeed
 */
#define  GPIO_Pin_Speed_LW 	0 //Low_speed
#define  GPIO_Pin_Speed_MD 	1 //Medium_speed
#define  GPIO_Pin_Speed_HG 	2 //High_speed
#define  GPIO_Pin_Speed_VH 	3 //Very_high_speed

/*
 * GPIO_PinOPType
 */
#define GPIO_PinOPType_push_pull 	0
#define GPIO_PinOPType_open_drain 	1

/*
 * GPIO_PinPuPdControl
 */
#define GPIO_PinPuPdControl_No_Pull_Push 		0 //No pull-up, pull-down
#define GPIO_PinPuPdControl_Pull_up 			1 //Pull-up
#define GPIO_PinPuPdControl_Pull_down			2 //Pull-down
#define GPIO_PinPuPdControl_Reserved			3 //Reserved

/*
 * GPIO_PinAltFunMode
 */

/*
 * GPIO APIs
 */
void GPIO_Init(GPIO_Handle_t *GPIOHandle); 													//Initialization of GPIO (A,B,C .... I)

void GPIO_DInit(GPIO_REGDEF_t *pGPIOx); 													//De-initialization

void GPIO_Peri_Clk_Ctrl(GPIO_REGDEF_t *pGPIOx, uint8_t EorDi); 								//Peripheral clock

uint8_t GPIO_Read_Frm_In_Pin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNo);							// Read from Input pin

uint16_t GPIO_Read_Frm_In_Port(GPIO_REGDEF_t *pGPIOx);										// Read from Input port

void GPIO_Write_To_Out_Pin(GPIO_REGDEF_t *pGPIOx ,  uint8_t PinNo , uint8_t value);			// Write to Output pin

void GPIO_Write_To_Out_Port(GPIO_REGDEF_t *pGPIOx , uint16_t value);						// Write to Output port

void GPIO_Toggle_Out_Pin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNo);								//toggle output

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
