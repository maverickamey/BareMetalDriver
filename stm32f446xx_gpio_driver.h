/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Jul 10, 2020
 *      Author: Amey
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

typedef struct
{
   uint8_t GPIO_PinNumber;
   uint8_t GPIO_PinMode;  //input mode
   uint8_t GPIO_PinSpeed;
   uint8_t GPIO_PinPuPdControl;
   uint8_t GPIO_PinOPType;
   uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

//This is to handle structure of GPIO pin..
typedef struct
{
	GPIO_Regdef_t  *pGPIOx;      //This holds the base address of GPIO port to which pin belongs
   GPIO_PinConfig_t GPIO_PinConfig;     //This holds GPIO pin configuration settings
}GPIO_Handle_t;

//GPIO pins numbers...
#define GPIO_PIN_NO_0    0
#define GPIO_PIN_NO_1    1
#define GPIO_PIN_NO_2    2
#define GPIO_PIN_NO_3    3
#define GPIO_PIN_NO_4    4
#define GPIO_PIN_NO_5    5
#define GPIO_PIN_NO_6    6
#define GPIO_PIN_NO_7    7
#define GPIO_PIN_NO_8    8
#define GPIO_PIN_NO_9    9
#define GPIO_PIN_NO_10   10
#define GPIO_PIN_NO_11   11
#define GPIO_PIN_NO_12   12
#define GPIO_PIN_NO_13   13
#define GPIO_PIN_NO_14   14
#define GPIO_PIN_NO_15   15



/*
 * In input mode it is used to read value from IDR
 * Case 1:When it is ON THE OUTPUT DRIVER is properly connected to ground seeking no leakage /noise.(I/p data register value==0) properly grounded.

Case 2:But when switch goes OFF no button pressed then it is floating then value may fluctuate from 0 to 1 and so on and then the value is captured by input data register which is fluctuating.
So we use for case 2 mode the use of pullup /pulldown for proper grounding no value fluctuate
 */
//GPIO possible pin modes
//1.Configure mode of GPIO.
#define GPIO_MODE_IN      0
#define GPIO_MODE_OUT     1
#define GPIO_MODE_ALTFN   2
#define GPIO_MODE_ANALOG  3
#define GPIO_MODE_IT_FT   4
#define GPIO_MODE_IT_RT   5
#define GPIO_MODE_IT_RFT  6


//2.GPIO possible output types.
/*In push pull
 * Case 1:Writing 1

When we want to write to it ie when we give logic 1 at bit/set register then output control has a inverter then PMOS goes ON and then value is passed to the I/O pin through the LED.
Case2:Writing 0    When value 1 is output of inverter and which in turn will turn OFF LED as it is reverse biased.

In open drain
Case 1: Here we considering have set write as 0 then it will invert and I/O pin will be grounded and LED will not glow. Ie open drain state
Case 2:Here we are considering have set write as 1 but it will still not make it as 1 we need to have pull up register attached  ie small current will actually pass LED and forward biased but still LED will not glow for it to glow we need to add 1 more  external  register making LED ON.
 */

#define GPIO_OP_TYPE_PP   0
#define GPIO_OP_TYPE_OD   1

//3.GPIO possible output speeds types.
/*How fast the trise and tfall goes on
 *
 */
#define GPIO_SPEED_LOW    0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST   2
#define GPIO_SPEED_HIGH	  3

//4.GPIO Pull up and pull down
//CTRL PULL UP AND PULL DOWN
#define GPIO_NO_PUPD      0
#define GPIO_PU           1
#define GPIO_PD           2


//API supported by the driver..
//also create macro for enable and disable in.h file
//Peripheral Clock Setup,GPIO_Regdef_t *pGPIOx(enable and disable for given base address),EnorDi(Enable/Disable)
void GPIO_PeriClockControl(GPIO_Regdef_t *pGPIOx,uint8_t EnorDi  );//enable and disable clk for given base address

//Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);//initialize registers of given GPIO
void GPIO_DeInit(GPIO_Regdef_t *pGPIOx);   //deinitialize means sending it back to reset state
//peripheral reset register is used to pg 136


//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_Regdef_t *GPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t *GPIOx);//returns content of input data register of 16 pins
void GPIO_WritetoOutputPin(GPIO_Regdef_t *GPIOx,uint8_t PinNumber,uint8_t value);//value can be set/reset
void GPIO_WritetoOutputPort(GPIO_Regdef_t *GPIOx,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_Regdef_t *GPIOx,uint8_t PinNumber);

//IRQ configuration and ISR handling
void GPIO_IRQITConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);//as IRQ handling must know from which pin interrupt is triggered.




#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
