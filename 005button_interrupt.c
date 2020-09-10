/*
 * 005button_interrupt.c
 *
 *  Created on: Jul 16, 2020
 *      Author: Amey
 */


#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx.h"

#define HIGH 1
#define LOW  0
#define BTN_PRESSED LOW

int main()
{

	GPIO_Handle_t GpioLed,GpioBtn;
	GpioLed.pGPIOx=GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx=GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13 ;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT; //INTERRUPT Falling Edge
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);


	GPIO_IRQITConfig(IRQ_NO_EXTI13_10, ENABLE);//we selected it as PC13 ie 13 refer notes for same
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI13_10,NVIC_IRQ_PRI15);

	while(1){;
	}

	//Button Code for STM32

}
void EXTI15_10_IRQHandler(void)
	{
		GPIO_IRQHandling(GPIO_PIN_NO_13);
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
	}
