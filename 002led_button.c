/*
 * 002led_button.c
 *
 *  Created on: Jul 12, 2020
 *      Author: Amey
 */

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx.h"

#define LOW 0
#define BTN_PRESSED LOW

void delay()
{
	for(int i=0;i<500000/2;i++);
}


int main(void)
{
	GPIO_Handle_t GpioLed,GPIOBtn;
	   GpioLed.pGPIOx=GPIOA;
	   GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	   GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	   GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	   GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	   GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;//GPIO_NO_PUPD=no toggle no LED blink but GPIO_PU=little

	   GPIO_PeriClockControl(GPIOA,ENABLE);
	   GPIO_Init(&GpioLed);
	   //GPIO_WritetoOutputPin(GPIOA,5,0);
	   GPIOBtn.pGPIOx=GPIOC;//
	       GPIOBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	       GPIOBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;//as button is input
	       GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	     //  GPIOBtn.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;No needed now as button as only in output mode
	       GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;//GPIO_NO_PUPD=no toggle no LED blink but GPIO_PU=little

	   	   GPIO_PeriClockControl(GPIOC,ENABLE);
	   	   GPIO_Init(&GPIOBtn);

while(1)

	{
	     if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)==BTN_PRESSED)
	     {
	    	 delay(); // delay is added to reduce debiuncing outherwise it will toggle every now and then
	    	 GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
	     }
		//delay();
		//GPIO_WritetoOutputPin(GPIOA,5,1);
		//GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		//GPIO_WritetoOutputPin(GPIOA, GPIO_PIN_NO_5, 0);
		delay();
	}

return 0;
}



