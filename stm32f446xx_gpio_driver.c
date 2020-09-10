/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Jul 10, 2020
 *      Author: Amey
 */


#include "stm32f446xx_gpio_driver.h"
/**************************
@fn      - GPIO_PeriClockControl
@brief   -  This function enables or disables peripheral clock for given GPIO port.
param1   -  base address of GPIO Peripheral
param2   -  Enable /Disable macros.
param3   -

@return  -  none

*****************************/
void GPIO_PeriClockControl(GPIO_Regdef_t *pGPIOx,uint8_t EnorDi  )//enable and disable clk for given base address
{
    if(EnorDi==ENABLE)
    {
    	if(pGPIOx==GPIOA)
    	{
    		GPIOA_PCLK_EN();
    	}else if(pGPIOx==GPIOB)
    	{
    		GPIOB_PCLK_EN();
    	}else if(pGPIOx==GPIOC)
    	{
    		GPIOC_PCLK_EN();
    	}else if(pGPIOx==GPIOD)
    	{
    		GPIOD_PCLK_EN();
    	}else if(pGPIOx==GPIOE)
    	{
    		GPIOE_PCLK_EN();
    	}else if(pGPIOx==GPIOF)
    	{
    		GPIOF_PCLK_EN();
    	}else if(pGPIOx==GPIOG)
    	{
    		GPIOG_PCLK_EN();
    	}else if(pGPIOx==GPIOH)
    	{
    		GPIOH_PCLK_EN();
    	}
    else
    	if(pGPIOx==GPIOA)
    	       {
    	       	GPIOA_PCLK_DI();
    	    	}else if(pGPIOx==GPIOB)
    	    	{
    	    		GPIOB_PCLK_DI();
    	    	}else if(pGPIOx==GPIOC)
    	    	{
    	    		GPIOC_PCLK_DI();
    	    	}else if(pGPIOx==GPIOD)
    	    	{
    	    		GPIOD_PCLK_DI();
    	    	}else if(pGPIOx==GPIOE)
    	    	{
    	    		GPIOE_PCLK_DI();
    	    	}else if(pGPIOx==GPIOF)
    	    	{
    	    		GPIOF_PCLK_DI();
    	    	}else if(pGPIOx==GPIOG)
    	    	{
    	    		GPIOG_PCLK_DI();
    	    	}else if(pGPIOx==GPIOH)
    	    	{
    	    		GPIOH_PCLK_DI();
    	    	}
    }


}
//Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)//initialize registers of given GPIO
{
    uint32_t temp=0;

    //Enable GPIO Clock
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1.Configure mode of GPIO.
    //of 2 bits as we see ref manual so multiplied by 2 and shift
     if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
     {
        //non interupt  mode
    	 temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    	 pGPIOHandle->pGPIOx->MODER &=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing bit =11(3)
    	 pGPIOHandle->pGPIOx->MODER |=temp;  //OR otherwise all bits will shift

     }else
     {
         //interupt mode
    	 if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT)//..interupt falling edge
    	 {
           // Configure the FTSR register
    		 EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		 //below for clearing RTSR if by default it is ON..
    		 EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    	 }else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT)
    	 {    //configure the RTSR
    		 EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		 EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    	 }else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT)
    	 {    //configure both FTSR and RTSR
    		 EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		 EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	 }
    	 //2.configure the GPIO port selection in SYSCFG,EXTICR
    	 uint8_t temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
    	 uint8_t temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
    	 uint8_t portcode=GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
    	 SYSCFG_PCLK_EN();//clock enable for peripheral
    	 SYSCFG->EXTICR[temp1]= portcode << (temp2 * 4);
    	 //SYSCFG_PCLK_EN();//clock enable for peripheral
    	 //SYSCFG->EXTICR[temp1]=portcode <<(temp2 *4);
    	 //3.enable EXTI interupt delivery using IMR.
    	    EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
     }
     temp=0;
     //2.Configure the speed.
     temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
     pGPIOHandle->pGPIOx->OSPEEDR &=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
     pGPIOHandle->pGPIOx->OSPEEDR|=temp;

     temp=0;


	//3.Configure the pupd settings.
     temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
     pGPIOHandle->pGPIOx->PUPDR &=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
     pGPIOHandle->pGPIOx->PUPDR|=temp;

     temp=0;
	//4.Configure the optype.
     temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
     pGPIOHandle->pGPIOx->OTYPER &=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
     pGPIOHandle->pGPIOx->OTYPER|=temp;

     temp=0;
	//5.Configure the alt functionality.
     if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN)
     {
         uint8_t temp1,temp2;
         temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
         temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
         pGPIOHandle->pGPIOx->AFR[temp1]&=~(0xF<<(4*temp2));//clearing bit
         pGPIOHandle->pGPIOx->AFR[temp1]|=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2));

     }

}
void GPIO_DeInit(GPIO_Regdef_t *pGPIOx)   //deinitialize means sending it back to reset all registers state
                                          //peripheral reset register is used to pg 136
{
	if(pGPIOx==GPIOA)
	    	{
	    		GPIOA_REG_RESET();
	    	}else if(pGPIOx==GPIOB)
	    	{
	    		GPIOB_REG_RESET();
	    	}else if(pGPIOx==GPIOC)
	    	{
	    		GPIOC_REG_RESET();
	    	}else if(pGPIOx==GPIOD)
	    	{
	    		GPIOD_REG_RESET();
	    	}else if(pGPIOx==GPIOE)
	    	{
	    		GPIOE_REG_RESET();
	    	}else if(pGPIOx==GPIOF)
	    	{
	    		GPIOF_REG_RESET();
	    	}else if(pGPIOx==GPIOG)
	    	{
	    		GPIOG_REG_RESET();
	    	}else if(pGPIOx==GPIOH)
	    	{
	    		GPIOH_REG_RESET();
	    	}



}
//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_Regdef_t *pGPIOx,uint8_t PinNumber)
{
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR>>PinNumber)& 0x00000001); //IDR is shifted by value asssigned to pin number
	return value;                                 //and mask all other bit positions

}

uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t *pGPIOx)//returns content of input data register of 16 pins
{
	uint16_t value;
	value=(uint16_t)pGPIOx->IDR; // as we need to read from entire port
	return value;

}
void GPIO_WritetoOutputPin(GPIO_Regdef_t *pGPIOx,uint8_t PinNumber,uint8_t value)//value can be set/reset
{
   if(value==GPIO_PIN_SET)
   {
	//write 1 to the output data register at bit field corresponding to pin number
    pGPIOx->ODR|=(1<<PinNumber); //setting bit
   }
   else
   {
	   pGPIOx->ODR &=~(1<<PinNumber);//clearing bit
    //write 0
   }
}
void GPIO_WritetoOutputPort(GPIO_Regdef_t *pGPIOx,uint16_t value)
{
	pGPIOx->ODR=value;//writing value to the whole port
}


void GPIO_ToggleOutputPin(GPIO_Regdef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR^=(1<<PinNumber);  //toggle bit field when functn is called..

}

//IRQ configuration and ISR handling
void GPIO_IRQITConfig(uint8_t IRQNumber,uint8_t EnorDi)//this is processor side
{
   if(EnorDi==ENABLE)
   {
	   if(IRQNumber<=32)
	   {//program ISER0 register
          *NVIC_ISER0|=(1 << IRQNumber);

	   }else if (IRQNumber >31 && IRQNumber <64)
	   {//program ISER1 register
          *NVIC_ISER1|=(1 << (IRQNumber%32));
	   }else if (IRQNumber >=64 && IRQNumber <96)
	   {//program ISER2 register.
          *NVIC_ISER3|=(1 <<(IRQNumber%64));
	   }
   }else
   {
	   if(IRQNumber<=32)
	   	   {
             *NVIC_ICER0 |=(1 << IRQNumber);
	   	   }else if (IRQNumber >31 && IRQNumber <64)
	   	   {
	   		*NVIC_ICER1 |=(1 << (IRQNumber%32));
	   	   }else if (IRQNumber >=64 && IRQNumber <96)
	   	   {
	   		*NVIC_ICER2 |=(1 << (IRQNumber%64));
	   	   }
   }

}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
   //1.first lets find out IRQ registers.
	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section=IRQNumber % 4;

	//uint8_t shift_amount=(8 * iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR+(iprx*4))|= IRQPriority << (8* iprx_section);

}
void GPIO_IRQHandling(uint8_t PinNumber)//as IRQ handling must know from which pin interrupt is triggered.
{
   if(EXTI->PR & (1<<PinNumber))
   {
	   EXTI->PR |= (1<<PinNumber);  //clearing is writing 1
   }

}
