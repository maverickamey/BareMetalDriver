/*
 * stm32f446xx.h
 *
 *  Created on: Jul 9, 2020
 *      Author: Amey
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_
#include <stdint.h>            //used for 32 bit as different platforms could have 2bytes/4bytes so generic 4 bytes
#include <stddef.h>
#define __vo                    volatile  //defined to as can't be modified
#define __weak 					__attribute__((weak))

/***********************Processor specific details***********************************************/
/*ARM CORTEX MX Processor NVIC ISERx register addresses*/

#define NVIC_ISER0              ((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1              ((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2              ((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3              ((__vo uint32_t *)0xE000E10C)

/*ARM CORTEX MX Processor NVIC ICERx register addresses*/
#define NVIC_ICER0              ((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1              ((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2              ((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3              ((__vo uint32_t *)0xE000E18C)

/*ARM CORTEX MX Processor Priority Register addresses*/
#define NVIC_PR_BASEADDR        ((__vo uint32_t *)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED           4

//base address of FLASH and SRAM
#define FLASH_BASEADDR      	0x08000000U
#define SRAM1_BASEADDR      	0x20000000U
#define SRAM2_BASEADDR  		0x20001C00U    //SRAM1+112 BYTES (CONVERT TO HEX)
#define ROM_BASEADDR			0x1FFF0000U     //system memory
#define SRAM					SRAM1_BASEADDR

#define PERIPH_BASEADDR				0X40000000U
#define APB1PERIPH_BASEADDR 		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR 		0x50000000U

//Base address of peripherals which are hanging on AHB1 bus.

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR+0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR+0x0400)
#define GPIOC_BASEADDR          (AHB1PERIPH_BASEADDR+0x0800)
#define GPIOD_BASEADDR          (AHB1PERIPH_BASEADDR+0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR+0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR+0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR+0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR+0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR+0x2000)

#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR+0x3800)

//Base address of APB1 bus

#define I2C1_BASEADDR 			(APB1PERIPH_BASEADDR+0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR+0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR+0x5C00)
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR+0x3800)
#define SPI3_BASEADDR           (APB1PERIPH_BASEADDR+0x3C00)
#define USART2_BASEADDR         (APB1PERIPH_BASEADDR+0x4400)
#define USART3_BASEADDR         (APB1PERIPH_BASEADDR+0x4800)
#define UART4_BASEADDR          (APB1PERIPH_BASEADDR+0x4C00)
#define UART5_BASEADDR          (APB1PERIPH_BASEADDR+0x5000)

//Base address of APB2 bus

#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR+0x3000)
#define USART1_BASEADDR         (APB2PERIPH_BASEADDR+0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR+0x1400)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR+0x3C00)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR+0x3800)

/*Registers of peripherals are specific to MCU
 * eg:number of registers of SPI peripheral of STM32 family of MCU may be different(more /less)
 * Compared to the number of registers of SPI peripheral of STM32LX or STM32F0x family of MCUs.
 * Please check your device RM
//peripheral registers definition structures
*/
typedef struct
{
	__vo uint32_t MODER;         //GPIO port mode register  			address offset:0x00
	__vo uint32_t OTYPER;        //GPIO port output type register   	address offset:0x04
	__vo uint32_t OSPEEDR;       //GPIO port output speed register  	address offset:0x08
	__vo uint32_t PUPDR;         //GPIO port pull-up/pull-down register address offset:0x0C
	__vo uint32_t IDR;           //GPIO port input data register        address offset:0x10
	__vo uint32_t ODR;           //GPIO port output data register		address offset:0x14
	__vo uint32_t BSRR;          //GPIO port bit set/reset register     address offset:0x18
	__vo uint32_t LCKR;          //GPIO port configuration lock register address offset:0x1C
	__vo uint32_t AFR[32];      //GPIO alternate function low register  address offset:0x20
}GPIO_Regdef_t;

typedef struct
{
	__vo uint32_t CR;            //RCC clock control register            address offset:0x00
	__vo uint32_t PLLCFGR;       //RCC PLL configuration register        address offset:0x04
	__vo uint32_t CFGR;          //RCC clock configuration register      address offset:0x08
	__vo uint32_t CIR;           //RCC clock interrupt register          address offset:0x0C
	__vo uint32_t AHB1RSTR;      //RCC AHB1 peripheral reset register    address offset:0x10
	__vo uint32_t AHB2RSTR;      //RCC AHB2 peripheral reset register    address offset:0x14
	__vo uint32_t AHB3RSTR;      //RCC AHB3 peripheral reset register    address offset:0x18
	     uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;      //RCC APB1 peripheral reset register    address offset:0x20
	__vo uint32_t APB2RSTR;      //RCC APB2 peripheral reset register    address offset:0x24
	     uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;       //RCC AHB1 peripheral clock enable register address offset:0x30
	__vo uint32_t AHB2ENR;      //RCC AHB2 peripheral clock enable register address offset:0x34
	__vo uint32_t AHB3ENR;      //RCC AHB3 peripheral clock enable register address offset:0x38
	     uint32_t RESERVED2;
	__vo uint32_t APB1ENR;  //RCC APB1 peripheral clock enable register address offset:0x40
	__vo uint32_t APB2ENR;  //RCC APB2 peripheral clock enable register address offset:0x44
	     uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;    //RCC AHB1 peripheral clock enable in low power mode register  address offset:0x50
	__vo uint32_t AHB2LPENR;    //RCC AHB2 peripheral clock enable in low power mode register  address offset:0x54
	__vo uint32_t AHB3LPENR;    //RCC AHB3 peripheral clock enable in low power mode register  address offset:0x58
	     uint32_t RESERVED4;
    __vo uint32_t APB1LPENR;    //RCC APB1 peripheral clock enable in low power mode register   address offset:0x60
    __vo uint32_t APB2LPENR;   //RCC APB2 peripheral clock enabled in low power mode register   address offset:0x64
         uint32_t RESERVED5[2];
    __vo uint32_t BDCR;        //RCC Backup domain control register         address offset:0x70
    __vo uint32_t CSR;         //RCC clock control & status register        address offset:0x74
         uint32_t RESERVED6[2];
    __vo uint32_t SSCGR;      //RCC spread spectrum clock generation register address offset:0x80
    __vo uint32_t PLLI2SCFGR; //RCC PLLI2S configuration register           address offset:0x84
    __vo uint32_t PLLSAICFGR; //RCC PLL configuration register              address offset:0x88
    __vo uint32_t DCKCFGR;    //RCC Dedicated Clock Configuration Register  address offset:0x8C
    __vo uint32_t CKGATENR;   //RCC clocks gated enable register            address offset:0x90
    __vo uint32_t DCKCFGR2;   //RCC dedicated clocks configuration register 2 address offset:0x94

}RCC_Regdef_t;
//peripheral definition structure for EXTI//we create this to access registers
typedef struct
{
   __vo uint32_t IMR;    //Interrupt mask register              address offset:0x00
   __vo uint32_t EMR;    //Event mask register                  address offset:0x04
   __vo uint32_t RTSR;   //Rising trigger selection register    address offset:0x08
   __vo uint32_t FTSR;   //Falling trigger selection register   address offset:0x0C
   __vo uint32_t SWIER;  //Software interrupt event register    address offset:0x10
   __vo uint32_t PR;     //Pending register                     address offset:0x14

}EXTI_Regdef_t;

typedef struct
{
   __vo uint32_t CR1;    //Control register 1            		address offset:0x00
   __vo uint32_t CR2;    //Control register 2            		address offset:0x04
   __vo uint32_t SR;     //Status register               		address offset:0x08
   __vo uint32_t DR;     //Data register                 		address offset:0x0C
   __vo uint32_t CRCPR;  //CRC polynomial register       		address offset:0x10
   __vo uint32_t RXCRCR;  //RX CRC register						address offset:0x14
   __vo uint32_t TXCRCR;  //TX CRC register                     address offset:0x18
   __vo uint32_t I2SCFGR; //I2S configuration register			address offset:0x1C
   __vo uint32_t I2SPR;  //I2S prescaler register				address offset:0x20

}SPI_Regdef_t;

typedef struct
{
	__vo uint32_t CR1;		//Control register 1            		address offset:0x00
	__vo uint32_t CR2;		//Control register 2            		address offset:0x04
	__vo uint32_t OAR1;		//Own address register 1				address offset:0x08
	__vo uint32_t OAR2; 	//Own address register 2				address offset:0x0c
	__vo uint32_t DR;		//Data Register							address offset:0x10
	__vo uint32_t SR1;		//Status Register 1						address offset:0x14
	__vo uint32_t SR2;		//Status Register 2						address offset:0x18
	__vo uint32_t CCR;		//Control Register						address offset:0x1C
	__vo uint32_t TRISE;	//TRISE Register						address offset:0x20
	__vo uint32_t FLTR;		//FLTR Register							address offset:0x24

}I2C_Regdef_t;

typedef struct
{
	__vo uint32_t SR;		//Status Register						address offset:0x00
	__vo uint32_t DR;		//data register							address offset:0x04
	__vo uint32_t BRR;		//Baud rate register					address offset:0x08
	__vo uint32_t CR1;		//Control register 1					address offset:0x0C
	__vo uint32_t CR2;		//Control register 2					address offset:0x10
	__vo uint32_t CR3;		//Control register 2					address offset:0x14
	__vo uint32_t GTPR;		//Guard time and prescalar register		address offset:0x18

}USART_Regdef_t;



typedef struct
{
	__vo uint32_t MEMRMP;   //memory remap register                         address offset:0x00
	__vo uint32_t PMC;      //peripheral mode configuration register        address offset:0x04
	__vo uint32_t EXTICR[4];  //External interrupt configuration register 1 address offset:0x08-0x14
	     uint32_t RESERVED1[2];//                                           address offset:0X18-0x1C
	__vo uint32_t CMPCR;//Compensation cell control register                address offset:0x20
	     uint32_t RESERVED2[2];//                                           Reserved:0x24-0x28
	__vo uint32_t CFGR;//configuration register                             address offset:0x2C

}SYSCFG_Regdef_t;

//Peripheral Definition (Peripheral Base ADDRESS typecasted to xxx_RegDef_t)

#define GPIOA    	((GPIO_Regdef_t*)GPIOA_BASEADDR)
#define GPIOB       ((GPIO_Regdef_t*)GPIOB_BASEADDR)
#define GPIOC       ((GPIO_Regdef_t*)GPIOC_BASEADDR)
#define GPIOD       ((GPIO_Regdef_t*)GPIOD_BASEADDR)
#define GPIOE       ((GPIO_Regdef_t*)GPIOE_BASEADDR)
#define GPIOF       ((GPIO_Regdef_t*)GPIOF_BASEADDR)
#define GPIOG       ((GPIO_Regdef_t*)GPIOG_BASEADDR)
#define GPIOH       ((GPIO_Regdef_t*)GPIOH_BASEADDR)
#define GPIOI       ((GPIO_Regdef_t*)GPIOI_BASEADDR)


#define RCC         ((RCC_Regdef_t*)RCC_BASEADDR)

#define SPI1        ((SPI_Regdef_t*)SPI1_BASEADDR)
#define SPI2        ((SPI_Regdef_t*)SPI2_BASEADDR)
#define SPI3        ((SPI_Regdef_t*)SPI3_BASEADDR)

#define I2C1        ((I2C_Regdef_t*)SPI3_BASEADDR)
#define I2C2		((I2C_Regdef_t*)SPI3_BASEADDR)
#define I2C3        ((I2C_Regdef_t*)SPI3_BASEADDR)

#define USART1		((USART_Regdef_t*)USART1_BASEADDR)
#define USART2		((USART_Regdef_t*)USART2_BASEADDR)
#define USART3		((USART_Regdef_t*)USART3_BASEADDR)
#define USART6		((USART_Regdef_t*)USART6_BASEADDR)


#define EXTI        ((EXTI_Regdef_t*)EXTI_BASEADDR)

#define SYSCFG      ((SYSCFG_Regdef_t*)SYSCFG_BASEADDR)

//Clock enable macros for GPIOX peripherals

#define GPIOA_PCLK_EN()     (RCC->AHB1ENR|=(1<<0))   //for setting OR operation
#define GPIOB_PCLK_EN()     (RCC->AHB1ENR|=(1<<1))  //SETTING 1ST bit for GPIOB
#define GPIOC_PCLK_EN()     (RCC->AHB1ENR|=(1<<2))
#define GPIOD_PCLK_EN()     (RCC->AHB1ENR|=(1<<3))
#define GPIOE_PCLK_EN()     (RCC->AHB1ENR|=(1<<4))
#define GPIOF_PCLK_EN()     (RCC->AHB1ENR|=(1<<5))
#define GPIOG_PCLK_EN()     (RCC->AHB1ENR|=(1<<6))
#define GPIOH_PCLK_EN()     (RCC->AHB1ENR|=(1<<7))

//Clock enable macros for I2CX peripherals

#define I2C1_PCLK_EN()      (RCC->APB1ENR|=(1<<21))//setting 21st bit for APB1ENR
#define I2C2_PCLK_EN()      (RCC->APB1ENR|=(1<<22))//setting 22nd bit for APB1ENR
#define I2C3_PCLK_EN()      (RCC->APB1ENR|=(1<<23))//setting 23rd bit for APB1ENR



//Clock enable macros for SPIX peripherals
#define SPI1_PCLK_EN()      (RCC->APB2ENR|=(1<<12))
#define SPI4_PCLK_EN()      (RCC->APB2ENR|=(1<<13))
#define SPI2_PCLK_EN()      (RCC->APB1ENR|=(1<<14))
#define SPI3_PCLK_EN()      (RCC->APB1ENR|=(1<<15))



//Clock enable macros for USARTx peripherals
#define USART1_PCLK_EN()    (RCC->APB2ENR|=(1<<4))
#define USART6_PCLK_EN()    (RCC->APB2ENR|=(1<<5))

#define USART2_PCLK_EN()    (RCC->APB1ENR|=(1<<17))
#define USART3_PCLK_EN()    (RCC->APB1ENR|=(1<<18))
#define USART4_PCLK_EN()    (RCC->APB1ENR|=(1<<19))
#define USART5_PCLK_EN()    (RCC->APB1ENR|=(1<<20))



//Clock enable macros for SYSCFG peripherals
#define SYSCFG_PCLK_EN()    (RCC->APB2ENR|=(1<<14))


//Clock disable macros for GPIOX peripherals

#define GPIOA_PCLK_DI()     (RCC->AHB1ENR &=(1<<0))   //for disabling AND operation
#define GPIOB_PCLK_DI()     (RCC->AHB1ENR &=(1<<1))  //disabling 1ST bit for GPIOB
#define GPIOC_PCLK_DI()     (RCC->AHB1ENR &=(1<<2))
#define GPIOD_PCLK_DI()     (RCC->AHB1ENR &=(1<<3))
#define GPIOE_PCLK_DI()     (RCC->AHB1ENR &=(1<<4))
#define GPIOF_PCLK_DI()     (RCC->AHB1ENR &=(1<<5))
#define GPIOG_PCLK_DI()     (RCC->AHB1ENR &=(1<<6))
#define GPIOH_PCLK_DI()     (RCC->AHB1ENR &=(1<<7))

//Clock disable macros for I2CX peripherals

#define I2C1_PCLK_DI()      (RCC->APB1ENR &=(1<<21))//disabling 21st bit for APB1ENR
#define I2C2_PCLK_DI()      (RCC->APB1ENR &=(1<<22))//disabling 22nd bit for APB1ENR
#define I2C3_PCLK_DI()      (RCC->APB1ENR &=(1<<23))//disabling 23rd bit for APB1ENR



//Clock disable macros for SPIX peripherals
#define SPI1_PCLK_DI()      (RCC->APB2ENR &=(1<<12))
#define SPI4_PCLK_DI()      (RCC->APB2ENR &=(1<<13))
#define SPI2_PCLK_DI()      (RCC->APB1ENR &=(1<<14))
#define SPI3_PCLK_DI()      (RCC->APB1ENR &=(1<<15))



//Clock disable macros for USARTx peripherals
#define USART1_PCLK_DI()    (RCC->APB2ENR &=(1<<4))
#define USART6_PCLK_DI()    (RCC->APB2ENR &=(1<<5))

#define USART2_PCLK_DI()    (RCC->APB1ENR &=(1<<17))
#define USART3_PCLK_DI()    (RCC->APB1ENR &=(1<<18))
#define USART4_PCLK_DI()    (RCC->APB1ENR &=(1<<19))
#define USART5_PCLK_DI()    (RCC->APB1ENR &=(1<<20))



//Clock disable macros for SYSCFG peripherals
#define SYSCFG_PCLK_DI()    (RCC->APB2ENR &=(1<<14))

//MACROS to reset GPIOx peripherals
#define GPIOA_REG_RESET()    do{ RCC->AHB1RSTR|=(1<<0); RCC->AHB1RSTR &= ~(1 << 0);}while(0);
#define GPIOB_REG_RESET()    do{ RCC->AHB1RSTR|=(1<<0); RCC->AHB1RSTR &= ~(1 << 1);}while(0);
#define GPIOC_REG_RESET()    do{ RCC->AHB1RSTR|=(1<<0); RCC->AHB1RSTR &= ~(1 << 2);}while(0);
#define GPIOD_REG_RESET()    do{ RCC->AHB1RSTR|=(1<<0); RCC->AHB1RSTR &= ~(1 << 3);}while(0);
#define GPIOE_REG_RESET()    do{ RCC->AHB1RSTR|=(1<<0); RCC->AHB1RSTR &= ~(1 << 4);}while(0);
#define GPIOF_REG_RESET()    do{ RCC->AHB1RSTR|=(1<<0); RCC->AHB1RSTR &= ~(1 << 5);}while(0);
#define GPIOG_REG_RESET()    do{ RCC->AHB1RSTR|=(1<<0); RCC->AHB1RSTR &= ~(1 << 6);}while(0);
#define GPIOH_REG_RESET()    do{ RCC->AHB1RSTR|=(1<<0); RCC->AHB1RSTR &= ~(1 << 7);}while(0);
//some generic macros

#define GPIO_BASEADDR_TO_CODE(x)     ((x=GPIOA)? 0 :\
                                      (x=GPIOB)? 1 :\
                                      (x=GPIOC)? 2 :\
                                      (x=GPIOD)? 3 :\
                                      (x=GPIOE)? 4 :\
                                      (x=GPIOF)? 5 :\
                                      (x=GPIOG)? 6 :\
                                      (x=GPIOH)? 7 :0  )

//Macros to reset SPIx Peripherals
#define SPI1_REG_RESET()    do{ RCC->APB2RSTR|=(1<<0); RCC->APB2RSTR &= ~(1 << 0);}while(0);
#define SPI2_REG_RESET()    do{ RCC->APB1RSTR|=(1<<0); RCC->APB1RSTR &= ~(1 << 1);}while(0);
#define SPI3_REG_RESET()    do{ RCC->APB1RSTR|=(1<<0); RCC->APB1RSTR &= ~(1 << 2);}while(0);

//Macros to reset I2Cx Peripherals
#define I2C1_REG_RESET()    do{ RCC->APB1RSTR|=(1<<0); RCC->APB1RSTR &= ~(1 << 0);}while(0);
#define I2C2_REG_RESET()    do{ RCC->APB1RSTR|=(1<<0); RCC->APB1RSTR &= ~(1 << 1);}while(0);
#define I2C3_REG_RESET()    do{ RCC->APB1RSTR|=(1<<0); RCC->APB1RSTR &= ~(1 << 2);}while(0);


//Macros to reset USARTx Peripherals
#define USART1_REG_RESET()    do{ RCC->APB2RSTR|=(1<<0); RCC->APB2RSTR &= ~(1 << 0);}while(0);
#define USART2_REG_RESET()    do{ RCC->APB1RSTR|=(1<<0); RCC->APB1RSTR &= ~(1 << 1);}while(0);
#define USART3_REG_RESET()    do{ RCC->APB1RSTR|=(1<<0); RCC->APB1RSTR &= ~(1 << 2);}while(0);
#define USART4_REG_RESET()    do{ RCC->APB1RSTR|=(1<<0); RCC->APB1RSTR &= ~(1 << 0);}while(0);
#define USART5_REG_RESET()    do{ RCC->APB1RSTR|=(1<<0); RCC->APB1RSTR &= ~(1 << 1);}while(0);
#define USART6_REG_RESET()    do{ RCC->APB2RSTR|=(1<<0); RCC->APB2RSTR &= ~(1 << 2);}while(0);

//Interupt request numbers of STM32F446RE MCU
#define IRQ_NO_EXTI0     6
#define IRQ_NO_EXTI1     7
#define IRQ_NO_EXTI2     8
#define IRQ_NO_EXTI3     9
#define IRQ_NO_EXTI4     10
#define IRQ_NO_EXTI9_5   23
#define IRQ_NO_EXTI15_10 40
#define IRQ_NO_SPI1		 35
#define IRQ_NO_SPI2		 36
#define IRQ_NO_SPI3		 51
#define IRQ_NO_I2C1_EV	 31
#define IRQ_NO_I2C1_ER	 32



/*****************IRQ priorities LEVELS***************************/

#define NVIC_IRQ_PRI0     0
#define NVIC_IRQ_PRI1     1
#define NVIC_IRQ_PRI2     2
#define NVIC_IRQ_PRI3     3
#define NVIC_IRQ_PRI4     4
#define NVIC_IRQ_PRI5     5
#define NVIC_IRQ_PRI6     6
#define NVIC_IRQ_PRI7     7
#define NVIC_IRQ_PRI8     8
#define NVIC_IRQ_PRI9     9
#define NVIC_IRQ_PRI10    10
#define NVIC_IRQ_PRI11    11
#define NVIC_IRQ_PRI12    12
#define NVIC_IRQ_PRI13    13
#define NVIC_IRQ_PRI14    14
#define NVIC_IRQ_PRI15    15


#define ENABLE			 1
#define DISABLE 	 	 0
#define SET 		 	 ENABLE
#define RESET 		 	 DISABLE
#define GPIO_PIN_SET	 SET
#define GPIO_PIN_RESET	 RESET
#define FLAG_RESET       RESET
#define FLAG_SET         SET

//Bit position definitions of SPI_CR1 peripheral

#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR        2
#define SPI_CR1_BR          3
#define SPI_CR1_SPE         6
#define SPI_CR1_LSBFIRST    7
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RXONLY      10
#define SPI_CR1_DFF         11
#define SPI_CR1_CRCNEXT     12
#define SPI_CR1_CRCEN       13
#define SPI_CR1_BIDIOE      14
#define SPI_CR1_BIDIMODE    15

/*Bit position definitions of SPI_CR2 peripheral*/

#define SPI_CR2_RXDMAEN     0
#define SPI_CR2_TXDMAEN     1
#define SPI_CR2_SSOE        2
#define SPI_CR2_FRF         4
#define SPI_CR2_ERRIE       5
#define SPI_CR2_RXNEIE      6
#define SPI_CR2_TXEIE       7

/*Bit position definitions of SPI_SR*/

#define SPI_SR_RXNE         0
#define SPI_SR_TXE          1
#define SPI_SR_CHSIDE       2
#define SPI_SR_UDR          3
#define SPI_SR_CRCERR       4
#define SPI_SR_MODF         5
#define SPI_SR_OVR          6
#define SPI_SR_BSY          7
#define SPI_SR_FRE          8


/*Bit position definitions of I2C_CR1*/

#define I2C_CR1_PE			0   //Peripheral enable
#define I2C_CR2_ENPEC		5	//PEC ENABLE
#define I2C_CR1_ENGC		6	//General call enable ie 0=(disabled address NACK) and viceversa
#define I2C_CR1_NOSTRETCH	7	//Clock Stretching 0-Enable/1-disable
#define I2C_CR1_START		8	//
#define I2C_CR1_STOP		9	//
#define I2C_CR1_ACK			10	//1-ACK returned
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12	//Packet error checking//No PEC transfer
#define I2C_CR1_SWRST		15	//Software reset


/*Bit position definitons of I2C_CR2*/

#define I2C_CR2_FREQ		0
#define I2C_CR2_ITEREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11

/*Bit position definitions of I2C_OAR1*/

#define I2C_OAR1_ADD0 		0
#define I2C_OAR1_ADD71		1
#define I2C_OAR1_ADD98		8
#define I2C_OAR1_ADDMODE	15

/*Bit position definitions of I2C_SR1*/

#define I2C_SR1_SB			0	//Start Bit
#define I2C_SR1_ADDR		1	//Address sent
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6	//1=DATA REG not empty IN RECEIVER MODE
#define I2C_SR1_TXE			7	//1=DATA REg empty ie when DR is empty in transmission
#define I2C_SR1_BERR		8	//Bus error
#define I2C_SR1_ARLO		9	//1=arbitration loss detected
#define I2C_SR1_AF			10	//1=ack failure
#define I2C_SR1_OVR			11	//1=SLAVE MODE WHEN NOSTRETCH=1
#define I2C_SR1_PECERR		12	//1=Error in reception
#define I2C_SR1_TIMEOUT		14	//1=SCL remained low for 25ms

/*Bit position definitions of I2C_SR2*/

#define I2C_SR2_MSL			0	//Master=1
#define I2C_SR2_BUSY		1	//1=Communication ongoing on the bus
#define I2C_SR2_TRA			2	//1=Data bytes transmitted
#define I2C_SR2_GENCALL		4	//1=General call address rcvd when ENGC=1
#define I2C_SR2_DUALF		7	//1=Received address matched with OAR2
#define I2C_SR2_158			8	//Packet error checking register

/*Bit position definitions of I2C_CCR	*/

#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15

/*Bit position definitions of USART_SR*/
#define USART_SR_PE			0 	//Parity error then PE=1 else 0
#define USART_SR_FE			1	//Framing error the FE=1 else 0 ie no framing error
#define USART_SR_NF			2	//Noise detected flag NF=1 else 0
#define USART_SR_ORE		3	//Overrun error ORE=1 else 0
#define USART_SR_IDLE		4	//IDLE Line detected=1 else 0
#define USART_SR_RXNE		5	//rxne =1 data is ready to read or 0=data not received
#define USART_SR_TC			6	//TC=1 transmission is complete
#define USART_SR_TXE		7	//TXE=1 data trnsferred to shuift register or data is not transferred to sghift register
#define USART_SR_LBD		8 	//LBD=1 LIN break detected flag else 0
#define USART_SR_CTS		9	//CTS=1 change occured on nCTS status line

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"



#endif /* INC_STM32F446XX_H_ */
