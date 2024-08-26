/********************************************************************************
 Author : CAC (CustomerApplications Center, Asia) 

 Date : 2012-01-04

 File name : AD7606.h

 Description : test the serial port operation of AD7606

 Hardware plateform : ADuC7026_DEMO_V1.2 + EVAL-AD7606EDZ_Rev.A1  	
********************************************************************************/

#ifndef AD7606_DRIVER_H
#define AD7606_DRIVER_H
#include <stm32f10x_rcc.h>              /* STM32F10x¿â */


/* AD7606*/
#define RCC_AD7606_CTRL					RCC_APB2Periph_GPIOB
#define GPIO_AD7606_CTRL_PORT			GPIOB
#define GPIO_AD7606_BUSY_Pin			GPIO_Pin_5   
#define GPIO_AD7606_RST_Pin			    GPIO_Pin_6
#define GPIO_AD7606_CNVST_Pin			GPIO_Pin_7


#define GPIO_AD7606_SCS_Pin		    	GPIO_Pin_7

#define AD7606_SCK_LOW				    GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define AD7606_SCK_HIGH					GPIO_SetBits(GPIOB, GPIO_Pin_9)												


#define AD7606_RST_LOW				    GPIO_ResetBits(GPIO_AD7606_CTRL_PORT, GPIO_AD7606_RST_Pin)
#define AD7606_RST_HIGH					GPIO_SetBits(GPIO_AD7606_CTRL_PORT, GPIO_AD7606_RST_Pin)												

#define AD7606_CNVST_LOW				GPIO_ResetBits(GPIO_AD7606_CTRL_PORT, GPIO_AD7606_CNVST_Pin)
#define AD7606_CNVST_HIGH				GPIO_SetBits(GPIO_AD7606_CTRL_PORT, GPIO_AD7606_CNVST_Pin)												

#define AD7606_SCS_LOW				    GPIO_ResetBits(GPIOA, GPIO_AD7606_SCS_Pin)
#define AD7606_SCS_HIGH					GPIO_SetBits(GPIOA, GPIO_AD7606_SCS_Pin)												

#define AD7606_BUSY_READ                GPIO_ReadInputDataBit(GPIO_AD7606_CTRL_PORT, GPIO_AD7606_BUSY_Pin)
#define AD7606_DOUTA_READ               GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)
#define AD7606_DOUTB_READ               GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)


void AD7606Initialization(void);
void AD7606Reset(void);
void AD7606ReadOneSample(unsigned short int *DoutA);

#endif
