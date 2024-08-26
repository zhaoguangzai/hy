#ifndef _FUNCTION_H
#define	_FUNCTION_H

#include "stm32f10x.h"


#define   START_BUTTON_PORT			        GPIOB
#define		START_BUTTON_PIN							GPIO_Pin_1
#define   START_GPIO_CLK								RCC_APB2Periph_GPIOB	
#define   STOP_GPIO_CLK									RCC_APB2Periph_GPIOA
#define   STOP_BUTTON_PORT			        GPIOA
#define		STOP_BUTTON_PIN								GPIO_Pin_8




#define   SUC_CTRL1_PORT								GPIOA
#define		SUC_CTRL1_PIN									GPIO_Pin_0
#define   SUC_CTRL2_PORT								GPIOA
#define		SUC_CTRL2_PIN									GPIO_Pin_1

#define		SUC_CTRL1_OPEN								GPIO_SetBits(SUC_CTRL1_PORT, SUC_CTRL1_PIN)
#define		SUC_CTRL1_CLOSED							GPIO_ResetBits(SUC_CTRL1_PORT, SUC_CTRL1_PIN)
#define		SUC_CTRL2_OPEN								GPIO_SetBits(SUC_CTRL2_PORT, SUC_CTRL2_PIN)
#define		SUC_CTRL2_CLOSED							GPIO_ResetBits(SUC_CTRL2_PORT, SUC_CTRL2_PIN)


#define   BLOW_CTRL1_PORT								GPIOA
#define		BLOW_CTRL1_PIN								GPIO_Pin_5
#define   BLOW_CTRL2_PORT								GPIOA
#define		BLOW_CTRL2_PIN								GPIO_Pin_6

#define		BLOW_CTRL1_OPEN								GPIO_SetBits(BLOW_CTRL1_PORT, BLOW_CTRL1_PIN)
#define		BLOW_CTRL1_CLOSED							GPIO_ResetBits(BLOW_CTRL1_PORT, BLOW_CTRL1_PIN)
#define		BLOW_CTRL2_OPEN								GPIO_SetBits(BLOW_CTRL2_PORT, BLOW_CTRL2_PIN)
#define		BLOW_CTRL2_CLOSED							GPIO_ResetBits(BLOW_CTRL2_PORT, BLOW_CTRL2_PIN)



#define   GRE_LED_CTRL_PORT							GPIOC
#define 	GRE_LED_CTRL_PIN							GPIO_Pin_5

#define   YEL_LED_CTRL_PORT							GPIOA
#define 	YEL_LED_CTRL_PIN							GPIO_Pin_7

#define   RED_LED_CTRL_PORT							GPIOC
#define 	RED_LED_CTRL_PIN							GPIO_Pin_4


#define   HORN_CTRL_PORT							  GPIOB
#define 	HORN_CTRL_PIN							    GPIO_Pin_0

#define		YEL_LED_CTRL_OPEN							GPIO_SetBits(YEL_LED_CTRL_PORT, YEL_LED_CTRL_PIN)
#define		YEL_LED_CTRL_CLOSED						GPIO_ResetBits(YEL_LED_CTRL_PORT, YEL_LED_CTRL_PIN)
#define		GRE_LED_CTRL_OPEN							GPIO_SetBits(GRE_LED_CTRL_PORT, GRE_LED_CTRL_PIN)
#define		GRE_LED_CTRL_CLOSED						GPIO_ResetBits(GRE_LED_CTRL_PORT, GRE_LED_CTRL_PIN)
#define		RED_LED_CTRL_OPEN							GPIO_SetBits(RED_LED_CTRL_PORT, RED_LED_CTRL_PIN)
#define		RED_LED_CTRL_CLOSED						GPIO_ResetBits(RED_LED_CTRL_PORT, RED_LED_CTRL_PIN)
#define		HORN_CTRL_OPEN							  GPIO_SetBits(HORN_CTRL_PORT, HORN_CTRL_PIN)
#define		HORN_CTRL_CLOSED						  GPIO_ResetBits(HORN_CTRL_PORT, HORN_CTRL_PIN)

#define   GAS_PUMP_PWM_PORT							GPIOB
#define 	GAS_PUMP_PWM_PIN							GPIO_Pin_9
#define   GAS_PWM_OPEN                  GPIO_ResetBits(GAS_PUMP_PWM_PORT, GAS_PUMP_PWM_PIN)
#define   GAS_PWM_CLOSED                GPIO_SetBits(GAS_PUMP_CTRL_PORT, GAS_PUMP_CTRL_PIN)

#define   GAS_PUMP_CTRL_PORT						GPIOA
#define 	GAS_PUMP_CTRL_PIN							GPIO_Pin_1
#define		GAS_PUMP_CTRL_OPEN						GPIO_SetBits(GAS_PUMP_CTRL_PORT, GAS_PUMP_CTRL_PIN)
#define		GAS_PUMP_CTRL_CLOSED					GPIO_ResetBits(GAS_PUMP_CTRL_PORT, GAS_PUMP_CTRL_PIN)



#define KEY1_INT_EXTI_PORTSOURCE   			GPIO_PortSourceGPIOB
#define KEY1_INT_EXTI_PINSOURCE    			GPIO_PinSource1

#define KEY1_INT_EXTI_LINE        			 EXTI_Line1
#define KEY1_INT_EXTI_IRQ          				EXTI1_IRQn
 
#define KEY1_IRQHandler            				EXTI1_IRQHandler


typedef enum{
	H_MEASURE,
	H_LEAKAGE,
	H_JAM,
	H_CALIBRATION,
	H_ADJUSTMENT,
	H_CORRENT,
	H_jiaozhun,
}system_function_t;

typedef enum{
	Init_Status,
	Init_Plus_Status,
	One_Status,
	One_status_plus,
	Two_Status,
	Thr_Status,
	For_Status,
	Fiv_Status,
	Six_Status,
	Sev_Status,
	Half_Status,
	Eig_Status,
}system_status_t;



void Button_Init(void);
void Ctrl_PIN_Init(void);
void TIM4_Init(void);
void TIM3_Init(u16 per,u16 psc);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
#endif
