/**
  ******************************************************************************
  * @file    Funciton.c
  * @author  zhaoguangzai
  * @version V1.0
  * @date    2022-09-27
  * @brief   ���ܺ�������
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Hydris  stm32 
  * 
  * 
  ******************************************************************************
  */
#include "function.h"   

static void NVIC_Configuration_GPIO(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  NVIC_InitStructure.NVIC_IRQChannel = KEY1_INT_EXTI_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

 /**
  * @brief  ��ȡSTART BUTTON  and  STOP Buttuon  value
  * @param 
  * @retval 
  */

void Button_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(STOP_GPIO_CLK|START_GPIO_CLK, ENABLE);
	
		NVIC_Configuration_GPIO();  //�����ⲿ�ж�
	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = START_BUTTON_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(START_BUTTON_PORT, &GPIO_InitStructure);	
	
   GPIO_EXTILineConfig(KEY1_INT_EXTI_PORTSOURCE, KEY1_INT_EXTI_PINSOURCE); 
   EXTI_InitStructure.EXTI_Line = KEY1_INT_EXTI_LINE;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);
	

}




void KEY1_IRQHandler(void)
{
//	 for(delay_int =0;delay_int <9999;delay_int++)
//	 {;}
	if((GPIO_ReadInputDataBit(START_BUTTON_PORT,START_BUTTON_PIN) == 0))
		 {
				TIM_Cmd(TIM4,ENABLE);	
				
			}
		if((GPIO_ReadInputDataBit(START_BUTTON_PORT,START_BUTTON_PIN) == 1))
		 {
				TIM_Cmd(TIM4,DISABLE);	
			 TIM4->CNT = 0;
				
			}
	  if(EXTI_GetITStatus(KEY1_INT_EXTI_LINE) != RESET) 
	  {	
		
		   EXTI_ClearITPendingBit(KEY1_INT_EXTI_LINE);     
	   }  
}

void Ctrl_PIN_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = SUC_CTRL1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SUC_CTRL1_PORT, &GPIO_InitStructure);	
	//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = SUC_CTRL2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SUC_CTRL2_PORT, &GPIO_InitStructure);	
//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = BLOW_CTRL1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BLOW_CTRL1_PORT, &GPIO_InitStructure);	
//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = BLOW_CTRL2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BLOW_CTRL2_PORT, &GPIO_InitStructure);	
// yellow_led
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = YEL_LED_CTRL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(YEL_LED_CTRL_PORT, &GPIO_InitStructure);	
//	gre_led
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GRE_LED_CTRL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GRE_LED_CTRL_PORT, &GPIO_InitStructure);	
//	red_led
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = RED_LED_CTRL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RED_LED_CTRL_PORT, &GPIO_InitStructure);
	
	//	horn
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = HORN_CTRL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HORN_CTRL_PORT, &GPIO_InitStructure);
//	gas_ctrl
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GAS_PUMP_CTRL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GAS_PUMP_CTRL_PORT, &GPIO_InitStructure);	

//	gas_PWM
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = GAS_PUMP_PWM_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GAS_PUMP_PWM_PORT, &GPIO_InitStructure);	
	

}


void TIM4_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct; //����TIMER�ṹ�����
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	
	TIM_TimeBaseInitStruct.TIM_Period = 9999; 
  TIM_TimeBaseInitStruct.TIM_Prescaler = 7199;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	
	
  TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //������ʱ���ж�
  TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
     
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;//��ʱ���ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;        //�����ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);    
     
	//TIM_Cmd(TIM4, ENABLE);

}

  /*******************************************************************************
  * ������       : TIM3_Init
  * ��������     : TIM3��ʼ������
  * ����         : per:��װ��ֵ
                   psc:��Ƶϵ��
  * ���         : ��
  *******************************************************************************/
  void TIM3_Init(u16 per,u16 psc)
  {
     TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
     NVIC_InitTypeDef NVIC_InitStructure;
     
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//ʹ��TIM3ʱ��
     
     TIM_TimeBaseInitStructure.TIM_Period=per;   //�Զ�װ��ֵ
     TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //��Ƶϵ��
     TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
     TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //�������ϼ���ģʽ
     TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
     
     TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //������ʱ���ж�
     TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
     
     NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;//��ʱ���ж�
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�
     NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;        //�����ȼ�
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQͨ��ʹ��
     NVIC_Init(&NVIC_InitStructure);    
     
     //TIM_Cmd(TIM3,DISABLE); //ʹ�ܶ�ʱ��  
 }

  /*******************************************************************************
  * ������       : TIM3_IRQHandler
  * ��������     : TIM3�жϺ���
  * ����         : ��
  * ���         : ��
  *******************************************************************************/
 uint8_t sec_flag =0;
 uint8_t Flush_Sec = 10;
 uint8_t sec_cnt = 0;
 uint8_t flag_10s = 0;
 uint8_t  startcurve;

  void TIM3_IRQHandler(void)
  {
     if(TIM_GetITStatus(TIM3,TIM_IT_Update))
     {
       if(sec_cnt < Flush_Sec)  
			 {
				 sec_cnt++;
				 if(sec_cnt == startcurve)
				 {flag_10s =1;}
			 }
			 else 
			 {
			   sec_flag =1;
				 sec_cnt =0;
				 TIM_Cmd(TIM3,DISABLE);
			 }
     }
     TIM_ClearITPendingBit(TIM3,TIM_IT_Update);    
 }

   /*******************************************************************************
  * ������       : TIM4_IRQHandler
  * ��������     : TIM4�жϺ���
  * ����         : ��
  * ���         : ��
  *******************************************************************************/
uint8_t     button_trigger_flag =0;
   void TIM4_IRQHandler(void)
  {
     if(TIM_GetITStatus(TIM4,TIM_IT_Update))
     {
			  
				TIM_Cmd(TIM4,DISABLE);
			  TIM4->CNT = 0;
			 button_trigger_flag =1;
     }
     TIM_ClearITPendingBit(TIM4,TIM_IT_Update);    
 }

/*********************************************END OF FILE**********************/
