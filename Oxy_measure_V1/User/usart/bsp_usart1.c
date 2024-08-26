/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   485����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� F103-�Ե� STM32  ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
#include "bsp_usart1.h"
#include <stdarg.h>
#include <stdio.h>


static void Delay(__IO u32 nCount); 




/// ����USART2_RS485�����ж����ȼ�
static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RS485_INT_IRQ; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/// ����USART1�����ж����ȼ�
static void NVIC_Configuration1(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = UART1_INT_IRQ; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
} 

/// ����USART3�����ж����ȼ�
static void NVIC_Configuration3(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = UART3_INT_IRQ; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/*
 * ��������RS485_Config
 * ����  ��USART GPIO ����,����ģʽ����
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void RS485_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART clock */
	RCC_APB2PeriphClockCmd(RS485_USART_RX_GPIO_CLK|RS485_USART_TX_GPIO_CLK|RS485_RE_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(RS485_USART_CLK, ENABLE); 	
	
	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = RS485_USART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RS485_USART_TX_GPIO_PORT, &GPIO_InitStructure);
  	
	// ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = RS485_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(RS485_USART_RX_GPIO_PORT, &GPIO_InitStructure);	
  
  /* 485�շ����ƹܽ� */
	GPIO_InitStructure.GPIO_Pin = RS485_RE_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	
  GPIO_Init(RS485_RE_GPIO_PORT, &GPIO_InitStructure);
	  
	/* USART ģʽ����*/
	USART_InitStructure.USART_BaudRate = RS485_USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(RS485_USART, &USART_InitStructure); 
	/*ʹ��USART*/
  USART_Cmd(RS485_USART, ENABLE);
	
	/*�����ж����ȼ�*/
	NVIC_Configuration();
	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE);
	
	/*����485оƬ�������ģʽ*/
	GPIO_SetBits(RS485_RE_GPIO_PORT,RS485_RE_PIN);
}


/*
 * ��������UART3_RS485_Config
 * ����  ��USART GPIO ����,����ģʽ����
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void UART3_RS485_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART clock */
	RCC_APB2PeriphClockCmd(USART3_RX_GPIO_CLK|USART3_RE_GPIO_CLK|RCC_APB2Periph_AFIO, ENABLE);

	RCC_APB1PeriphClockCmd(USART3_USART_CLK, ENABLE); 	
		
	//GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStructure);
  	
	// ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStructure);	
  
  /* 485�շ����ƹܽ� */
	GPIO_InitStructure.GPIO_Pin = USART3_RE_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	
  GPIO_Init(USART3_RE_GPIO_PORT, &GPIO_InitStructure);
	  
	/* USART ģʽ����*/
	USART_InitStructure.USART_BaudRate = USART3_USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure); 
	/*ʹ��USART*/
  USART_Cmd(USART3, ENABLE);
	
	/*�����ж����ȼ�*/
	NVIC_Configuration3();
	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	/*����485оƬ�������ģʽ*/
	GPIO_SetBits(USART3_RE_GPIO_PORT,USART3_RE_PIN);
}






/***************** ����һ���ַ�  **********************/
//ʹ�õ��ֽ����ݷ���ǰҪʹ�ܷ������ţ����ͺ�Ҫʹ�ܽ������š�
void RS485_SendByte(  uint8_t ch )
{
	/* ����һ���ֽ����ݵ�USART1 */
	USART_SendData(RS485_USART,ch);
		
	/* �ȴ�������� */
	while (USART_GetFlagStatus(RS485_USART, USART_FLAG_TXE) == RESET);	
	
}
/***************** ����һ���ַ�  **********************/
//ʹ�õ��ֽ����ݷ���ǰҪʹ�ܷ������ţ����ͺ�Ҫʹ�ܽ������š�
void UART3_SendByte(  uint8_t ch )
{
	/* ����һ���ֽ����ݵ�USART1 */
	USART_SendData(USART3,ch);
		
	/* �ȴ�������� */
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	
	
}


/***************** ����һ���ַ�  **********************/
//ʹ�õ��ֽ����ݷ���ǰҪʹ�ܷ������ţ����ͺ�Ҫʹ�ܽ������š�
void Uart1_SendByte(  uint8_t ch )
{
	/* ����һ���ֽ����ݵ�USART1 */
	USART_SendData(DEBUG_USART,ch);
		
	/* �ȴ�������� */
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);	
	
}
/*****************  ����ָ�����ȵ��ַ��� **********************/
void RS485_SendStr_length( uint8_t *str,uint32_t strlen1 )
{
	unsigned int k=0;

	RS485_TX_EN()	;//	ʹ�ܷ�������	
    do 
    {
        RS485_SendByte( *(str + k) );
			  Delay(0xFF);
        k++;
    } while(k < strlen1);
		
	/*�Ӷ�����ʱ����֤485�����������*/
	Delay(0xFFF);
		
	RS485_RX_EN()	;//	ʹ�ܽ�������
}

/*****************  ����ָ�����ȵ��ַ��� **********************/

void UART3_SendStr_length( uint8_t *str,uint32_t strlen1 )
{
	unsigned int k=0;

	UART3_TX_EN()	;//	ʹ�ܷ�������	
    do 
    {
        UART3_SendByte( *(str + k) );
			  Delay(0xFF);
        k++;
    } while(k < strlen1);
		
	/*�Ӷ�����ʱ����֤485�����������*/
	Delay(0xFFF);
		
	UART3_RX_EN()	;//	ʹ�ܽ�������
}

/*****************  �����ַ��� **********************/
void RS485_SendString(  uint8_t *str)
{
	unsigned int k=0;
	
	RS485_TX_EN()	;//	ʹ�ܷ�������
	
    do 
    {
        RS485_SendByte(  *(str + k) );
        k++;
    } while(*(str + k)!='\0');
	
	/*�Ӷ�����ʱ����֤485�����������*/
	Delay(0xFFF);
		
	RS485_RX_EN()	;//	ʹ�ܽ�������
}


void UART3_SendStr( uint8_t *str )
{
	unsigned int k=0;

	UART3_TX_EN()	;//	ʹ�ܷ�������	
    do 
    {
        UART3_SendByte( *(str + k) );
			  Delay(0xFF);
        k++;
    } while(*(str + k)!='\0');
		
	/*�Ӷ�����ʱ����֤485�����������*/
	Delay(0xFFF);
		
	UART3_RX_EN()	;//	ʹ�ܽ�������
}

/*****************  �����ַ��� **********************/
void Uart1_SendString(  uint8_t *str)
{
	unsigned int k=0;

    do 
    {
        Uart1_SendByte(  *(str + k) );
        k++;
    } while(*(str + k)!='\0');
	
	/*�Ӷ�����ʱ����֤485�����������*/
	Delay(0xFFF);
		
}








//�жϻ��洮������

#define Error_code					0x83
#define Error_val						1
volatile    uint16_t uart_p = 0;
uint8_t     uart_buff[UART_BUFF_SIZE];
uint8_t			uart_full_flag = 0;
uint8_t			uart_rec_size;

void RS485_IRQHandler(void)
{

   if(USART_GetITStatus(RS485_USART, USART_IT_RXNE) != RESET)
   {
      uart_buff[uart_p] = USART_ReceiveData(RS485_USART);
			//Uart1_SendByte(0x01);
			uart_p++;
	
			USART_ClearITPendingBit(RS485_USART, USART_IT_RXNE);
   }

	if(uart_p== UART_BUFF_SIZE)
	{
			USART_ClearITPendingBit(RS485_USART, USART_IT_RXNE);
			uart_p = 0;
			uart_full_flag = 1;
			uart_rec_size	 = UART_BUFF_SIZE;
	
	}
	if((uart_buff[Error_val] == Error_code) &&(uart_p == UART_BUFF_SIZE-2) )
	{
		
			USART_ClearITPendingBit(RS485_USART, USART_IT_RXNE);
			uart_p = 0;
			uart_full_flag = 1;	
			uart_rec_size  = UART_BUFF_SIZE-2;
	}
	 	/*     if(USART_GetITStatus(RS485_USART,USART_IT_IDLE) != RESET)
        {
                RS485_USART->SR;
                RS485_USART->DR;                                               
                uart_p=0;
								uart_full_flag = 1;				
        }*/
}

volatile    uint16_t uart3_p = 0;
uint8_t     uart3_buff[UART_BUFF_SIZE];
uint8_t			uart3_full_flag = 0;
uint8_t			uart3_rec_size;

void UART3_IRQHandler(void)
{

   if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
   {
      uart3_buff[uart3_p] = USART_ReceiveData(USART3);
			//Uart1_SendByte(0x01);
			uart3_p++;
	
			USART_ClearITPendingBit(USART3, USART_IT_RXNE);
   }

	if(uart3_p== UART_BUFF_SIZE)
	{
			USART_ClearITPendingBit(USART3, USART_IT_RXNE);
			uart3_p = 0;
			uart3_full_flag = 1;
			uart3_rec_size	 = UART_BUFF_SIZE;
	
	}
	if((uart3_buff[Error_val] == Error_code) &&(uart3_p == UART_BUFF_SIZE-2) )
	{
		
			USART_ClearITPendingBit(USART3, USART_IT_RXNE);
			uart3_p = 0;
			uart3_full_flag = 1;	
			uart3_rec_size  = UART_BUFF_SIZE-2;
	}
	/*     if(USART_GetITStatus(USART3,USART_IT_IDLE) != RESET)
        {
                USART3->SR;
                USART3->DR;                                               
                uart3_p=0;
								uart3_full_flag = 1;				
        }*/
}




volatile    uint16_t uart1_p = 0;
uint8_t     uart1_buff[UART1_BUFF_SIZE];
uint8_t			temp;
uint8_t     uart1_rec_flag =0;

void UART1_IRQHandler(void)
{
   if(USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET)
     {
        uart1_buff[uart1_p] = USART_ReceiveData(DEBUG_USART);
						//temp = uart1_buff[uart1_p];
					  //Uart1_SendByte(temp);
			    uart1_p++;					
		      USART_ClearITPendingBit(DEBUG_USART, USART_IT_RXNE);
      }
    if(USART_GetITStatus(DEBUG_USART,USART_IT_IDLE) != RESET)
        {
                DEBUG_USART->SR;
                DEBUG_USART->DR;                                               
                uart1_p=0;
								uart1_rec_flag = 1;				
        }

  }

void Uart1_SendStr_length( uint8_t *str,uint32_t strlen1 )
{
	unsigned int k=0;
    do 
    {
        Uart1_SendByte( *(str + k) );
        k++;
    } while(k < strlen1);
		uart1_p =0;
	/*�Ӷ�����ʱ����֤�����������*/
	Delay(0xFFF);

}	
	
	
//��ȡ���յ������ݺͳ���
char *get_rebuff(uint16_t *len) 
{
    *len = uart_p;
    return (char *)&uart_buff;
}

//��ջ�����
void clean_rebuff(void) 
{

    uint16_t i=UART_BUFF_SIZE+1;
    uart_p = 0;
	while(i)
		uart_buff[--i]=0;

}




static void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}



//UART1
void USART1_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		
		/* config USART1 clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
		
		/* USART1 GPIO config */
		/* Configure USART1 Tx (PA.09) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);    
		/* Configure USART1 Rx (PA.10) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
			
		/* USART1 mode config */
		USART_InitStructure.USART_BaudRate = 19200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART1, &USART_InitStructure); 
		USART_Cmd(USART1, ENABLE);
		
			/*�����ж����ȼ�*/
	NVIC_Configuration1();
	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
}

///�ض���c�⺯��printf��USART1
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ�USART1 */
		USART_SendData(USART1, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);		
	
		return (ch);
}

///�ض���c�⺯��scanf��USART1
int fgetc(FILE *f)
{
		/* �ȴ�����1�������� */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART1);
}
