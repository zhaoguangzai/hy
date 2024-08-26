/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   485驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 F103-霸道 STM32  开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
#include "bsp_usart1.h"
#include <stdarg.h>
#include <stdio.h>


static void Delay(__IO u32 nCount); 




/// 配置USART2_RS485接收中断优先级
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
/// 配置USART1接收中断优先级
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

/// 配置USART3接收中断优先级
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
 * 函数名：RS485_Config
 * 描述  ：USART GPIO 配置,工作模式配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void RS485_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART clock */
	RCC_APB2PeriphClockCmd(RS485_USART_RX_GPIO_CLK|RS485_USART_TX_GPIO_CLK|RS485_RE_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(RS485_USART_CLK, ENABLE); 	
	
	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = RS485_USART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RS485_USART_TX_GPIO_PORT, &GPIO_InitStructure);
  	
	// 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = RS485_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(RS485_USART_RX_GPIO_PORT, &GPIO_InitStructure);	
  
  /* 485收发控制管脚 */
	GPIO_InitStructure.GPIO_Pin = RS485_RE_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	
  GPIO_Init(RS485_RE_GPIO_PORT, &GPIO_InitStructure);
	  
	/* USART 模式配置*/
	USART_InitStructure.USART_BaudRate = RS485_USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(RS485_USART, &USART_InitStructure); 
	/*使能USART*/
  USART_Cmd(RS485_USART, ENABLE);
	
	/*配置中断优先级*/
	NVIC_Configuration();
	/* 使能串口接收中断 */
	USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE);
	
	/*控制485芯片进入接收模式*/
	GPIO_SetBits(RS485_RE_GPIO_PORT,RS485_RE_PIN);
}


/*
 * 函数名：UART3_RS485_Config
 * 描述  ：USART GPIO 配置,工作模式配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void UART3_RS485_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART clock */
	RCC_APB2PeriphClockCmd(USART3_RX_GPIO_CLK|USART3_RE_GPIO_CLK|RCC_APB2Periph_AFIO, ENABLE);

	RCC_APB1PeriphClockCmd(USART3_USART_CLK, ENABLE); 	
		
	//GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStructure);
  	
	// 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStructure);	
  
  /* 485收发控制管脚 */
	GPIO_InitStructure.GPIO_Pin = USART3_RE_PIN;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	
  GPIO_Init(USART3_RE_GPIO_PORT, &GPIO_InitStructure);
	  
	/* USART 模式配置*/
	USART_InitStructure.USART_BaudRate = USART3_USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure); 
	/*使能USART*/
  USART_Cmd(USART3, ENABLE);
	
	/*配置中断优先级*/
	NVIC_Configuration3();
	/* 使能串口接收中断 */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	/*控制485芯片进入接收模式*/
	GPIO_SetBits(USART3_RE_GPIO_PORT,USART3_RE_PIN);
}






/***************** 发送一个字符  **********************/
//使用单字节数据发送前要使能发送引脚，发送后要使能接收引脚。
void RS485_SendByte(  uint8_t ch )
{
	/* 发送一个字节数据到USART1 */
	USART_SendData(RS485_USART,ch);
		
	/* 等待发送完毕 */
	while (USART_GetFlagStatus(RS485_USART, USART_FLAG_TXE) == RESET);	
	
}
/***************** 发送一个字符  **********************/
//使用单字节数据发送前要使能发送引脚，发送后要使能接收引脚。
void UART3_SendByte(  uint8_t ch )
{
	/* 发送一个字节数据到USART1 */
	USART_SendData(USART3,ch);
		
	/* 等待发送完毕 */
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	
	
}


/***************** 发送一个字符  **********************/
//使用单字节数据发送前要使能发送引脚，发送后要使能接收引脚。
void Uart1_SendByte(  uint8_t ch )
{
	/* 发送一个字节数据到USART1 */
	USART_SendData(DEBUG_USART,ch);
		
	/* 等待发送完毕 */
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);	
	
}
/*****************  发送指定长度的字符串 **********************/
void RS485_SendStr_length( uint8_t *str,uint32_t strlen1 )
{
	unsigned int k=0;

	RS485_TX_EN()	;//	使能发送数据	
    do 
    {
        RS485_SendByte( *(str + k) );
			  Delay(0xFF);
        k++;
    } while(k < strlen1);
		
	/*加短暂延时，保证485发送数据完毕*/
	Delay(0xFFF);
		
	RS485_RX_EN()	;//	使能接收数据
}

/*****************  发送指定长度的字符串 **********************/

void UART3_SendStr_length( uint8_t *str,uint32_t strlen1 )
{
	unsigned int k=0;

	UART3_TX_EN()	;//	使能发送数据	
    do 
    {
        UART3_SendByte( *(str + k) );
			  Delay(0xFF);
        k++;
    } while(k < strlen1);
		
	/*加短暂延时，保证485发送数据完毕*/
	Delay(0xFFF);
		
	UART3_RX_EN()	;//	使能接收数据
}

/*****************  发送字符串 **********************/
void RS485_SendString(  uint8_t *str)
{
	unsigned int k=0;
	
	RS485_TX_EN()	;//	使能发送数据
	
    do 
    {
        RS485_SendByte(  *(str + k) );
        k++;
    } while(*(str + k)!='\0');
	
	/*加短暂延时，保证485发送数据完毕*/
	Delay(0xFFF);
		
	RS485_RX_EN()	;//	使能接收数据
}


void UART3_SendStr( uint8_t *str )
{
	unsigned int k=0;

	UART3_TX_EN()	;//	使能发送数据	
    do 
    {
        UART3_SendByte( *(str + k) );
			  Delay(0xFF);
        k++;
    } while(*(str + k)!='\0');
		
	/*加短暂延时，保证485发送数据完毕*/
	Delay(0xFFF);
		
	UART3_RX_EN()	;//	使能接收数据
}

/*****************  发送字符串 **********************/
void Uart1_SendString(  uint8_t *str)
{
	unsigned int k=0;

    do 
    {
        Uart1_SendByte(  *(str + k) );
        k++;
    } while(*(str + k)!='\0');
	
	/*加短暂延时，保证485发送数据完毕*/
	Delay(0xFFF);
		
}








//中断缓存串口数据

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
	/*加短暂延时，保证发送数据完毕*/
	Delay(0xFFF);

}	
	
	
//获取接收到的数据和长度
char *get_rebuff(uint16_t *len) 
{
    *len = uart_p;
    return (char *)&uart_buff;
}

//清空缓冲区
void clean_rebuff(void) 
{

    uint16_t i=UART_BUFF_SIZE+1;
    uart_p = 0;
	while(i)
		uart_buff[--i]=0;

}




static void Delay(__IO uint32_t nCount)	 //简单的延时函数
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
		
			/*配置中断优先级*/
	NVIC_Configuration1();
	/* 使能串口接收中断 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
}

///重定向c库函数printf到USART1
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到USART1 */
		USART_SendData(USART1, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);		
	
		return (ch);
}

///重定向c库函数scanf到USART1
int fgetc(FILE *f)
{
		/* 等待串口1输入数据 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART1);
}
