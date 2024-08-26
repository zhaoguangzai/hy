#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include <stdio.h>

/** 
  * ���ں궨�壬��ͬ�Ĵ��ڹ��ص����߲�һ������ֲʱ��Ҫ�޸��⼸����
  */
#define  DEBUG_USART                   USART1
#define  DEBUG_USART_CLK               RCC_APB2Periph_USART1
#define  DEBUG_USART_APBxClkCmd        RCC_APB2PeriphClockCmd
#define  DEBUG_USART_BAUDRATE          115200

// USART GPIO ���ź궨��
#define  DEBUG_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  DEBUG_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  DEBUG_USART_TX_GPIO_PORT       GPIOA   
#define  DEBUG_USART_TX_GPIO_PIN        GPIO_Pin_2
#define  DEBUG_USART_RX_GPIO_PORT       GPIOA
#define  DEBUG_USART_RX_GPIO_PIN        GPIO_Pin_3



/*USART�š�ʱ�ӡ�������*/
#define RS485_USART                             USART2
#define RS485_USART_CLK                         RCC_APB1Periph_USART2
#define RS485_USART_BAUDRATE                    9600

/*UART2 RX����*/
#define RS485_USART_RX_GPIO_PORT                GPIOA
#define RS485_USART_RX_GPIO_CLK                 RCC_APB2Periph_GPIOA
#define RS485_USART_RX_PIN                      GPIO_Pin_3

/*UART2 TX����*/
#define RS485_USART_TX_GPIO_PORT                GPIOA
#define RS485_USART_TX_GPIO_CLK                 RCC_APB2Periph_GPIOA
#define RS485_USART_TX_PIN                      GPIO_Pin_2

/*UART2 485�շ���������*/
#define RS485_RE_GPIO_PORT											GPIOA
#define RS485_RE_GPIO_CLK												RCC_APB2Periph_GPIOB
#define RS485_RE_PIN														GPIO_Pin_4


/*USART�š�ʱ�ӡ�������*/
#define USART_3                                 USART3
#define USART3_USART_CLK                        RCC_APB1Periph_USART3
#define USART3_USART_BAUDRATE                   9600

/*UART3 RX����*/
#define USART3_RX_GPIO_PORT                     GPIOB
#define USART3_RX_GPIO_CLK                      RCC_APB2Periph_GPIOB
#define USART3_RX_PIN                           GPIO_Pin_11

/*UART3 TX����*/
#define USART3_TX_GPIO_PORT                     GPIOB
#define USART3_TX_GPIO_CLK                      RCC_APB2Periph_GPIOB
#define USART3_TX_PIN                           GPIO_Pin_10

/*UART3 485�շ���������*/
#define USART3_RE_GPIO_PORT											GPIOC
#define USART3_RE_GPIO_CLK											RCC_APB2Periph_GPIOC
#define USART3_RE_PIN														GPIO_Pin_13

/*�ж����*/
#define RS485_INT_IRQ                 					 USART2_IRQn
#define RS485_IRQHandler                         USART2_IRQHandler

#define UART1_INT_IRQ                 					 USART1_IRQn
#define UART1_IRQHandler                         USART1_IRQHandler

#define UART3_INT_IRQ                 					 USART3_IRQn
#define UART3_IRQHandler                         USART3_IRQHandler

#define UART_BUFF_SIZE       7
#define UART1_BUFF_SIZE      22
	/// ����ȷ����ʱ
static void RS485_delay(__IO u32 nCount)
{
	for(; nCount != 0; nCount--);
} 


/*�����շ�����*/
//�������ģʽ,����Ҫ����ʱ�ȴ�485����������
#define RS485_RX_EN()			RS485_delay(1000); GPIO_ResetBits(RS485_RE_GPIO_PORT,RS485_RE_PIN);  RS485_delay(1000);
//���뷢��ģʽ,����Ҫ����ʱ�ȴ�485����������
#define RS485_TX_EN()			RS485_delay(1000); GPIO_SetBits(RS485_RE_GPIO_PORT,RS485_RE_PIN);  RS485_delay(1000);

/*�����շ�����*/
//�������ģʽ,����Ҫ����ʱ�ȴ�485����������
#define UART3_RX_EN()			RS485_delay(1000); GPIO_ResetBits(USART3_RE_GPIO_PORT,USART3_RE_PIN);  RS485_delay(1000);
//���뷢��ģʽ,����Ҫ����ʱ�ȴ�485����������
#define UART3_TX_EN()			RS485_delay(1000); GPIO_SetBits(USART3_RE_GPIO_PORT,USART3_RE_PIN);  RS485_delay(1000);

/*debug*/

#define RS485_DEBUG_ON          1
#define RS485_DEBUG_ARRAY_ON   1
#define RS485_DEBUG_FUNC_ON    1
   
   
// Log define
#define RS485_INFO(fmt,arg...)           printf("<<-RS485-INFO->> "fmt"\n",##arg)
#define RS485_ERROR(fmt,arg...)          printf("<<-RS485-ERROR->> "fmt"\n",##arg)
#define RS485_DEBUG(fmt,arg...)          do{\
																					 if(RS485_DEBUG_ON)\
																					 printf("<<-RS485-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
																				 }while(0)

#define RS485_DEBUG_ARRAY(array, num)    do{\
                                         int32_t i;\
                                         uint8_t* a = array;\
                                         if(RS485_DEBUG_ARRAY_ON)\
                                         {\
                                            printf("<<-RS485-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printf("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printf("\n");\
                                                }\
                                            }\
                                            printf("\n");\
                                        }\
                                       }while(0)

#define RS485_DEBUG_FUNC()               do{\
                                         if(RS485_DEBUG_FUNC_ON)\
                                         printf("<<-RS485-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)


																			 
																			 
																			 
void RS485_Config(void);
void RS485_SendByte(  uint8_t ch );
void RS485_SendStr_length( uint8_t *str,uint32_t strlen );
void RS485_SendString(  uint8_t *str);


																			 
void UART3_RS485_Config(void);
void UART3_SendByte(  uint8_t ch );
void UART3_SendStr_length( uint8_t *str,uint32_t strlen );
void UART3_SendStr(  uint8_t *str);																		 

void RS485_IRQHandler(void);
char *get_rebuff(uint16_t *len);
void clean_rebuff(void);
void u2_printf(char* fmt, ...);	
void USART1_Config(void);	


void Uart1_SendString(  uint8_t *str);
void Uart1_SendByte(  uint8_t ch );
void Uart1_SendStr_length( uint8_t *str,uint32_t strlen1 );		
void UART1_IRQHandler(void);																			 
//int fputc(int ch, FILE *f);
//void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);

#endif /* __USART1_H */
