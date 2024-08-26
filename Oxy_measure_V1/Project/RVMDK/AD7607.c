/********************************************************************************
 Author : CAC (CustomerApplications Center, Asia) 

 Date : 2012-01-04

 File name : AD7606.c

 Description : test the serial port operation of AD7606

 Hardware plateform : ADuC7026_DEMO_V1.2 + EVAL-AD7606EDZ_Rev.A1  	
********************************************************************************/

//#include "ADuC7026.h"
//#include "ADuC7026Driver.h"
#include "AD7606.h"
#include <string.h>
//#include "stm32f10x_systick.h"

#define  UIP_BYTE_ORDER     UIP_LITTLE_ENDIAN





short  eAdc[30];


/*void delay_ms(unsigned int u)	  //�ӳٺ�������һ�ο���SysTick����MS�ӳٴ���
{
   unsigned int k;
   while(u--)
    {
	   k=7200;
     while(k--);
	}
}*/




/*void delay_ms(u16 nms)
{
	//TimingDelay = nTime;
	//ʹ��ϵͳ�δ�ʱ��
	//while(TimingDelay !=0);
	u32 temp;   
	SysTick->LOAD = 9000*nms;   
	SysTick->VAL=0X00;//��ռ�����   
	SysTick->CTRL=0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ  
	do  {    temp=SysTick->CTRL;//��ȡ��ǰ������ֵ   
	}while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��     
	SysTick->CTRL=0x00; //�رռ�����     
	SysTick->VAL =0X00; //��ռ����� 
}

*/



void IO_Configuration(void)			//AD7606�ܽų�ʼ��
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOA/B/C��ʱ��
	RCC->APB2ENR |=0x09;  //ʹ��GPIO��ʱ�ӣ�������APB2
	
	/* ���� SPI ����: Ҳ������IOģ��SPI��SCK, MISO and MOSI */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //����������50MHz
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   //�����������������IO���
  	GPIO_Init(GPIOB, &GPIO_InitStructure);             //�ܽ�ѡ��ΪGPIOB9����ΪAD7606_SCK

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //����������50MHz
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	   //�ڲ��������룬��������IO����
  	GPIO_Init(GPIOB, &GPIO_InitStructure); 			   //�ܽ�ѡ��ΪGPIOB8������DOUTA�����ݴ���

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;		   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //����������50MHz
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	   //�ڲ��������룬��������IO����
  	GPIO_Init(GPIOC, &GPIO_InitStructure);			   //�ܽ�ѡ��ΪGPIOC4������DOUTB�����ݴ���
	
	/* ����AD7606�ļ����������ţ�����AD7606�ĸ�λ����GPIOB5�Ͳ�����������GPIOB7 */
	GPIO_InitStructure.GPIO_Pin=GPIO_AD7606_RST_Pin|GPIO_AD7606_CNVST_Pin;		
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;				//����������50MHz
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;              //ͨ�������������������IO���
	GPIO_Init(GPIO_AD7606_CTRL_PORT, &GPIO_InitStructure);		//�ܽ�ѡ��ΪGPIOB5��GPIOB7
	
	GPIO_SetBits(GPIO_AD7606_CTRL_PORT, GPIO_AD7606_RST_Pin);	//GPIOB6ΪRST���ţ�����Ϊ1   GPIO_SetBitsΪ��1	��GPIO_ResetBitsΪ��0
    GPIO_SetBits(GPIO_AD7606_CTRL_PORT, GPIO_AD7606_CNVST_Pin);	//GPIOB7ΪCNVST���ţ�����Ϊ1   GPIO_SetBitsΪ��1	��GPIO_ResetBitsΪ��0
	
	GPIO_InitStructure.GPIO_Pin=GPIO_AD7606_SCS_Pin;		   
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;		//����������50MHz
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;      //�����������������IO���
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//�ܽ�ѡ��ΪGPIOA4,����AD7606��CS��
	
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  //	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //�������,���������,�͵�ƽ,������������
  //GPIO_Init(GPIOA, &GPIO_InitStructure);  
	// GPIO_SetBits(GPIOB, GPIO_AD7606_SCS_Pin);  

	GPIO_InitStructure.GPIO_Pin=GPIO_AD7606_BUSY_Pin;		  
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;		      //����������50MHz
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;		          //�ڲ��������룬��������IO����
	GPIO_Init(GPIO_AD7606_CTRL_PORT, &GPIO_InitStructure);    //�ܽ�ѡ��ΪGPIOB5,����AD7606��BUSY��
}

void AD7606Initialization(void)	 //AD7606�ĳ�ʼ��
{
	IO_Configuration();		         //AD7606�Ŀ�����������
	AD7606Reset();				     //����AD7606�ĸ�λ����
}

void AD7606Reset(void)			 //AD7606�ĸ�λ����
{
	AD7606_RST_LOW;				 //GPIOB6ΪRST���ţ���λ�õ�
	
	AD7606_RST_HIGH;			 //GPIOB6ΪRST���ţ���λ�ø�
		 
	AD7606_RST_LOW;				 //GPIOB6ΪRST���ţ���λ�õ�

}
void AD7606ReadOneSample(unsigned short int *buf)
{
	unsigned int j, k;
	unsigned short int TempA, TempB;


	for(j=0; j<4;	j++ )
	{ 
		TempA=0;
		TempB=0;
		GPIOA->BRR=GPIO_Pin_7;               //AD7606_SCS_LOW��AD7606��CS���ͣ�ʹ�����ݴ���;
		for(k=0;k<16;k++)
		{ 
			GPIOB->BRR=GPIO_Pin_9; 		     	    //AD7606_SCK_LOW����ʼ���ݴ���;
//			TempA += (GPIOB->IDR&GPIO_Pin_8)?1:0; TempA<<=1;   //��ȡDoutAͨ�����ݣ���16λ
//			TempB += (GPIOC->IDR&GPIO_Pin_4)?1:0; TempB<<=1;   //��ȡDoutBͨ�����ݣ���16λ
			TempA <<= 1;
			if(GPIOB->IDR&GPIO_Pin_8)
			TempA |= 0x01;
			
			TempB <<= 1;
			if(GPIOC->IDR&GPIO_Pin_4)
			TempB |= 0x01;
			GPIOB->BSRR=GPIO_Pin_9;		     	    //AD7606_SCK_HIGH��ֹͣ���ݴ���;
		}	
	  GPIOA->BSRR=GPIO_Pin_7;               //AD7606_SCS_HIGH��CS���ߣ����ݴ��䲻ʹ��;		
		buf[j]=(short int)TempA;
		buf[4+j]=(short int)TempB;
	} 
}

void AD7606ReadOneRoad(short int *buf)
{
	unsigned int j, k;
	unsigned short int TempA, TempB;

	for(j=0; j<4;	j++ )
	{ 
		TempA=0;
		TempB=0;
		GPIOA->BRR=GPIO_Pin_4;               //AD7606_SCS_LOW��AD7606��CS���ͣ�ʹ�����ݴ���;
		for(k=0;k<16;k++)
		{ 
			GPIOB->BRR=GPIO_Pin_9; 		     	    //AD7606_SCK_LOW����ʼ���ݴ���;
			TempA += (GPIOB->IDR&GPIO_Pin_8)?1:0; TempA<<=1;   //��ȡDoutAͨ�����ݣ���16λ
			TempB += (GPIOC->IDR&GPIO_Pin_4)?1:0; TempB<<=1;   //��ȡDoutBͨ�����ݣ���16λ
			GPIOB->BSRR=GPIO_Pin_9;		     	    //AD7606_SCK_HIGH��ֹͣ���ݴ���;
		}	
	  GPIOA->BSRR=GPIO_Pin_4;               //AD7606_SCS_HIGH��CS���ߣ����ݴ��䲻ʹ��;		
		buf[j]=(short int)TempA;
		buf[4+j]=(short int)TempB;
	} 
}

