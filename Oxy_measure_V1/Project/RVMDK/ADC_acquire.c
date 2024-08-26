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

#ifndef HTONS
#   if UIP_BYTE_ORDER == UIP_BIG_ENDIAN
#      define HTONS(n) (n)
#   else /* UIP_BYTE_ORDER == UIP_BIG_ENDIAN */
#      define HTONS(n) //(unsigned short)((((unsigned short) (n)) << 8) | (((unsigned short) (n)) >> 8))
#   endif /* UIP_BYTE_ORDER == UIP_BIG_ENDIAN */
#else
#error "HTONS already defined!"
#endif /* HTONS */ 



short  eAdc[30];

/*void delay_ms(unsigned int u)	  //延迟函数，这一段可用SysTick（）MS延迟代替
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
	//使能系统滴答定时器
	//while(TimingDelay !=0);
	u32 temp;   
	SysTick->LOAD = 9000*nms;   
	SysTick->VAL=0X00;//清空计数器   
	SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源  
	do  {    temp=SysTick->CTRL;//读取当前倒计数值   
	}while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达     
	SysTick->CTRL=0x00; //关闭计数器     
	SysTick->VAL =0X00; //清空计数器 
}

*/



void IO_Configuration(void)			//AD7606管脚初始化
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA/B/C的时钟
	RCC->APB2ENR |=0x09;  //使能GPIO的时钟，这里用APB2
	
	/* 配置 SPI 引脚: 也就是用IO模拟SPI的SCK, MISO and MOSI */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //最高输出速率50MHz
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   //推挽输出，就是用作IO输出
  	GPIO_Init(GPIOB, &GPIO_InitStructure);             //管脚选择为GPIOB9，作为AD7606_SCK

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //最高输出速率50MHz
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	   //内部上拉输入，就是用作IO输入
  	GPIO_Init(GPIOB, &GPIO_InitStructure); 			   //管脚选择为GPIOB8，用作DOUTA的数据传输

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;		   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //最高输出速率50MHz
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	   //内部上拉输入，就是用作IO输入
  	GPIO_Init(GPIOC, &GPIO_InitStructure);			   //管脚选择为GPIOC4，用作DOUTB的数据传输
	
	/* 配置AD7606的几个控制引脚，包括AD7606的复位引脚GPIOB5和采样触发引脚GPIOB7 */
	GPIO_InitStructure.GPIO_Pin=GPIO_AD7606_RST_Pin|GPIO_AD7606_CNVST_Pin;		
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;				//最高输出速率50MHz
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;              //通用推挽输出，就是用作IO输出
	GPIO_Init(GPIO_AD7606_CTRL_PORT, &GPIO_InitStructure);		//管脚选择为GPIOB5和GPIOB7
	
	GPIO_SetBits(GPIO_AD7606_CTRL_PORT, GPIO_AD7606_RST_Pin);	//GPIOB6为RST引脚，设置为1   GPIO_SetBits为置1	，GPIO_ResetBits为置0
    GPIO_SetBits(GPIO_AD7606_CTRL_PORT, GPIO_AD7606_CNVST_Pin);	//GPIOB7为CNVST引脚，设置为1   GPIO_SetBits为置1	，GPIO_ResetBits为置0
	
	GPIO_InitStructure.GPIO_Pin=GPIO_AD7606_SCS_Pin;		   
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;		//最高输出速率50MHz
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;      //推挽输出，就是用作IO输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//管脚选择为GPIOA4,用作AD7606的CS线
	
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  //	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出,可以输出高,低电平,连接数字器件
  //GPIO_Init(GPIOA, &GPIO_InitStructure);  
	// GPIO_SetBits(GPIOB, GPIO_AD7606_SCS_Pin);  

	GPIO_InitStructure.GPIO_Pin=GPIO_AD7606_BUSY_Pin;		  
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;		      //最高输出速率50MHz
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;		          //内部上拉输入，就是用作IO输入
	GPIO_Init(GPIO_AD7606_CTRL_PORT, &GPIO_InitStructure);    //管脚选择为GPIOB5,用作AD7606的BUSY线
}

void AD7606Initialization(void)	 //AD7606的初始化
{
	IO_Configuration();		         //AD7606的控制引脚配置
	AD7606Reset();				     //调用AD7606的复位程序
}

void AD7606Reset(void)			 //AD7606的复位程序
{
	AD7606_RST_LOW;				 //GPIOB6为RST引脚，复位置低
	
	AD7606_RST_HIGH;			 //GPIOB6为RST引脚，复位置高
		 
	AD7606_RST_LOW;				 //GPIOB6为RST引脚，复位置低

}
void AD7606ReadOneSample(unsigned short int *buf)
{
	unsigned int j, k;
	unsigned short int TempA, TempB;


	for(j=0; j<4;	j++ )
	{ 
		TempA=0;
		TempB=0;
		GPIOA->BRR=GPIO_Pin_7;               //AD7606_SCS_LOW，AD7606的CS拉低，使能数据传输;
		for(k=0;k<14;k++)
		{ 
			GPIOB->BRR=GPIO_Pin_9; 		     	    //AD7606_SCK_LOW，开始数据传输;
//			TempA += (GPIOB->IDR&GPIO_Pin_8)?1:0; TempA<<=1;   //读取DoutA通道数据，共16位
//			TempB += (GPIOC->IDR&GPIO_Pin_4)?1:0; TempB<<=1;   //读取DoutB通道数据，共16位
			TempA <<= 1;
			if(GPIOB->IDR&GPIO_Pin_8)
			TempA |= 0x01;
			
			TempB <<= 1;
			if(GPIOC->IDR&GPIO_Pin_4)
			TempB |= 0x01;
			GPIOB->BSRR=GPIO_Pin_9;		     	    //AD7606_SCK_HIGH，停止数据传输;
		}	
	  GPIOA->BSRR=GPIO_Pin_7;               //AD7606_SCS_HIGH，CS拉高，数据传输不使能;		
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
		GPIOA->BRR=GPIO_Pin_7;               //AD7606_SCS_LOW，AD7606的CS拉低，使能数据传输;
		for(k=0;k<16;k++)
		{ 
			GPIOB->BRR=GPIO_Pin_9; 		     	    //AD7606_SCK_LOW，开始数据传输;
			TempA += (GPIOB->IDR&GPIO_Pin_8)?1:0; TempA<<=1;   //读取DoutA通道数据，共16位
			TempB += (GPIOC->IDR&GPIO_Pin_4)?1:0; TempB<<=1;   //读取DoutB通道数据，共16位
			GPIOB->BSRR=GPIO_Pin_9;		     	    //AD7606_SCK_HIGH，停止数据传输;
		}	
	  GPIOA->BSRR=GPIO_Pin_7;               //AD7606_SCS_HIGH，CS拉高，数据传输不使能;		
		buf[j]=(short int)TempA;
		buf[4+j]=(short int)TempB;
	} 
}

