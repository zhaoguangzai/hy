/**
******************************************************************************
* @file    			main.c
* @author  			WIZnet Software Team
* @version 			V1.0
* @date    			2015-02-14
*
******************************************************************************
*/ 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "stm32f10x.h"

#include "ADC_Channel.h"

#include "bsp_usart1.h"
#include "bsp_i2c_ee.h"
#include "bsp_i2c_gpio.h"
#include "bsp_led.h"

#include "w5500.h"
#include "W5500_conf.h"
#include "socket.h"
#include "utility.h"
/*app函数头文件*/
#include "udp_demo.h"

/*funciton函数头文件*/
#include "function.h"
#include "Read_Write_Flash.h"

#include "ad7606.h"
#include "Read_Write_Flash.h"

//最小和最大电势值

#define    Start_Mea     471                //1.5mV -----500℃
#define    Max_Mea       1884                //10mV*15/5*8192----- 1800   
 


#define    Temp_Start       6               //0.235mv[40℃]*191.7/5*8192 =
#define    Temp_End        8192
#define     MV_JUMP        5             //输出6mV的跳动就很大了

//气体压强变量

#define  GAS_PRE								10.10
#define  Aver_Num								2
#define  Gas_Pre_High						3
#define  Gas_Pre_Low						4



// 计数器

//零位
float       RS_1000=1000;//      1000                //13mV*4.7/5*8192----- 1300    
uint8_t 		zero_T_cnt;

float       sum_zero_T;
//实际温度
int16  		actual_T_vol[20];
uint16      max_min_temp;
uint8_t 		actual_T_cnt;

uint8_t     zero_flag;
uint16      sum_flag;

float       sum_actual_T;

//存储信号
uint8_t     T_type;






extern __IO uint16_t ADC_ConvertedValue[NOFCHANEL];
float ADC_ConvertedValueLocal[NOFCHANEL];    

volatile system_function_t 			 m_func_status;
volatile system_status_t  			 m_work_status; 
volatile system_status_t  			 m_work_status_adj;
uint16	 Gas_pre_one=2500;
uint16	 Gas_pre_two=4500;
uint16   Max_Meas_Time = 45;
uint16 	 Interval =5;
uint16   Lv_Par =5;
uint16   Low_Par = 300;
extern uint8_t 			startcurve;
//UART
uint8_t   str_tran[15];
uint8_t    parameter1;
uint8_t    parameter2;
uint32_t      ppm_real0;
uint32_t      ppm_real1;
uint32_t      ppm_real2;
uint32_t      ppm_real3;
extern uint8_t			uart_full_flag;				
extern uint8_t     	uart_buff[UART_BUFF_SIZE];
extern uint8_t			uart_rec_size;


extern uint8_t			uart3_full_flag;				
extern uint8_t     	uart3_buff[UART_BUFF_SIZE];
extern uint8_t			uart3_rec_size;


extern  uint8_t     uart1_rec_flag;
extern  uint8_t     uart1_buff[UART1_BUFF_SIZE];
extern  uint8_t     button_trigger_flag;

uint8 read_press[8] = {0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A}; //suc
uint8 read_press1[8] = {0x02,0x03,0x00,0x00,0x00,0x01,0x84,0x39};
//uint8 read_press[8] = {0x01,0x06,0x00,0x07,0x00,0x03,0x78,0x0A}; //suc
//uint8 read_press1[8] = {0x02,0x06,0x00,0x07,0x00,0x03,0x78,0x39};
uint8 read_H2[4]      = {0x50, 0x30, 0x3f, 0x0d}; 

uint8 Zero_H2[7]      = {0x50, 0x36, 0x30, 0x3D,0x46,0x30,0x0D};  //50 36 30 3D 46 30 0D 
uint8 Zero_setting[8] = {0x50, 0x33, 0x39, 0x3D,0x46,0x32,0x31, 0x0d};

uint8 read_thermal[5] = {0x50, 0x35, 0x33, 0x3f, 0x0d};
uint8 one_time;
uint8 one_time_flag;
//UDP
uint8 rec_buffer[30]; 
uint8 buff[37]; 
uint16 r_len=0;

// CNT
extern uint8_t		sec_flag; 	
extern uint8_t    Flush_Sec;
extern uint8_t 		flag_10s ;
extern uint8_t 		sec_cnt;
// H2/Gas
uint16 Gas_Blow_Pre[Aver_Num];
uint16 Gas_Suc_Pre[Aver_Num];	
uint32 Total_Pre_b;
uint32 Total_Pre_s;
uint8_t H_PPM1;
uint8_t H_PPM2;
uint8_t H_PPM3;
uint8_t H_PPM4;
uint8_t H_PPM5;
int8_t H_PPM_temp;
uint8_t Thermal1;
uint8_t Thermal2;
uint8_t Thermal3;
uint8_t Thermal4;

uint8_t real_H1;
uint8_t real_H2;
uint8_t real_H3;
uint8_t real_H4;
uint8_t Temp1;
uint8_t Temp2;
uint8_t Temp3;
uint8_t Temp4;
float H2_Zero;    //零位氢含量
uint16 Temp_H;

uint16 Humidity;





uint32_t H_PPM;
uint32_t Thermal;
uint32_t T_temp;

//
uint8_t PPM_rec_flag=0;
float  yuzhi;
uint32_t  real_H;
float  result_H[10]; 
float  filter_H[10];
float  H_com_pre;
float  Pre_s;
uint8_t cnt;
uint8_t cnt_temp;
float  Abs_result_H;
// leakage parameter 
uint16_t LEAK_PARA = 2000;
// prenaumic parameter



// FLASH
uint8_t read_data_from_flash;
uint8_t *point_flash=&read_data_from_flash;

//flag
uint8_t enable_flag;   //UDP_rec标志位


static void Delay(__IO u32 nCount); 
void UDP_rec(void);
void UDP_send_message(void);
void clear_buffer(void);
void Read_pre(void);
void Read_Gas_vol(void);
void Read_temp(void);
float Int_to_float(uint32_t source,int8_t beishu);
float Suc_deal(uint32_t  suc_press);
uint16 min_value(uint16 *point,uint8 num);
uint16 max_value(uint16 *point,uint8 num);



unsigned char socketstatus;
unsigned char i;
unsigned short int DataA[8];  //AD实时下载数组 8通道
int16 zero_setting;
int scount=0;				  //AD发送打包计数
int Fs;               //采样率
unsigned char array[6];
unsigned char IPnum[13]="b192168000011";
//int sac=0;				  //发送点数限制 测试用
/*AD读写时序配置，PWM接口开启 并执行初始化 */
void PWMON(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;               //结构体声明 名称GPIO_InitStructure{pin,speed,mode}
	TIM_OCInitTypeDef TIM_OCInitStructure;               //结构体声明 名称TIM_InitStructure{}
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;			   //定时器配置
		
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;            //管脚选择为GPIOB7，作为PWM脉冲输出引脚
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;            //复用推挽输出，就是用作PWM输出		          
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;          //最高输出速率50MHz
  	GPIO_Init(GPIOB, &GPIO_InitStructure);                     //PortB在这里选择，把结构体里的参数直接导进去
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);          //开启TIM4时钟
	TIM_DeInit(TIM4); 										                       //配置TIM4恢复为复位值
	TIM_TimeBaseStructure.TIM_Period =1000-1; 	                 //大于3200,8路通道都不干采样率设置7200/72M	，TIM_Period设置自动装入的值，累计（TIM_Period+1）个时钟脉冲后产生更新或中断			   
	TIM_TimeBaseStructure.TIM_Prescaler = 91;                      //不分频 //TIM_Prescaler设置预分频系数，定时器的时钟频率=72Mhz/(TIM_Prescaler+1) 计数一次时间1us   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //设置时钟分割   0  
	//TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数 
	TIM_CounterModeConfig(TIM4, TIM_CounterMode_Up);        //设置TIMx计数器模式
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;		          //设置重复溢出次数，就是多少次溢出后会给你一次中断，一般设置为0，只有高级定时器才有用
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;	                 //PWM模式设置 与通道设置TIM4_CH2      
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;        //比较输出使能
	TIM_OCInitStructure.TIM_Pulse =0;					      		 //占空比设置
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	         //当计数器<127时,PWM为高电平 ，极性为正
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);		   //根据指定的值初始化TIM4
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //开启TIM4通道2的预装载寄存器
	TIM_ARRPreloadConfig(TIM4, ENABLE);                //设置是否使用 预装载缓冲器			 
	TIM_Cmd(TIM4, ENABLE);                             //开启TIM4
	TIM4->CCR2 =TIM_TimeBaseStructure.TIM_Period/2;	                               //PWM占空比设置，占空比=TIM4->CCR2/Period
	Fs=1000/TIM_TimeBaseStructure.TIM_Period;      //采样率
}


/*配置外部中断 使用PB5 */
void ExtINTConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
	EXTI_DeInit();											 //外中断初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //GPIOB5用作外部中断输入引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //最高输出速率50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 			 //上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);	//PB5中断模式设置
	EXTI_ClearITPendingBit(EXTI_Line5);							//清除中断标志位
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;					//使用中断线5，即将PB5作为外部中断线5
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;			//中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;     //下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;					//开中断
	EXTI_Init(&EXTI_InitStructure);
		
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);				//外部中断优先级配置
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//响应级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//启动
	NVIC_Init(&NVIC_InitStructure);
}

/*AD采样引致的外部中断处理*/
unsigned char Send_Data_Buffer[2048];   //声明	 
unsigned char one_convert_flag =0;
void EXTI9_5_IRQHandler(void) 
{	

	
	 EXTI_InitTypeDef EXTI_InitStructure;	
  if(EXTI_GetITStatus(EXTI_Line5) != RESET ) 
	{
	
	    EXTI_InitStructure.EXTI_LineCmd = DISABLE;					//关中断
	    AD7606ReadOneSample(DataA);				                  //读取7606采集数据，这个程序的问题是我只要1个通道数据的时候，没必要花时间读其余7个通道。
		                                                      //将读到的AD数据转移至发送缓冲区，当积累一定数据N后，这个需要测试优化，一并发送。这个逻辑里边潜在问题就是发送缓冲区内数据还没发送完时，又来覆盖的问题。
  	  memcpy(&Send_Data_Buffer[8+scount*16],DataA,16);
		scount++;	
		one_convert_flag =1;
		if(scount==10)
		{
		     
      //one_convert_flag =1;
			scount=0;
		}
	
		   EXTI_ClearFlag(EXTI_Line5);  			 //清除中断标志位
		   EXTI_ClearITPendingBit(EXTI_Line5);		 //清除外部中断线5的挂起位
	     EXTI_InitStructure.EXTI_LineCmd = ENABLE;					//开中断
	  
	
	}  
}

int main(void)
{ 

  /*初始化USART1*/
 // Debug_USART_Config();
//	char *pbuf;


	uint8 i;
//  uint32_t  change;
//	uint8_t   cnt_10;
	//crc
// unsigned short tmp = 0xffff;
// unsigned short ret1 = 0;
 //unsigned char buff_H[8] = {0};

 //int i_n ;

	//test flash

	uint8_t  Par_change_flag =0;
	FLASH_Status FLASHstatus = FLASH_COMPLETE;
  
	// status definition
	
	m_func_status = H_MEASURE;
	
	//init
	
	systick_init(72);						/*初始化Systick工作时钟*/
	USART1_Config();						/*初始化串口通信:115200@8-n-1*/
	//RS485_Config();             /*初始化485使用的串口，使用中断模式接收*/
	//UART3_RS485_Config();    	
	TIM3_Init(9999,7199);
	Flush_Sec	= 8;
	AD7606Initialization();
//	LED_GPIO_Config();
	
  ExtINTConfig();
	PWMON();

 
	gpio_for_w5500_config();						 /*初始化MCU相关引脚*/
	reset_w5500();											/*硬复位W5500*/
	set_w5500_mac();										/*配置MAC地址*/
	set_w5500_ip();											/*配置IP地址*/
	socket_buf_init(txsize, rxsize);		/*初始化8个Socket的发送接收缓存大小*/

	do_udp();                         /*UDP 接收与发送*/
	
	//状态初始化
	m_func_status = H_MEASURE;
	m_work_status = Init_Plus_Status ;
 Par_change_flag =1;
 //crc校验
zero_T_cnt  = 0;
actual_T_cnt =0;
Uart1_SendByte(0x55);
delay_ms(500);
delay_ms(500);
delay_ms(500);
delay_ms(500);
delay_ms(500);
delay_ms(500);
sum_flag =0;
for(i = 0; i<20 ;i++)
{
	actual_T_vol[i] = 0;
}
Flush_Sec = 20;

  while(1)
  {
		   if(Par_change_flag == 1)
		  {
				Par_change_flag = 0;
				
        T_type = *(__IO uint8_t*)(START_ADDR);
				
			Uart1_SendByte(T_type);

			}
		
		
     if(one_convert_flag ==1)			
		{
			one_convert_flag =0;
			UDP_send_message();
			switch (m_func_status) 
			{
				
				case H_MEASURE:
					switch(m_work_status)
					{
						
						/* 开机的状态------存储零位***/
							case Init_Plus_Status:
							//printf("Init_Status = %d\r\n",Init_Status);
						  //UART3_SendStr("$001,00#");       //初始化显示0x00，代表开始
						  buff[1] = 0x10;        				//初始化状态指示
						  Uart1_SendByte(0x55);
						  TIM_Cmd(TIM3,ENABLE);
              //delay_ms(100);
              actual_T_vol[4] = actual_T_vol[3] ;								
              actual_T_vol[3] = actual_T_vol[2] ;							
						  actual_T_vol[2] = actual_T_vol[1] ;
						  actual_T_vol[1] = actual_T_vol[0] ;
						  actual_T_vol[0] = DataA[2] ;      //热电偶测量
						  zero_setting =(actual_T_vol[0]+actual_T_vol[1]+actual_T_vol[2]+actual_T_vol[3]+actual_T_vol[4])/5;  //后续需要考虑负数
              //zero_setting = -1;
              //sum_actual_T =  DataA[2] ;  
						
							buff[5] = DataA[1];	
							buff[4] = DataA[1]>>8;  //氧电势
							buff[13] = DataA[0];
							buff[12] = DataA[0]>>8;	//环境温度	 										
							buff[27] =((uint16)zero_setting)%256;		
							buff[26] =((uint16)zero_setting)/256;	
							//buff[25] = DataA[2] ; 	
			        //buff[24] = DataA[1]>>8; 
							
							if(sec_flag == 1)
							{
								    m_work_status = Init_Status;
									  Uart1_SendByte(0xAA);
								    delay_ms(300);
										TIM_Cmd(TIM3,DISABLE);	
										sec_flag =0;
										sec_cnt =0;
								    zero_T_cnt =0;
									  actual_T_vol[2] =0 ;
						        actual_T_vol[1] =0 ;
						        actual_T_vol[0] =0;
								    sum_flag =0;
							 }

						break;
							 
						case Init_Status:            
							buff[1] = 0x10;        				//初始化状态指示
						  Uart1_SendByte(0xAA);
						  //TIM_Cmd(TIM3,ENABLE);
						  //m_work_status = Init_Status;
              //delay_ms(200);
					   	//sum_flag =0;
						  actual_T_vol[2] = actual_T_vol[1] ;
						  actual_T_vol[1] = actual_T_vol[0] ;
							actual_T_vol[0]= DataA[2] ;//热电偶测量

						  enable_flag =0;
							if(DataA[2] >= 8192)
							{
								actual_T_vol[0]  = 16383-DataA[2];  //求绝对值
							}
								
							UDP_rec();
							
							
							if(T_type == 0x01)   //R  ----1000℃ (10.560*191.7/5000*8192)
							{
								Uart1_SendByte(T_type);
								RS_1000 = 3317;
							}
							if(T_type == 0x02)   //S  ----1000℃ (9.587*191.7/5000*8192)
							{
								RS_1000 = 2697;
								Uart1_SendByte(T_type);
							}
						  if(T_type == 0x03)   //B  ----1000℃ ( 4.834*191.7/5000*8192)
							{
								RS_1000 = 1518;
								Uart1_SendByte(T_type);
							}
              sum_actual_T = (actual_T_vol[0] +actual_T_vol[1]+actual_T_vol[2])/3;
							buff[5] = DataA[1];	
							buff[4] = DataA[1]>>8;  //氧电势
							buff[13] = DataA[0];
							buff[12] = DataA[0]>>8;	//环境温度	 										
							buff[9] =((uint16)sum_actual_T)%256;		
							buff[8] =((uint16)sum_actual_T)/256;	
		          
							//Uart1_SendByte(T_type);
							//if(zero_T_cnt > 10)
							//{
               //   max_min_temp = max_value(actual_T_vol,10)-min_value(actual_T_vol,10);
							//}
							if(actual_T_vol[0] < 1000)   //实际电压值
							{
								sum_flag =sum_flag+1;
		 
						  }
							
							if(sum_flag >= 3)
							{
								m_work_status = One_status_plus;
								sum_flag =0;
								//Flush_Sec = 59;        //如果准备时间超过20s，那么就调回去初始阶段
								//TIM_Cmd(TIM3,ENABLE);
								sec_flag =0;
								sec_cnt =0;
								Uart1_SendByte(0x55);
								delay_ms(500);
								delay_ms(500);
								delay_ms(500);
								delay_ms(500);
								Uart1_SendByte(0xAA);
								actual_T_vol[2] = 0 ;
						    actual_T_vol[1] = 0 ;
						    actual_T_vol[0] = 0 ;
							  
							}
							
							
							
							else m_work_status = Init_Status;
						break;
						
						
	


						case One_status_plus:
							buff[1] = 0x11;                    //准备
							actual_T_vol[2] = actual_T_vol[1] ;
						  actual_T_vol[1] = actual_T_vol[0] ;
						  actual_T_vol[0] = DataA[2] ;      //热电偶测量

						  //sum_actual_T =actual_T_vol[0];//(actual_T_vol[0]+actual_T_vol[1]+actual_T_vol[2])/3;						
							// -zero_setting ;
							buff[5] = DataA[1];	
							buff[4] = DataA[1]>>8;  //氧电势
							buff[13] = DataA[0];
							buff[12] = DataA[0]>>8;	//环境温度	 										
							buff[9] =((uint16)sum_actual_T)%256;		
							buff[8] =((uint16)sum_actual_T)/256;	
						  delay_ms(50);
             if((actual_T_vol[0] > RS_1000)&&(actual_T_vol[1]> RS_1000 )&&(actual_T_vol[2]> RS_1000))
							{
								
								m_work_status = Two_Status;
								Flush_Sec =7;
								sec_flag =0;
								sec_cnt =0;
                TIM_Cmd(TIM3,ENABLE);
								

								//actual_T_vol[2] = 0 ;
						    //actual_T_vol[1] = 0 ;
						   // actual_T_vol[0] = 0 ;
							}

							
						break;
						case Two_Status:  //measuring
							//delay_ms(200);
							 buff[1] = 0x12; 
						  actual_T_vol[2] = actual_T_vol[1] ;
						  actual_T_vol[1] = actual_T_vol[0] ;
						  actual_T_vol[0] =DataA[2] ;
						  sum_actual_T =actual_T_vol[0];//(actual_T_vol[0]+actual_T_vol[1]+actual_T_vol[2])/3;
						  //sum_actual_T =sum_actual_T-sum_zero_T;
						  buff[5] = DataA[1];	
							buff[4] = DataA[1]>>8;  //氧电势
							buff[13] = DataA[0];
							buff[12] = DataA[0]>>8;	//环境温度	 										
							buff[9] =((int16)sum_actual_T)%256;		
							buff[8] =((int16)sum_actual_T)/256;	
						  if(sec_flag == 1)
							{
								sec_cnt =0;
								sec_flag =0;
								Flush_Sec = 10;
								TIM_Cmd(TIM3,DISABLE);						  
					
								m_work_status = Thr_Status;
							}
							break;
						case Thr_Status:
							buff[1] = 0x13; 
						  actual_T_vol[2] = actual_T_vol[1] ;
						  actual_T_vol[1] = actual_T_vol[0] ;
						  actual_T_vol[0] =DataA[2] ;
						   UDP_rec();
							if(DataA[2] >= 8192)
							{
								actual_T_vol[0]  = 16383-DataA[2];  //求绝对值
							}
								
						  sum_actual_T = (actual_T_vol[0]+actual_T_vol[1]+actual_T_vol[2])/3;//actual_T_vol[0];
							buff[5] = DataA[1];	
							buff[4] = DataA[1]>>8;  //氧电势
							buff[13] = DataA[0];
							buff[12] = DataA[0]>>8;	//环境温度	 										
							buff[9] =((int16)sum_actual_T)%256;		
							buff[8] =((int16)sum_actual_T)/256;	

              //actual_T_vol[2] = actual_T_vol[1] ;
						  //  actual_T_vol[1] = actual_T_vol[0] ;
						  //  actual_T_vol[0] = DataA[2] ;      
	             
               if((actual_T_vol[0] >= 6700)&&(actual_T_vol[1] >= 6700)&&(actual_T_vol[2] >= 6700))							
								 {
								 sec_cnt =0;
								sec_flag =0;
							  m_work_status =Init_Status;
               	sum_flag =0;
                zero_T_cnt =0;
							 }								 
							break;
						default:break;;
						/////////////////////////////////////////////////////////////////////
					}
				 break;
				case H_ADJUSTMENT:
					switch(m_work_status_adj)
					{
					case	Init_Status:
	           FLASH_Unlock();
	           FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR); 
				     FLASHstatus=FLASH_ErasePage(START_ADDR);
				     FLASH_Lock();
						 FLASH_WriteByte(0,rec_buffer[3]); 
						 delay_ms(50);
            Par_change_flag	= 1;				 
        	  m_work_status_adj = Two_Status;
            //Uart1_SendByte(0x04) ;
	          break;

					case Two_Status:
					
					 // enable_flag =0;
					  m_func_status = H_MEASURE;
					 // m_work_status = Init_Status;
	
					break;
						default:break; 
          }
			   break;
			  default:break;
			 }
		 }
	 }
}

  /*******************************************************************************
  * 函数名       : Read_pre()
  * 函数功能     : 接收压力传感器的信息
  * 输入         : 无
  * 输出         : 无
  *******************************************************************************/
void Read_pre()
{
	  uint8_t  j;
		Total_Pre_b = 0;
		Total_Pre_s = 0;
		for(j = 0;j < Aver_Num; j++)
		{
			RS485_SendStr_length(read_press1,8);
			delay_ms(50);
			//printf("uart_full_flag = %d\r\n",uart_full_flag);
			//while(uart_full_flag == 0);
			if(uart_full_flag == 1)
			{
			//	USART_Cmd(USART3, DISABLE);
				uart_full_flag  = 0;
				
				Gas_Blow_Pre[j] = (uart_buff[Gas_Pre_High]<<8)+uart_buff[Gas_Pre_Low];
			}
			
			RS485_SendStr_length(read_press,8);
			delay_ms(50);
			//while(uart3_full_flag == 0);		
      if(uart_full_flag==1)	
			{				
			//USART_Cmd(RS485_USART, DISABLE);
			uart_full_flag = 0;
			
			Gas_Suc_Pre[j] =  (uart_buff[Gas_Pre_High]<<8)+uart_buff[Gas_Pre_Low];	
			}				
              
			Total_Pre_b =Total_Pre_b + Gas_Blow_Pre[j];
      Total_Pre_s =Total_Pre_s+ Gas_Suc_Pre[j];
		}
		//USART_Cmd(RS485_USART, ENABLE);
		//USART_Cmd(USART3, ENABLE);
		Total_Pre_b = Total_Pre_b/Aver_Num;
    Total_Pre_s = Total_Pre_s/Aver_Num;
	
}


 /*******************************************************************************
  * 函数名       : Read_Gas_vol
  * 函数功能     : 接收GAS传感器的信息
  * 输入         : 无
  * 输出         : 无
  *******************************************************************************/
void Read_Gas_vol()
{

			Uart1_SendStr_length(read_H2,4);
	    //printf("uart1_rec_flag = %d\r\n",uart1_rec_flag);
		
			//while((uart1_rec_flag == 0)|(uart1_buff[2] != 0x0C)){Uart1_SendStr_length(read_H2,8);delay_ms(100);};
      delay_ms(250);
			if(uart1_rec_flag == 1)
			{
			 H_PPM1 = uart1_buff[4]&(0x0F);
			 H_PPM2 = uart1_buff[6]&(0x0F);
			 H_PPM3 = uart1_buff[7]&(0x0F);
			 H_PPM4 = uart1_buff[8]&(0x0F);
  	
	     H_PPM5  = uart1_buff[13]&(0x0F);
			
	     H_PPM_temp =H_PPM5-4;
		   H_PPM = H_PPM1*1000+H_PPM2*100+H_PPM3*10+H_PPM4;
			}
	    uart1_rec_flag  = 0;

		}
 /*******************************************************************************
  * 函数名       : Read_temp
  * 函数功能     : 接收GAS传感器的信息
  * 输入         : 无
  * 输出         : 无
  *******************************************************************************/
void Read_temp()
{
			Uart1_SendStr_length(read_thermal,5);
			delay_ms(250);
    
			if(uart1_rec_flag == 1)
			{	
			Thermal1 = uart1_buff[5]&(0x0F);
			Thermal2 = uart1_buff[6]&(0x0F);
			Thermal3 = uart1_buff[8]&(0x0F);
			Thermal4 = uart1_buff[9]&(0x0F);
			Thermal = (Thermal1 *1000) + (Thermal2 *100) + (Thermal3 *10) + Thermal4;
			}
			uart1_rec_flag  = 0;
}

  /*******************************************************************************
  * 函数名       : UDP_rec
  * 函数功能     : 接收UDP的信息，并根据信息进行功能和模式的判别
  * 输入         : 无
  * 输出         : 无
  *******************************************************************************/

void  UDP_rec()
{

   if(getSn_IR(SOCK_UDPS) & Sn_IR_RECV)
		{
		setSn_IR(SOCK_UDPS, Sn_IR_RECV);                                     
		}	
		if((r_len=getSn_RX_RSR(SOCK_UDPS))>0)                                   
		 {
			 //recvfrom(SOCK_UDPS,rec_buffer, 30, remote_ip,&remote_port);               
			 recvfrom(SOCK_UDPS,rec_buffer, 30, remote_ip1,&remote_port);  

			}	
			

  // enable_flag = 0;
		
   if((rec_buffer[0] == 0xAA)&& (rec_buffer[1] == 0x11))
		{
			m_func_status = H_ADJUSTMENT;
			m_work_status_adj = Init_Status;
			rec_buffer[0] = 0x00; //clear zero
			 rec_buffer[1] = 0x01 ;//clear zero
			 enable_flag = 1;
		}	
}
  /*******************************************************************************
  * 函数名       : UDP_send_message
  * 函数功能     : 发送UDP信息
  * 输入         : 无
  * 输出         : 无
  *******************************************************************************/

void UDP_send_message()
{
	      /****************UDP发送信息***********************/
	    uint8_t k;
	    uint8_t cnt_UDP;
	    if((m_func_status == H_MEASURE)|(m_func_status == H_ADJUSTMENT))
			{
				buff[0] = 0x55;
				buff[2] = real_H1;
				buff[3] = real_H2;
				//buff[4] = real_H3;
				//buff[5] = real_H4;
				
				//buff[6] = H_PPM1;
				//buff[7] = H_PPM2;
				//buff[8] = H_PPM3;
				//buff[9] = H_PPM4;
				
				//buff[10] = Thermal1;
				//buff[11] = Thermal2;
				//buff[12] = Thermal3;	     
        //buff[13] = Thermal4;

				//buff[14] = Temp1;
				//buff[15] = Temp2;	
				//buff[16] = Temp3;
				//buff[17] = Temp4;

				//buff[18] = Thermal1;	
				//buff[19] = Thermal2;	
				//buff[20] = Thermal3;	
				//buff[21] = Thermal4;
				
				buff[22] = 0x00;	
				buff[23] = 0x00;
				for( k =0;k<22;k++)
				{
					buff[23] = buff[k]^buff[23];
				}
				//buff[24] = Temp1;	
				//buff[25] = Temp2;	
			 // buff[26] = Temp1;	
			//	buff[27] = Temp2;				
        cnt_UDP = 28;	
		
			}
			


			remote_port =5000;
			
			sendto(SOCK_UDPS,buff,cnt_UDP, remote_ip, remote_port);
			//////////////////////////////////////////////////

			delay_ms(100);
}
  /*******************************************************************************
  * 函数名       : max_value
  * 函数功能     : 求最大值
  * 输入         : 无
  * 输出         : 无
  *******************************************************************************/
uint16 max_value(uint16 *point,uint8 num)
{
	uint8  j;
	uint16 max_temp =0;
	for(j = 0; j < num; j++)
	{
		if(max_temp < point[j])
		  max_temp =point[j];
	}
	return max_temp;
}
  /*******************************************************************************
  * 函数名       : min_value
  * 函数功能     : 求最小值
  * 输入         : 无
  * 输出         : 无
  *******************************************************************************/
uint16 min_value(uint16 *point,uint8 num)
{
	uint8  j;
	uint16 min_temp =0;
	for(j = 0; j < num; j++)
	{
		if(min_temp < point[j])
		  min_temp =point[j];
	}
	return min_temp;
}


