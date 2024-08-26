/**
******************************************************************************
* @file   		udp_demo.c
* @author  		WIZnet Software Team 
* @version 		V1.0
* @date    		2015-02-14
* @brief   		UDP演示函数
******************************************************************************
**/
#include <stdio.h>
#include <string.h>
#include "w5500_conf.h"
#include "w5500.h"
#include "socket.h"
#include "utility.h"
#include "udp_demo.h"
                                                      /*定义一个2KB的缓存*/	
/**
*@brief		UDP测试程序
*@param		无
*@return	无
*/
void do_udp(void)
{                                                              

//	uint8 buff[2048];                                                          /*定义一个2KB的缓存*/	
	switch(getSn_SR(SOCK_UDPS))                                                /*获取socket的状态*/
	{
		case SOCK_CLOSED:                                                        /*socket处于关闭状态*/
			socket(SOCK_UDPS,Sn_MR_UDP,local_port,0);                              /*初始化socket*/
		  break;
		
		case SOCK_UDP:                                                           /*socket初始化完成*/
			delay_ms(10);


			if(getSn_IR(SOCK_UDPS) & Sn_IR_RECV)
			{
				setSn_IR(SOCK_UDPS, Sn_IR_RECV);                                     /*清接收中断*/
			}

			break;
	}

}

uint8 buffer[128]; 
uint16 len = 128;
void UDP_Send_to()
{
   // socket(SOCK_UDPS, Sn_MR_UDP, local_port, 0);
    buffer[0] = 0x55;
    sendto(SOCK_UDPS, buffer, len, remote_ip, remote_port);
	 // printf("%s\r\n",buff); 
    delay_ms(100);
}


