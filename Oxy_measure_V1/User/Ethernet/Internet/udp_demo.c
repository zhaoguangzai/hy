/**
******************************************************************************
* @file   		udp_demo.c
* @author  		WIZnet Software Team 
* @version 		V1.0
* @date    		2015-02-14
* @brief   		UDP��ʾ����
******************************************************************************
**/
#include <stdio.h>
#include <string.h>
#include "w5500_conf.h"
#include "w5500.h"
#include "socket.h"
#include "utility.h"
#include "udp_demo.h"
                                                      /*����һ��2KB�Ļ���*/	
/**
*@brief		UDP���Գ���
*@param		��
*@return	��
*/
void do_udp(void)
{                                                              

//	uint8 buff[2048];                                                          /*����һ��2KB�Ļ���*/	
	switch(getSn_SR(SOCK_UDPS))                                                /*��ȡsocket��״̬*/
	{
		case SOCK_CLOSED:                                                        /*socket���ڹر�״̬*/
			socket(SOCK_UDPS,Sn_MR_UDP,local_port,0);                              /*��ʼ��socket*/
		  break;
		
		case SOCK_UDP:                                                           /*socket��ʼ�����*/
			delay_ms(10);


			if(getSn_IR(SOCK_UDPS) & Sn_IR_RECV)
			{
				setSn_IR(SOCK_UDPS, Sn_IR_RECV);                                     /*������ж�*/
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


