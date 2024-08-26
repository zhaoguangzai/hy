/***************************************
*0x0807F800-0x0807F801: FLUSH A times :0~20s
*0x0807F802-0x0807F803: H2 window of Var
*
*
*
*
*
*
*
*
*
***************************************/
#include <stdio.h>
#include "stm32f10x_flash.h"
#include "Read_Write_Flash.h"

void FLASH_WriteByte(u32 addr,uint16_t Data)
{
	
	FLASH_Status FLASHstatus = FLASH_COMPLETE;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR); 
  //FLASHstatus=FLASH_ErasePage(START_ADDR);
	if(FLASHstatus == FLASH_COMPLETE)
	{
		FLASHstatus = FLASH_ProgramHalfWord(START_ADDR+addr*2, Data);
	}
	FLASH_Lock();
}



/**
 *@���ܣ����ڲ�FLASH��ȡnum�ֽ�����
 *@����1��ReadAddress:������ʼ��ַ
 *@����2: *dest_Data:��ȡ�������ݻ����׵�ַ
 *@����3: num: ��ȡ�ֽڸ���
 */
uint16_t  i_1;
void ReadFlashData(uint8_t *dest_Data, uint32_t num)
{

	for(i_1=0;i_1<num;i_1++)
	{
		dest_Data[2*i_1]  = *(__IO uint8_t*)(START_ADDR+2*i_1);	//��ȡ����
	//	printf("data[2*%d]=%d\r\n",i,dest_Data[2*i]);
		dest_Data[2*i_1+1] = *(__IO uint8_t*)(START_ADDR+2*i_1+1);	//��ȡ����
	//	printf("data[2*%d+1]=%d\r\n",i,dest_Data[2*i+1]);
	}
	
	return;
}
