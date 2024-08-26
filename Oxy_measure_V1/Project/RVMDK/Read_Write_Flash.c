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
 *@功能：从内部FLASH读取num字节数据
 *@参数1：ReadAddress:数据起始地址
 *@参数2: *dest_Data:读取到的数据缓存首地址
 *@参数3: num: 读取字节个数
 */
uint16_t  i_1;
void ReadFlashData(uint8_t *dest_Data, uint32_t num)
{

	for(i_1=0;i_1<num;i_1++)
	{
		dest_Data[2*i_1]  = *(__IO uint8_t*)(START_ADDR+2*i_1);	//读取数据
	//	printf("data[2*%d]=%d\r\n",i,dest_Data[2*i]);
		dest_Data[2*i_1+1] = *(__IO uint8_t*)(START_ADDR+2*i_1+1);	//读取数据
	//	printf("data[2*%d+1]=%d\r\n",i,dest_Data[2*i+1]);
	}
	
	return;
}
