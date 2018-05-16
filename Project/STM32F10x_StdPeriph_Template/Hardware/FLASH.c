#include "FLASH.h"



/**
  * @brief  对flash进行读写操作
  * @param  data-待读写的数据地址  flag:1 写   0 读
  * @retval e_state: enFlag-读写成功 rstFlag-读写失败
  * 存放时按照字节增顺序依次是：个、十、百、标志位（1表示写入成功）
  */
e_state flashHandle(uint16_t * data,uint8_t flag)
{
/* Porgram FLASH Bank1 ********************************************************/
  /* Unlock the Flash Bank1 Program Erase controller */
	uint32_t Address = 0x00;
	uint32_t dataTmp = 0;
	e_state result = enFlag;

	Address = BANK1_WRITE_START_ADDR;
	if(flag == 1)//写操作
	{
		dataTmp = 0x1000000;
		dataTmp |= (*data/100%10)<<16;
		dataTmp |= (*data/10%10)<<8;
		dataTmp |= (*data%10);
		FLASH_UnlockBank1();
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
		if(FLASH_ErasePage(Address) != FLASH_COMPLETE) result = rstFlag;
		if(FLASH_ProgramWord(Address, dataTmp) != FLASH_COMPLETE) result = rstFlag;
		FLASH_LockBank1();
	}
	else
	{
		dataTmp = (*(__IO uint32_t*) Address);
		if((dataTmp&0xFF000000) == 0x1000000)//确实有写入过标志符
		{
			*data = ((dataTmp&0xFF0000)>>16)*100;
			*data += ((dataTmp&0xFF00)>>8)*10;
			*data += (dataTmp&0x0FF);
		}
		else//数据存放有误，没有读出数据来
		{
			result = rstFlag;
			*data = 0;
		}
	}

  return result;
}
