#include "stm32f4xx_hal.h"

#define DEVICE_ADDRESS   0x08020000
#define DEVICE_SECTOR FLASH_SECTOR_5


uint32_t flash_read(uint32_t address) 
{
   return (*(uint32_t*) address);
} 


void ReadDeviceAddress(char* Dout,char* dout2) 
{
	uint32_t i,s;
  for (i=0;i<20;i++)
	{
		Dout[i] = flash_read(DEVICE_ADDRESS+i*0x10);
	}
	s=0;
	for (i=20;i<40;i++)
	{
		dout2[s] = flash_read(DEVICE_ADDRESS+i*0x10);
		s++;
	}
}
void WriteDeviceAddress(char* data,char*map)
{
  uint8_t i,s;

	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(DEVICE_SECTOR, FLASH_VOLTAGE_RANGE_3);
	for (i=0;i<20;i++)
	{
	 HAL_FLASH_Program(TYPEPROGRAM_WORD, DEVICE_ADDRESS+i*0x10, data[i]);
	}
	s=0;
	for (i=20;i<40;i++)
	{
	 HAL_FLASH_Program(TYPEPROGRAM_WORD, DEVICE_ADDRESS+i*0x10, map[s]);
		s++;
	}
	HAL_FLASH_Lock();
}