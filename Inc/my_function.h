#ifndef MY_FUNCTION_H
#define MY_FUNCTION_H

	#include "stm32f4xx_hal.h"
	
	#define READ_BUTTON  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)

	//flash_memory
	uint32_t flash_read(uint32_t address) ;
	void WriteDeviceAddress(char* data,char*map);
	void ReadDeviceAddress(char* Dout,char* dout2);
	//LCD
	void Initial_Inscription();
	//GPS
	void WriteGPSinMassiv(char buf[300]);
	void Output_Line(char*to, char*from, uint8_t next_comma);
	void StringInFloat(char*to, char*from, uint8_t next_comma);

#endif 