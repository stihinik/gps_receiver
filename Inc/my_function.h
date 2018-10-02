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
	void WriteMasForLCD(char from[300],char to_late[20],char to_log[20], char to_v[20], char to_dat[20], 
                      char to_count[20], char to_visota[20], char to_visota2[20], uint8_t comma);
	void First_Button(char late[20], char log[20], char visota[20], char count[20], char dop[20], char dop2[20]);
	void Second_Button (char time[20], char v[20], char visota2[20]);
	void Theird_Button( char dat[20]);
	void WriteNewCoordin (char late[20], char log[20]);
	
	//GPS
	void WriteGPSinMassiv(char buf[300]);
	void Output_Line(char*to, char*from, uint8_t next_comma);
  void StringInFloat(char to, char *from, uint8_t next_comma);
	uint8_t SearchCommaGGA(char from[300]);
#endif 