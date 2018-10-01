#include "stm32f4xx_hal.h"

void WriteGPSinMassiv(char buf[300])
{
	uint8_t data = 0;
	uint16_t i = 0;
		if(huart3.RxXferCount==0)
		{				
			buf[0]='$';
			buf[1]='G';
			buf[2]='P';
			while(data!='R')
				{
					HAL_UART_Receive(&huart3,&data,1,0);
				}
			buf[3]=data;
			i=4;
			while(i<300)
			{
				HAL_UART_Receive(&huart3,&data,1,1);
				buf[i]=data;
				i++;
			}
		}
}

void Output_Line(char*to, char*from, uint8_t next_comma)
{
	uint16_t i,s;
	i=0;
	while (next_comma!=0)
	{
		if(from[i]==','){next_comma--;}
		i++;
	}
	s=0;
	while (from[i]!=',')
	{
	  to[s]=t[i];
		i++;
		s++;
	}
}
void StringInFloat(char to, char*from, uint8_t next_comma)
{
	char string[20];
	Output_Line(string,from,next_comma);
	to=atof(string);
}