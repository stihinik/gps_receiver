#include "stm32f4xx_hal.h"
#include "stdlib.h"

extern UART_HandleTypeDef huart3;

//Record GPS data in the array
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
//translation of necessary data after a comma in line
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
	  to[s]=from[i];
		i++;
		s++;
	}
}
//transfer of the necessary data after to float
void StringInFloat(char to, char *from, uint8_t next_comma)
{
	char string[20];
	char str;
	Output_Line(string,from,next_comma);
	strcpy(&str,string);
	to=atof(&str);
}
//Search for a comma after which the GGA data begin
uint8_t SearchCommaGGA(char from[300])
{
	uint16_t counter=1;
	uint8_t comma=0;
	while(from[counter]!='$')
	{
		if(from[counter]==',')
		{
			comma++;
		}
		counter++;
	}
	return comma;
}

