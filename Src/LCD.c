#include "stm32f4xx_hal.h"
#include "tm_stm32f4_pcd8544.h"
#include "my_function.h"

//output of the original inscription on the display
void Initial_Inscription()
{
	//Initialize LCD with 0x38 software contrast
	PCD8544_GotoXY(14, 3);
	//Print data with Pixel Set mode and Fontsize of 5x7px
	PCD8544_Puts("STM32F429", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
	PCD8544_GotoXY(15, 13);
	PCD8544_Puts("Discovery", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
	PCD8544_GotoXY(30, 26);
	PCD8544_Puts("2014", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
	//Display data on LCD
	PCD8544_Refresh();
}
////display if the button is pressed once
void First_Button(char late[20], char log[20], char visota[20], char count[20], char dop[20], char dop2[20])
{
		 PCD8544_Clear();
		 //Go to x=14, y=3 position
		 PCD8544_GotoXY(10, 0);
		 PCD8544_Puts(late, PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
		 PCD8544_GotoXY(10, 10);
		 PCD8544_Puts(log, PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
		 PCD8544_GotoXY(0, 20);
		 PCD8544_Puts("h_s=", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
		 PCD8544_GotoXY(25, 20);
		 PCD8544_Puts(visota, PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
		 PCD8544_GotoXY(55, 20);
		 PCD8544_Puts("sp=", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
		 PCD8544_GotoXY(72, 20);
		 PCD8544_Puts(count, PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
		 ReadDeviceAddress(dop,dop2);
		 PCD8544_GotoXY(10, 30);
		 PCD8544_Puts(dop, PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
		 PCD8544_GotoXY(10, 40);
	   PCD8544_Puts(dop2, PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
		 //Display data on LCD
		 PCD8544_Refresh();
}
//display if the button is pressed twice
void Second_Button (char time[20], char v[20], char visota2[20])
{
	 PCD8544_Clear();
	 //Go to x=14, y=3 position
	 PCD8544_GotoXY(10, 0);
	 PCD8544_Puts("time=", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
	 PCD8544_GotoXY(30, 10);
	 PCD8544_Puts(time, PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
	 PCD8544_GotoXY(10, 20);
	 PCD8544_Puts("speed=", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
	 PCD8544_GotoXY(30, 30);
	 PCD8544_Puts(v, PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
	 PCD8544_GotoXY(10, 40);
	 PCD8544_Puts("h_ellip=", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
	 PCD8544_GotoXY(55, 40);
	 PCD8544_Puts(visota2, PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
	 //Display data on LCD
	 PCD8544_Refresh();
}
//display if the button is pressed three times
void Theird_Button( char dat[20])
{
	 PCD8544_Clear();			 
	 PCD8544_GotoXY(10, 10);				
	 PCD8544_Puts("dat=", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
	 PCD8544_GotoXY(30, 20);				
	 PCD8544_Puts(dat, PCD8544_Pixel_Set, PCD8544_FontSize_5x7);					
	 PCD8544_Refresh();
}
//writing data to flash memory if the button is long pressed
void WriteNewCoordin (char late[20], char log[20])
{
	HAL_GPIO_WritePin(GPIOD	,GPIO_PIN_15, GPIO_PIN_SET);
	WriteDeviceAddress(late,log);
	HAL_GPIO_WritePin(GPIOD	,GPIO_PIN_15, GPIO_PIN_RESET);
}

//Writing necessary data to arrays for display
void WriteMasForLCD(char from[300],char to_late[20],char to_log[20], char to_v[20], char to_dat[20], 
                    char to_count[20], char to_visota[20], char to_visota2[20], uint8_t comma)
{
	Output_Line(to_late,from,3);
	Output_Line(to_log,from,5);
	Output_Line(to_v,from,7);
	Output_Line(to_dat,from,9);				
	Output_Line(to_count,from,comma+7);
	Output_Line(to_visota,from,comma+9);
	Output_Line(to_visota2,from,comma+11);
}