#include "stm32f4xx_hal.h"
#include "tm_stm32f4_pcd8544.h"

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