/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tm_stm32f4_pcd8544.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



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
float preobr(char*t, uint8_t k)
{
	uint8_t i,s,f;
	char num[20];
	float b,p;
	i=0;
	f=0;
	while (k!=0)
	{
		if(t[i]==','){k--;}
		i++;
	}
	
	s=0;
	while (s!=20)
	{
		num[s]=0;
		s++;
	}
	s=0;
	while (t[i]!=',')
	{
		num[s]=t[i];
		s++;
		i++;
		f++;
	}
	num[s]='.';
	s=0;
	b=1;
	while(num[s]!='.')
	{
		b=b*10;
		s++;
	}
	p=0;
	s=0;
	b=b/10;
	while(f!=0)
	{
		if(num[s]!='.')
		{
		 p=p+b*(num[s]-48);
		 b=b/10;
		}
		s++;
		f--;
	}
	return(p);
}
char prchar(char*t, uint8_t k)
{
	uint16_t i;
	char num;
	i=0;
	while (k!=0)
	{
		if(t[i]==','){k--;}
		i++;
	}
	num=t[i];
	return(num);
}
void prcharmas (char*num, char*t, uint8_t k)
{
	uint16_t i,s;
	i=0;
	while (k!=0)
	{
		if(t[i]==','){k--;}
		i++;
	}
	s=0;
	while (t[i]!=',')
	{
	num[s]=t[i];
		i++;
		s++;
	}
}
void prcharmas2 (char*num, char*t, uint8_t k)
{
	uint16_t i,s,n;
	i=0;
	while (k!=0)
	{
		if(t[i]==','){k--;}
		i++;
	}
	s=0;
	n=0;
	while (n!=2)
	{
		if(t[i+1]==','){n++;}
	  num[s]=t[i];
		i++;
		s++;
	}
}


struct GP {
	char tip[10];
	float time;
	char sost;
	float dat;
	float lat;
  char naplat;
	float log;
  char naplog;
	float v;
	float count;
	float geom;
	float vis1;
	char rvis1;
	float vis2;
	char rvis2;
	
};

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
char buf[300];
char time[20],late[20], log[20],v[20], dat[20], visota[20],visota2[20],count[20],dop[20],dop2[20];
uint8_t data;
uint16_t k,m,i=0;
uint32_t N=0;	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */

	//Initialize LCD with 0x38 software contrast

	PCD8544_Init(0x38);
	HAL_Delay(50);
	PCD8544_Clear();
	HAL_Delay(50);
	PCD8544_GotoXY(20, 20);
	PCD8544_Puts("pasha cool", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
	PCD8544_Refresh();
	HAL_Delay(50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//HAL_UART_Receive(&huart3,&data,2,3);
  while (1)
  {
		
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
			
			i=0;
			k=0;
   		struct GP GPRMC;
				prcharmas(GPRMC.tip,buf,0);
				GPRMC.time=preobr(buf,1);
				prcharmas(time,buf,1);
				GPRMC.sost=prchar(buf,2);
			
				if (GPRMC.sost=='V')
				{ 
					GPRMC.dat=preobr(buf,9);
			  }
				else
				{
						GPRMC.lat=preobr(buf,3);
            prcharmas2(late,buf,3);
            GPRMC.naplat=prchar(buf,4);
						GPRMC.log=preobr(buf,5);
					  prcharmas2(log,buf,5);
            GPRMC.naplog=prchar(buf,6);
						GPRMC.v=preobr(buf,7);
            prcharmas(v,buf,7);					
						GPRMC.dat=preobr(buf,9);
					  prcharmas(dat,buf,9);
						k=1;
						m=0;
 						while(buf[k]!='$')
						{
							if(buf[k]==',')
							{
								m++;
							}
							k++;
						}
						i=0;
						
						struct GP GPGGA;
						while(buf[k+1]!=',')
						{	
							GPGGA.tip[i]=buf[k+1];
							k++;
							i++;
						}
			      GPGGA.time=preobr(buf,m+1);
			      GPGGA.lat=preobr(buf,m+2);
            GPGGA.naplat=prchar(buf,m+3);
						GPGGA.log=preobr(buf,m+4);
            GPGGA.naplog=prchar(buf,m+5);
						GPGGA.count=preobr(buf,m+7);
				    prcharmas(count,buf,m+7);
						GPGGA.geom=preobr(buf,m+8);
						GPGGA.vis1=preobr(buf,m+9);
						GPGGA.rvis1=prchar(buf,m+10);
				    prcharmas(visota,buf,m+9);
						GPGGA.vis2=preobr(buf,m+11);
						GPGGA.rvis2=prchar(buf,m+12);
				    prcharmas2(visota2,buf,m+11);
					}
			i=0;
      k=0;	
	  }		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1)
		{ 
		  if (N<5) {N++;}
			else  
			{
				HAL_GPIO_WritePin(GPIOD	,GPIO_PIN_15, GPIO_PIN_SET);
				WriteDeviceAddress(late,log);
				N=1;
				HAL_GPIO_WritePin(GPIOD	,GPIO_PIN_15, GPIO_PIN_RESET);
			}
	  }

    if ((N==1)&(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0))
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
		else if ((N==2)&(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0))
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
			if ((N==3)&(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0))
			{	
       PCD8544_Clear();			 
			 PCD8544_GotoXY(10, 10);				
			 PCD8544_Puts("dat=", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
			 PCD8544_GotoXY(30, 20);				
			 PCD8544_Puts(dat, PCD8544_Pixel_Set, PCD8544_FontSize_5x7);					

			 PCD8544_Refresh();
			 N=0;
			}
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 
                           PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
