/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "st7789.h"
#include "stdio.h"
#include <stdlib.h>



#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5
#define BWHITE 6
//#define Aqua 7

#define BYELLOW 8

uint16_t color2,color1;


uint16_t RGB(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
//

int cx=80;
int cy=84;
int r=68;
float x[360]; //outer points of Speed gaouges
float y[360];
float px[360]; //ineer point of Speed gaouges
float py[360];
float lx[360]; //text of Speed gaouges
float ly[360];
float nx[360]; //needle low of Speed gaouges
float ny[360];
float x2[360]; //outer points of RPM gaouges
float y2[360];
float px2[360]; //ineer point of RPM gaouges
float py2[360];
float lx2[360]; //text of RPM gaouges
float ly2[360];
float nx2[360]; //needle low of RPM gaouges
float ny2[360];
float sA;
float rA;
float rad=0.01745;

float speedAngle=0; //...speed variable 0-240
float rpmAngle=5;  //.....RPM variable 0-9

char buf[20];


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ST7789_Init();
//	IIC_Init();

//	unsigned int x,y;
//	char buf[20];

//	ST7789_Test();

  int a=120;
  for(int i=0;i<360;i++)
    {
       x[i]=((r-10)*cos(rad*a))+cx;
       y[i]=((r-10)*sin(rad*a))+cy;
       px[i]=((r-14)*cos(rad*a))+cx;
       py[i]=((r-14)*sin(rad*a))+cy;
       lx[i]=((r-24)*cos(rad*a))+cx;
       ly[i]=((r-24)*sin(rad*a))+cy;
       nx[i]=((r-36)*cos(rad*a))+cx;
       ny[i]=((r-36)*sin(rad*a))+cy;
       x2[i]=((r-10)*cos(rad*a))+320-cx;
       y2[i]=((r-10)*sin(rad*a))+cy;
       px2[i]=((r-14)*cos(rad*a))+320-cx;
       py2[i]=((r-14)*sin(rad*a))+cy;
       lx2[i]=((r-24)*cos(rad*a))+320-cx;
       ly2[i]=((r-24)*sin(rad*a))+cy;
       nx2[i]=((r-36)*cos(rad*a))+320-cx;
       ny2[i]=((r-36)*sin(rad*a))+cy;

       a++;
       if(a==360)
       a=0;
    }


  ST7789_Fill_Color(BLACK);
  ringMeter(1020, 0, 1020, 5, 10, 75,3, Aqua);
  ringMeter(1020, 0, 1020, 165, 10, 75,3, Aqua);

  ringMeter1(0, 0, 1020, 13, 18, 67,2,WHITE, BWHITE);
  ringMeter1(0, 0, 1020, 173, 18, 67,2,WHITE, BWHITE);

	  ringMeter1(825, 0, 1020, 20, 25, 60,2,MAGENTA, BWHITE);


//	  ringMeter1(800, 0, 1020, 180, 25, 60,2,MAGENTA,BWHITE);

	     for(int i=0;i<26;i++){
	     if(i<20) {color1=Aqua; color2=WHITE;} else {color1=MAGENTA; color2=MAGENTA;}

	  if(i%2==0) {
		  ST7789_DrawLine(x[i*12],y[i*12],px[i*12],py[i*12],color1);

	  sprintf(buf, "%d", i);
	  ST7789_WriteString(lx[i*12]-5, ly[i*12]-5, buf, Font_7x10, YELLOW, BLACK);
	  }else
		  ST7789_DrawLine(x[i*12],y[i*12],px[i*12],py[i*12],color2);
	  }

	      for(int i=0;i<19;i++){
	      if(i<20) {color1=Aqua; color2=WHITE;} else {color1=MAGENTA; color2=MAGENTA;}

	  if(i%2==0) {
		  ST7789_DrawLine(x2[i*16],y2[i*16],px2[i*16],py2[i*16],color1);

	  sprintf(buf, "%d", i/2);
	  ST7789_WriteString(lx2[i*16]-3, ly2[i*16]-3, buf, Font_7x10, YELLOW, BLACK);
	  }else
		  ST7789_DrawLine(x2[i*16],y2[i*16],px2[i*16],py2[i*16],color2);
	  }


	  ST7789_WriteString(60, 140, "SPEED", Font_7x10, YELLOW, BLACK);
	  ST7789_WriteString(60, 150, "x10", Font_7x10, YELLOW, BLACK);

	  ST7789_WriteString(225, 140, "RPM", Font_7x10, YELLOW, BLACK);
	  ST7789_WriteString(225, 150, "x1000", Font_7x10, YELLOW, BLACK);

	  fillTriangle(130, 15, 150, 5, 150, 25, BLUE);
	  fillTriangle(170, 5, 170, 25, 190, 15, BLUE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  for (int i = 0; i <= 1019; i+=50) {

		    ringMeter(i, 0, 1020, 50, 55, 30,7, GREEN2RED);
		    ringMeter(i, 0, 1020, 210, 55, 30,7, GREEN2RED);

			  fillTriangle(130, 15, 150, 5, 150, 25, BLUE);
			  fillTriangle(170, 5, 170, 25, 190, 15, BLUE);
			  ST7789_DrawFilledCircle(310, 10, 5, RED);
			  ST7789_DrawFilledRectangle(0, 5, 10, 10, GREEN);


			  HAL_Delay(300);
			  fillTriangle(130, 15, 150, 5, 150, 25, YELLOW);
			  fillTriangle(170, 5, 170, 25, 190, 15, YELLOW);
			  ST7789_DrawFilledCircle(310, 10, 5, BLACK);
			  ST7789_DrawFilledRectangle(0, 5, 10, 10, BLACK);


			  HAL_Delay(300);
			  fillTriangle(130, 15, 150, 5, 150, 25, BLUE);
			  fillTriangle(170, 5, 170, 25, 190, 15, BLUE);
			  ST7789_DrawFilledCircle(310, 10, 5, RED);
			  ST7789_DrawFilledRectangle(0, 5, 10, 10, GREEN);


	  }


//	  ringMeter(500, 0, 1020, 100, 120, 60, BLUE2BLUE);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST7789_CS_Pin|ST7789_RST_Pin|ST7789_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ST7789_CS_Pin ST7789_RST_Pin ST7789_DC_Pin */
  GPIO_InitStruct.Pin = ST7789_CS_Pin|ST7789_RST_Pin|ST7789_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
