/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define SPI1_DR_8b          (*(__IO uint8_t *)((uint32_t)SPI1 + 0x0C))
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
void matrix_transmit(uint8_t adress, uint8_t data);
void Spi1_send (uint8_t data);
void matrix_init();
void matrix_clear();
void matrix_test();
int calc_degree(uint8_t x, uint8_t dg);
void print(char* string);



#define GridSize 8
volatile uint8_t tim1 = 0;
volatile uint8_t tim2 = 0;



int** GridInit(int gen);
void GridShow(int** grid);
void GridCopy(int*** togrid, int** fromgrid);
void GridNext(int*** togrid, int** fromgrid);
int CellCheck(int** grid, int x, int y);
int GridSame(int** grid1, int** grid2);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

int GridSame(int** grid1, int** grid2){
	int flag = 1;
	for(int i = 0; i<GridSize; i++){
		for(int j = 0; j<GridSize; j++){
			if(grid1[i][j] != grid2[i][j]){
				flag = 0;
				return flag;
			}
		}
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)"s", strlen("s"), 1000);
	return flag;
}
int** GridInit(int gen){
    srand(gen);
    int** grid = NULL;
    grid = (int**)malloc(sizeof(int*)*GridSize);
    for (int i = 0; i<GridSize; i++){
        grid[i] = (int*)malloc(sizeof(int)*GridSize);
        for(int j = 0; j < GridSize; j++){
            if(gen!=0)
                grid[i][j] = rand()%2;
            else
                grid[i][j] = 0;
        }
    }
    return grid;
}
void GridShow(int** grid){
	matrix_clear();
	uint8_t gx = 0;
	uint8_t gy = 0;
	char trans_str[64] = {0,};


    for(int i = 0; i < GridSize; i++){
    	gy = i+1;
        for(int j = 0; j < GridSize; j++){
        	gx = gx*2;
        	gx = gx+grid[i][j];
        }
        matrix_transmit(gy,gx);
        gx = 0;
        gy = 0;
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", strlen("\n\r"), 1000);
}
void GridToComport(int** grid){
	char trans_str[64] = {0,};
	for(int i = 0; i < GridSize; i++){
	    snprintf(trans_str, 63, "%d %d %d %d %d %d %d %d\n\r", (uint16_t)grid[i][0], (uint16_t)grid[i][1], (uint16_t)grid[i][2], (uint16_t)grid[i][3], (uint16_t)grid[i][4], (uint16_t)grid[i][5], (uint16_t)grid[i][6], (uint16_t)grid[i][7]);
	    HAL_UART_Transmit(&huart2, (uint8_t*)trans_str, strlen(trans_str), -1);
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)"\n", strlen("\n"), 1000);
}
void GridCopy(int*** togrid, int** fromgrid){
    int** temp = *togrid;
    for(int i = 0; i < GridSize; i++){
        for(int j = 0; j < GridSize; j++){
            temp[i][j] = fromgrid[i][j];
        }
    }
}
int CellCheck(int** grid, int x, int y){
    int count = 0;
    for(int i = -1; i <= 1; i++){
        for(int j = -1; j <= 1; j++){
            if(i==0 && j==0){
            }
            else
                if((x+i>=0) && (x+i<GridSize) && (y+j>=0) && (y+j<GridSize)){
                    count += grid[x+i][y+j];
                }

        }
    }
    return count;
}
void GridNext(int*** togrid, int** fromgrid){
    int** temp = *togrid;
    for(int i = 0; i < GridSize; i++){
        for(int j = 0; j < GridSize; j++){
            if(fromgrid[i][j] == 0){
                if(CellCheck(fromgrid, i, j) == 3){
                    temp[i][j] = 1;
                }
            }
            else{
                if((CellCheck(fromgrid, i, j) == 3) || (CellCheck(fromgrid, i, j) == 2)){
                    temp[i][j] = 1;
                }
                else
                    temp[i][j] = 0;
            }
            //temp[i][j] = CellCheck(fromgrid, i, j);
        }
    }
}

void GridPoint(int** temp,int x, int y){
	if(temp[x][y] == 1) temp[x][y] = 0;
	else temp[x][y] = 1;
	GridShow(temp);
}


void matrix_transmit(uint8_t adress, uint8_t data){
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    Spi1_send(adress);
    Spi1_send(data);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

void Spi1_send (uint8_t data)
{
		SPI1_DR_8b = data;
		while (!(SPI1->SR & SPI_SR_TXE));
		while (SPI1->SR & SPI_SR_BSY);
}

void matrix_init(){
    matrix_transmit(0x0C,0x00); //matrix off
    matrix_transmit(0x09,0x00); //no decoding
    matrix_transmit(0x0A,0x0A); //brightness
    matrix_transmit(0x0B,0x07); //scan_limit 8 leds
    matrix_transmit(0x0F,0x01); // test mode on
    HAL_Delay(1000);
    matrix_transmit(0x0F,0x00); // test mode off
    matrix_transmit(0x0C,0x01); //matrix on
}
void matrix_clear(){
	for(uint8_t y = 0x01; y<0x09; y++){
		matrix_transmit(y, 0x00);
	}
}
void matrix_test(){
	for(uint8_t y = 0x01; y<0x09; y++){
		for(uint8_t x = 0x01; x<0xFF; x++){
			matrix_transmit(y, x);
			HAL_Delay(10);
			//matrix_clear();
		}
	}
}
int calc_degree(uint8_t x, uint8_t dg)
{
	for(int i = 0; i < dg; i++){
		x*=x;
	}
	return x;
}


void ADC_Select_CH8 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	ADC1->CHSELR = 9;
	  sConfig.Channel = ADC_CHANNEL_8;
	  sConfig.Rank = 1;
	  //sConfig.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH9 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	ADC1->CHSELR = 10;
	  sConfig.Channel = ADC_CHANNEL_9;
	  sConfig.Rank = 2;
	  //sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Spi1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER |=  GPIO_MODER_MODER6_0;
    GPIOA->OTYPER &=  ~GPIO_OTYPER_OT_6;
    GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER7);
    GPIOA->MODER |=  GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1;
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL7);

    SPI1->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;                   // задаем режим работы SPI (полярность и фазу)
    SPI1->CR1 |= SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;             // задаем режим передачи

    SPI1->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;    // задаем формат данных (разрядность)
    SPI1->CR1 &= ~SPI_CR1_LSBFIRST;                             // задаем способ передачи (формат фрейма)
    SPI1->CR1 |= SPI_CR1_SSM;                                  	// задаем режим управления NSS (аппаратное или программное)
    SPI1->CR1 |= SPI_CR1_SSI;                                  	// задаем режим управления NSS (аппаратное или программное)
    SPI1->CR2 &= ~SPI_CR2_SSOE;                                 // задаем режим внешнего вывода NSS
    SPI1->CR1 |= SPI_CR1_MSTR;                                  // задаем режим работы модул SPI (ведущий или ведомый)
    SPI1->CR1 |= SPI_CR1_SPE;                                   // включаем модуль SPI
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
        if(htim->Instance == TIM1) //check if the interrupt comes from TIM1
        {

        	tim1 = 1;
        }
        if(htim->Instance == TIM3) //check if the interrupt comes from TIM3
        {

           	tim2 = 1;
        }
}
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
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Spi1_init();
  HAL_Delay(100);
  matrix_init();
  matrix_clear();
  //matrix_test();


  int** grid1 = NULL;
  int** grid2 = NULL;
  int step = 1;
  grid1 = GridInit(step);
  grid2 = GridInit(0);

  GridCopy(&grid2, grid1);

  HAL_ADC_Init(&hadc);

  char str[64] = {0,};

  HAL_Init();

  uint8_t posx = 0;
  uint8_t posy = 0;
  uint8_t vx = 0;
  uint8_t vy = 0;
  uint8_t timer = 0;
  uint8_t mod_change = 0;
  int AD_RES0 = 0;
  int AD_RES1 = 0;
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  if (tim2 == 1){
		  sprintf(str, "x=%d y=%d\n\r\0",AD_RES0, AD_RES1);
		  HAL_UART_Transmit(&huart2, str, strlen(str), 10);
		  GridToComport(grid1);

		  if(timer == 0 && tim1 == 1){
		  		  GridNext(&grid1, grid2);
		  		  if(GridSame(grid1, grid2) == 1){
		  				  step++;
		  				  grid1 = GridInit(step);
		  		  }
		  		  GridCopy(&grid2, grid1);
		  		  GridShow(grid1);
		  		  GridToComport(grid1);
		  }
		  else if(timer > 0 && tim2 == 1){
			  if(tim1 == 1){
				  posx += vx;
				  posy += vy;
				  if(posy < 0) posy = GridSize-1;
				  if(posy >= GridSize) posy = 0;
				  if(posx < 0) posx = GridSize-1;
				  if(posx >= GridSize) posx = 0;
				  if(mod_change > 0){
					  GridPoint(grid1, posx, posy);
				      mod_change = 0;
				  }
			  }
			  GridPoint(grid1, posx, posy);
			  timer -= 1;
		  }


		  tim1 = 0;
		  tim2 = 0;
	  }




	  ADC_Select_CH8();
	  HAL_ADC_Start(&hadc);
	  HAL_ADC_PollForConversion(&hadc, 1000);
	  AD_RES0 = HAL_ADC_GetValue(&hadc);
	  HAL_ADC_Stop(&hadc);
	  //HAL_Delay(10);

	  ADC_Select_CH9();
	  HAL_ADC_Start(&hadc);
  	  HAL_ADC_PollForConversion(&hadc, 1000);
  	  AD_RES1 = HAL_ADC_GetValue(&hadc);
  	  HAL_ADC_Stop(&hadc);
  	  //HAL_Delay(10);



  	  if(AD_RES0 < 700)
  		  vx = -1;
  	  else if(AD_RES0 > 3300)
  		  vx = 1;
  	  else
  		  vx = 0;

  	  if(AD_RES1 < 700)
  		  vy = -1;
  	  else if(AD_RES1 > 3300)
  	  	  vy = 1;
  	  else
  	  	  vy = 0;

  	  if (vx != 0 || vy != 0){
  		  timer = 10;
  	  }

  	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET){
  		  mod_change = 1;
  		  HAL_UART_Transmit(&huart2, "touch_me /r/n/0", strlen("touch_me /r/n/0"), 10);
  		  HAL_Delay(10);
  	  }

    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 12499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
