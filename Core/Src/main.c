/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "VL53L1X_api.h"

#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include "debug.h"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t dev = 0x52;
int status = 0;

char uart_buf[256];

static const uint16_t time_budgets[] = {20, 50, 100, 200, 500};
static uint8_t current_budget = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t data_ready = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == LASER_INT_Pin) {
    data_ready = 1;
  }
}

void reset_laser_int(){
  data_ready = 0;
  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
}

/**
 * @brief Wake up module from hardware standby mode (no i2c comminucation)
*/
void laser_on(){
  HAL_GPIO_WritePin(LASER_SHUT_GPIO_Port, LASER_SHUT_Pin, GPIO_PIN_SET);
}
/**
 * @brief Make module into hardware standby mode
*/
void laser_off(){
  HAL_GPIO_WritePin(LASER_SHUT_GPIO_Port, LASER_SHUT_Pin, GPIO_PIN_RESET); // turn off
}

/**
 * 
 */
VL53L1X_ERROR laser_init(uint16_t dev/* , uint32_t timeout_ms */){
  uint8_t state = 1;
  while(state){
    //TODO: timeout
    status = VL53L1X_BootState(dev, &state);
    DEBUG_transmit_fmt("status = %d", status);
    HAL_Delay(5);
  }
  VL53L1X_SensorInit(dev);
  return 0;
}

uint8_t uart_command = '\0';
uint8_t uart_flag = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (huart == &huart1){
    uart_flag = 1;
  }
}

void send_CSV_header(){
  sprintf(uart_buf, "%s;%s;%s;%s;%s;%s;%s;%s\n",
    "time",        "dist_mode", "budget",    "signal_rate", "ambient_rate", "spad_num", "status", "dist"
  );
  HAL_UART_Transmit(&huart1, (uint8_t*) uart_buf, strlen(uart_buf), 100);
}

void send_CSV_data(uint8_t dist_mode, uint16_t time_budget, uint8_t status, uint16_t dist, uint16_t signal_rate, uint16_t ambient_rate, uint16_t spad_num){
  sprintf(uart_buf, "%lu;%u;%u;%u;%u;%u;%u;%u\n",
    HAL_GetTick(), dist_mode,   time_budget, signal_rate,   ambient_rate,   spad_num,   status,   dist
  );
  HAL_UART_Transmit(&huart1, (uint8_t*) uart_buf, strlen(uart_buf), 100);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t byteData, sensorState=0;
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t dist_mode;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);

  DEBUG_transmit_str("# booting laser");
  laser_on();
  laser_init(dev);
  DEBUG_transmit_str("# inited laser");
  
  // TODO: function and structs for operate on laser
  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
  status = VL53L1X_SetTimingBudgetInMs(dev, time_budgets[current_budget]); /* in ms possible values [20, 50, 100, 200, 500] */
  status = VL53L1X_SetInterMeasurementInMs(dev, 500); /* in ms, IM must be > = TB */ //TODO: checher
  DEBUG_transmit_str("# configured laser");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  send_CSV_header();
  HAL_UART_Receive_IT(&huart1, &uart_command, 1);
  status = VL53L1X_StartRanging(dev);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (data_ready){
      status = VL53L1X_GetDistanceMode(dev, &dist_mode);

      status = VL53L1X_GetRangeStatus(dev, &RangeStatus); //TODO: stringify status 
      status = VL53L1X_GetDistance(dev, &Distance);
      status = VL53L1X_GetSignalRate(dev, &SignalRate);
      status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
      status = VL53L1X_GetSpadNb(dev, &SpadNum);
      
      send_CSV_data(dist_mode, time_budgets[current_budget], RangeStatus, Distance, SignalRate, AmbientRate, SpadNum);

      reset_laser_int();
    }

    // process uart_command
    if (uart_flag){
      status = VL53L1X_StopRanging(dev);

      if (uart_command == '1'){
        status = VL53L1X_SetDistanceMode(dev, 1); /* 1=short, 2=long */
      }else if (uart_command == '2'){
        status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
      }else if (uart_command == '3'){
        if (current_budget == sizeof(time_budgets)/sizeof(time_budgets[0]) - 1)
          current_budget = -1;
        status = VL53L1X_SetTimingBudgetInMs(dev, time_budgets[++current_budget]);
      }

      // restart ranging
      status = VL53L1X_StartRanging(dev);

      // reset uart rx
      uart_flag = 0;
      HAL_UART_Receive_IT(&huart1, &uart_command, 1);
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
