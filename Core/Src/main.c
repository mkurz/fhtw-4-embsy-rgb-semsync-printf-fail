/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

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
UART_HandleTypeDef huart2;

/* Definitions for blink_red */
osThreadId_t blink_redHandle;
const osThreadAttr_t blink_red_attributes = {
  .name = "blink_red",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for blink_green */
osThreadId_t blink_greenHandle;
const osThreadAttr_t blink_green_attributes = {
  .name = "blink_green",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for blink_blue */
osThreadId_t blink_blueHandle;
const osThreadAttr_t blink_blue_attributes = {
  .name = "blink_blue",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myBinarySem */
osSemaphoreId_t myBinarySemHandle;
const osSemaphoreAttr_t myBinarySem_attributes = {
  .name = "myBinarySem"
};
/* USER CODE BEGIN PV */
int current_led = 0; // 0 = red, 1 = green, 2 = blue

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void led_blink_red(void *argument);
void led_blink_green(void *argument);
void led_blink_blue(void *argument);

/* USER CODE BEGIN PFP */
void switch_color(char*, int, GPIO_TypeDef*, uint16_t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Print given character on UART 2. Translate '\n' to "\r\n" on the fly. */
int __io_putchar(int ch) {
  int ret;
  while ((ret=HAL_UART_GetState(&huart2)) != HAL_UART_STATE_READY)
    ;

  if (ch == '\n') {
    static uint8_t buf[2] = {'\r', '\n'};
    HAL_UART_Transmit_IT(&huart2, buf, sizeof(buf));
  } else {
    static char buf;
    buf = ch;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&buf, 1);
  }
  return ch;
}

int _write(int file, char *ptr, int len) {
  for (int DataIdx = 0; DataIdx < len; DataIdx++) {
    __io_putchar(*ptr++);
  }
  return len;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(GPIOA, RGB_BLUE_Pin|RGB_RED_Pin|RGB_GREEN_Pin);

  printf("\n\nDas Programm wird gestartet\n\n");

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySem */
  myBinarySemHandle = osSemaphoreNew(1, 1, &myBinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blink_red */
  blink_redHandle = osThreadNew(led_blink_red, NULL, &blink_red_attributes);

  /* creation of blink_green */
  blink_greenHandle = osThreadNew(led_blink_green, NULL, &blink_green_attributes);

  /* creation of blink_blue */
  blink_blueHandle = osThreadNew(led_blink_blue, NULL, &blink_blue_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RGB_BLUE_Pin|RGB_RED_Pin|RGB_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RGB_BLUE_Pin RGB_RED_Pin RGB_GREEN_Pin */
  GPIO_InitStruct.Pin = RGB_BLUE_Pin|RGB_RED_Pin|RGB_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void switch_color(char* color, int led, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin) {
  /* Infinite loop */
  for(;;)
  {
    osStatus_t status = osSemaphoreAcquire(myBinarySemHandle, osWaitForever);
    if (status != osOK) {
      // Konnte den Semaphor nicht acquiren, es gab irgendein Problem :(
      continue; // brich hier ab und fang die for-Schleife nochmal von vorne an
    }
    // Der Semaphor wurde ab hier erfolgreich acquired -> wir befinden uns im "kritischen Code"

    if (current_led != led) {
      // Es soll eine andere LED leuchten :( Deshalb brechen wir ab, aber geben vorher noch den Semaphor wieder frei
      printf("Es laeuft der Task fuer Farbe %s aber die Farbe %d ist jetzt dran\n", color, current_led);
      osSemaphoreRelease(myBinarySemHandle);
      osDelay(1);
      continue; // brich hier ab und fang die for-Schleife nochmal von vorne an
    }

    // Es läuft der Task mit der gewünschten Farbe, also drehen wir diese Farbe auf
    printf("Nun kommt die Farbe %s\n", color);
    HAL_GPIO_TogglePin(GPIO_Port, GPIO_Pin); // Schalte LED ein
    osDelay(1000); // Warte x ticks
    HAL_GPIO_TogglePin(GPIO_Port, GPIO_Pin); // Schalte LED wieder aus

    // Nun wechseln wir zu nächsten Farbe
    if(current_led == 2) {
      current_led = 0; // Nach blau soll wieder die rote LED leuchten
    } else {
      current_led++; // Die nächste Farbe kommt dran (nach rot -> grün, nach grün -> blau) (in ihrem jeweiligen Task)
    }

    // Fertig, jetzt nur noch Semaphor freigeben und das wars
    // Mit dem releasen werden alle Tasks, welche auf den Semaphore warten, notified
    // Der erste Task der osSemaphoreAcquire aufgerufen hat kommt zuerst dran (FIFO)
    printf("Farbe %s fertig\n\n", color);
    osSemaphoreRelease(myBinarySemHandle);
    osDelay(1);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_led_blink_red */
/**
  * @brief  Function implementing the blink_red thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_led_blink_red */
void led_blink_red(void *argument)
{
  /* USER CODE BEGIN 5 */
  switch_color("red", 0, RGB_RED_GPIO_Port, RGB_RED_Pin);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_led_blink_green */
/**
* @brief Function implementing the blink_green thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_blink_green */
void led_blink_green(void *argument)
{
  /* USER CODE BEGIN led_blink_green */
  switch_color("green", 1, RGB_GREEN_GPIO_Port, RGB_GREEN_Pin);
  /* USER CODE END led_blink_green */
}

/* USER CODE BEGIN Header_led_blink_blue */
/**
* @brief Function implementing the blink_blue thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_blink_blue */
void led_blink_blue(void *argument)
{
  /* USER CODE BEGIN led_blink_blue */
  switch_color("blue", 2, RGB_BLUE_GPIO_Port, RGB_BLUE_Pin);
  /* USER CODE END led_blink_blue */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

