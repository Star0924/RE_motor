/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "joint.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define t 0.01f  // t = 0.01

#define RX_BUF_SIZE 128
#define TX_BUF_SIZE 256

//#define TIMEOUT_MS 500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile float time = 0;

//volatile uint16_t rx_index = 0;      // 目前寫入到第幾個字元
//uint8_t rx_data;              // 每次中斷接收的單個字元
uint8_t rx_buf[RX_BUF_SIZE];  // 存放一整行字串的陣列
uint8_t tx_buf[TX_BUF_SIZE];
int gripper_status = 0;
// 計時器變數

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Send_Status_To_Pi(void)
{
    // 檢查 UART 是否處於就緒狀態，這是 DMA 發送的必要保護機制，避免覆寫正在傳送的緩衝區
    if (huart2.gState == HAL_UART_STATE_READY)
    {
        // 打包格式：
        // #速度x6, 位置x6, 夾爪狀態x1, 校正(限位)旗標x6 \n
        sprintf((char*)tx_buf, "#%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d\n",
                joint[0].omega, joint[1].omega, joint[2].omega,
                joint[3].omega, joint[4].omega, joint[5].omega,  // 6 個速度
                joint[0].theta, joint[1].theta, joint[2].theta,
                joint[3].theta, joint[4].theta, joint[5].theta,  // 6 個位置
                gripper_status,                                  // 1 個夾爪狀態
                joint[0].calibration, joint[1].calibration, joint[2].calibration,
                joint[3].calibration, joint[4].calibration, joint[5].calibration); // 6 個限位/校正旗標

        // 使用 DMA 發送
        HAL_UART_Transmit_DMA(&huart2, tx_buf, strlen((char*)tx_buf));
    }
}

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  joint_config();
  HAL_TIM_Base_Start_IT(&htim6);
//  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, RX_BUF_SIZE);
  /* USER CODE END 2 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6){
		for(int i=0; i<=5; i++)Joint_Update(&joint[i], t);
		time += t;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // GPIO Encoder A
    if (GPIO_Pin == joint[0].enc.hw.gpio.pinA)
    {
        // 讀取A相與B相
        GPIO_PinState stateA = HAL_GPIO_ReadPin(joint[0].enc.hw.gpio.portA, joint[0].enc.hw.gpio.pinA);
        GPIO_PinState stateB = HAL_GPIO_ReadPin(joint[0].enc.hw.gpio.portB, joint[0].enc.hw.gpio.pinB);

        // 判斷正反轉
        if (stateA == stateB) {
        	joint[0].enc.hw.gpio.counter++;
        } else {
        	joint[0].enc.hw.gpio.counter--;
        }
    }
    //限位開關觸發
    if (GPIO_Pin == GPIO_PIN_10){
    	joint[0].calibration = 1;
    	joint[0].theta = 0;
    }
    if (GPIO_Pin == GPIO_PIN_11){
    	joint[1].calibration = 1;
    	joint[1].theta = 0;
    }
    if (GPIO_Pin == GPIO_PIN_12){
    	joint[2].calibration = 1;
    	joint[2].theta = 0;
    }
    if (GPIO_Pin == GPIO_PIN_13){
    	joint[3].calibration = 1;
    	joint[3].theta = 0;
    }
    if (GPIO_Pin == GPIO_PIN_1){
    	joint[4].calibration = 1;
    	joint[4].theta = 0;
    }
    if (GPIO_Pin == GPIO_PIN_15){
    	joint[5].calibration = 1;
    	joint[5].theta = 0;
    }
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if(huart->Instance == USART2)
//    {
//        // 情況 A：還沒收到換行字元 ('\n')
//        if(rx_data != '\n')
//        {
//            // 確保 rx_index 不會超過陣列大小，預留 1 格給 '\0'
//            if(rx_index < RX_BUF_SIZE - 1)
//            {
//                rx_buf[rx_index++] = rx_data;
//            }
//            // 如果超過了，就直接丟棄後面的字元，直到收到換行為止
//        }
//        // 情況 B：收到換行字元 ('\n')，代表一整包結束
//        else
//        {
//            rx_buf[rx_index] = '\0'; // 補上字串結尾符號
//            int parsed_count = sscanf((char*)rx_buf,
//                                      "$%f,%f,%f,%f,%f,%f,%d",
//                                      &joint[0].omega_ref, &joint[1].omega_ref, &joint[2].omega_ref,
//                                      &joint[3].omega_ref, &joint[4].omega_ref, &joint[5].omega_ref,
//                                      &gripper_status);
//
//            if (parsed_count == 7) {
//                last_rx_time = HAL_GetTick(); // 只有確認收到完整的 7 個數值，才重置斷線計時器
//                Send_Status_To_Pi();
//            }
//
//            rx_index = 0; // 歸零，準備接收下一包
//        }
//        // 重新開啟單字元中斷接收
//        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
//    }
//}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        // Size 是 DMA 實際收到的位元組數量 (例如收到 45 個字元，Size 就是 45)
        // 直接在陣列尾端補上 '\0'，這樣 sscanf 才知道字串到哪裡結束
        rx_buf[Size] = '\0';

        // 解析字串
        int parsed_count = sscanf((char*)rx_buf,
                                  "$%f,%f,%f,%f,%f,%f,%d",
                                  &joint[0].omega_ref, &joint[1].omega_ref, &joint[2].omega_ref,
                                  &joint[3].omega_ref, &joint[4].omega_ref, &joint[5].omega_ref,
                                  &gripper_status);

        if (parsed_count == 7) {
            Send_Status_To_Pi();          // 觸發 DMA 發送狀態回樹莓派
        }

        // ★ 重要：處理完畢後，必須重新啟動 DMA 來接收下一包資料
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, RX_BUF_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // 直接重新啟動接收中斷
//        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
        HAL_UART_DMAStop(&huart2);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, RX_BUF_SIZE);
    }
}
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
#ifdef USE_FULL_ASSERT
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
