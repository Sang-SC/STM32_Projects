/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "my_can.h"
#include "my_motor.h"
#include "my_spdlog.h"

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

/* USER CODE BEGIN PV */

float AS5048_angle;
extern float motor_angle_float;

volatile uint8_t timer4_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

    CAN_Init(&hcan1);  // 初始化CAN总线
    CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(13) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0x141, 0x000);

    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);  // 使能定时器输入捕获通道中断，用于捕获 PWM 信号，计算频率和占空比

    // 电机归零位
    multiTurnPositionControl2(0, 300);
    while (motor_angle_float > 0.1 || motor_angle_float < -0.1) {
        readMultiTurnAngle();
        HAL_Delay(2);
    }
		
		HAL_TIM_Base_Start_IT(&htim4);   // 使能定时器 4 中断，用于位置控制

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        // Read_Angle_Test();
        // Square_Wave_Test(60, 800);
        Sine_Wave_Test(10, 1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim4)
	{
		timer4_flag = 1;
//		user_spdlog_debug("enter timer4");
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        // !一些 PWM 涉及的变量，下面 5 个 static 去掉的话，AS5048_angle 就会计算的很奇怪，不知道为什么
        static uint32_t total_count;         // 一个 PWM 周期内的总计数
        static uint32_t high_count;          // 一个 PWM 周期内的高电平计数
        static uint32_t low_count;           // 一个 PWM 周期内的低电平计数
        static float duty_circle;            // 占空比
        static float frequency;              // 频率
        static uint8_t risingEdgeFlag = 0;   // 上升沿标志位
        static uint8_t fallingEdgeFlag = 0;  // 下降沿标志位

        // 根据电平信号，获取计算 PWM 要用的数据
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 1) {                                             // 上升沿
            risingEdgeFlag = 1;                                                                      // 上升沿标志位置 1
            low_count = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);                            // 读取低电平计数值
            __HAL_TIM_SET_COUNTER(&htim2, 0);                                                        // 定时器 4 计数器清零
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);  // 设置下降沿捕获
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0) {                                            // 下降沿
            fallingEdgeFlag = 1;                                                                    // 下降沿标志位置 1
            high_count = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);                          // 读取高电平计数值
            __HAL_TIM_SET_COUNTER(&htim2, 0);                                                       // 定时器 4 计数器清零
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);  // 设置上升沿捕获
        }
        if (risingEdgeFlag == 1 && fallingEdgeFlag == 1) {
            // 一次 PWM 信号完成，计算相关数据
            total_count = high_count + low_count;  // 计算一个 PWM 周期内的总计数
            duty_circle = (float)high_count / (high_count + low_count);
            // duty_circle = (high_count * 1.0) / (total_count * 1.0);  // 加上这行竟然就卡了？
            risingEdgeFlag = 0;
            fallingEdgeFlag = 0;
            AS5048_angle = duty_circle * 360 - 92;

            // static float last_AS5048_angle = AS5048_angle;

            // 计数，用来输出一些信息
//            static uint32_t cnt = 0;
//            cnt++;
//            if (cnt % 1 == 0) {
//                //                user_spdlog_trace("total_count=%d", total_count);
//                //                user_spdlog_trace("high_count=%d", high_count);
//                //                user_spdlog_trace("duty_circle=%.3f", duty_circle);
//                //                user_spdlog_trace("frequency=%.3f", frequency);
//                // user_spdlog_trace("AS5048_angle=%.3f", AS5048_angle);
//                if (AS5048_angle > 1 || AS5048_angle < -1) {
//                    // user_spdlog_error("AS5048_angle=%.3f", AS5048_angle);
//                }
//            }
        }
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
    while (1) {
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
