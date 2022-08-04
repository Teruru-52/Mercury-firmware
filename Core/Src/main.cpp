/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <list>
#include <vector>
#include <unistd.h>
#include <cmath>
#include "my_header.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Maze maze;
Maze maze_backup;
Agent agent(maze);

hardware::LED led;
hardware::IRsensor irsensors(2300);
hardware::Speaker speaker;
undercarriage::Odometory odom(0.001);
undercarriage::Controller controller(0.001);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char mode = 'i';

char initialization = 'i';
char slalom = 's';
char translation = 't';
char pivot_turn = 'p';
char stop_movement = 'x';
char control = 'c';
char output = 'o';
char wait = 'w';

int cnt16kHz = 0;
int cnt1kHz = 0;
int cnt_pivot = 0;

float bat_vol;
bool flag_interruption = false;
bool flag_odometory = true;

std::vector<float> cur_pos{0, 0, 0};
std::vector<float> cur_vel{0, 0};
std::vector<uint32_t> ir_data{0, 0, 0, 0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (flag_interruption)
  {
    if (htim == &htim1) // interruption 16kHz
    {
      irsensors.UpdateSideValue();
      irsensors.UpdateFrontValue();

      cnt16kHz = (cnt16kHz + 1) % 16;

      if (cnt16kHz == 0) // interruption 1kHz
      {
        cnt1kHz = (cnt1kHz + 1) % 1000;
        bat_vol = irsensors.GetBatteryVoltage();
        controller.UpdateBatteryVoltage(bat_vol);
        irsensors.Update();
        ir_data = irsensors.GetIRSensorData();
        odom.Update();
        cur_pos = odom.GetPosition();
        cur_vel = odom.GetVelocity();

        if (mode == translation)
        {
          controller.GoStraight(cur_pos, cur_vel, ir_data);
          static int cnt_trans = 0;
          cnt_trans++;
          // if (irsensors.GetFrontWallFlag())
          if (cnt_trans > 200)
          {
            mode = slalom;
          }
        }

        else if (mode == slalom)
        {
          if (flag_odometory)
          {
            odom.Reset();
            cur_pos = odom.GetPosition();
            cur_vel = odom.GetVelocity();
            flag_odometory = false;
          }

          if (controller.GetFlag())
          {
            controller.KanayamaTurnRight90(cur_pos, cur_vel);
          }
          else
          {
            flag_interruption = false;
            mode = output;
          }
        }

        else if (mode == pivot_turn)
        {
          // if (controller.GetFlag() && cnt_pivot < 10)
          if (controller.GetFlag())
          {
            controller.PivotTurn180(cur_pos, cur_vel);
          }
          else
          {
            flag_interruption = false;
            controller.ResetFlag();
            // mode = wait;
            // cnt_pivot++;

            mode = output;
          }
        }

        if (cnt1kHz == 0)
        {
          led.on_back_right();
        }
        else
          led.off_back_right();

        if (cnt1kHz % 200 == 0)
        {
          // printf("%f, %f\n", cur_pos[0], cur_pos[1]);
          // printf("%f, %f\n", cur_pos[2], cur_vel[1]);
          // printf("%f, %f\n", cur_vel[0], cur_vel[1]);
          // printf("%f\n", bat_vol);
          // printf("%lu, %lu,%lu, %lu\n", ir_data[0], ir_data[1], ir_data[2], ir_data[3]);
        }
      }
    }
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

  setbuf(stdout, NULL);
  speaker.Beep();
  irsensors.BatteryCheck();
  irsensors.on_all_led();
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (mode == initialization)
    {
      irsensors.UpdateFrontValue();
      if (!irsensors.StartInitialize())
      {
        speaker.Beep();
        odom.Initialize();
        speaker.Beep();
        flag_interruption = true;
        mode = translation;
      }
    }

    else
    {
      odom.IMU_Update();
      if (mode == output)
      {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0)
        {
          controller.OutputLog();
          // step_identification.OutputLog();
        }
      }
      else if (mode == wait)
      {
        HAL_Delay(500);
        mode = pivot_turn;
        flag_interruption = true;
      }
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

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
