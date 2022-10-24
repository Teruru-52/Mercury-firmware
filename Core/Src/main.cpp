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
Agent::State prevState = Agent::State::IDLE;
State state;

hardware::LED led;
hardware::IRsensor irsensors(2300);
hardware::Speaker speaker;
undercarriage::Controller controller(0.001, 0.001);
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
Direction wallData;
IndexVec robotPos;

int cnt16kHz = 0;
int cnt1kHz = 0;

float bat_vol;
std::vector<uint32_t> ir_data{0, 0, 0, 0};

void Initialize()
{
  irsensors.UpdateFrontValue();
  if (!irsensors.StartInitialize())
  {
    speaker.Beep();
    controller.InitializeOdometory();
    speaker.Beep();
    state.interruption = State::INTERRUPT;
    // state.mode = State::FORWARD;
    state.mode = State::PIVOT_TURN180;
  }
}

void UpdateSensorData()
{
  bat_vol = irsensors.GetBatteryVoltage();
  controller.UpdateBatteryVoltage(bat_vol);
  irsensors.Update();
  ir_data = irsensors.GetIRSensorData();
  controller.UpdateOdometory();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (state.interruption == State::INTERRUPT)
  {
    if (htim == &htim1) // interruption 16kHz
    {
      irsensors.UpdateSideValue();
      irsensors.UpdateFrontValue();

      cnt16kHz = (cnt16kHz + 1) % 16;

      if (cnt16kHz == 0) // interruption 1kHz
      {
        cnt1kHz = (cnt1kHz + 1) % 1000;
        UpdateSensorData();

        // controller.robotMove(state.mode, ir_data);
        // if (controller.GetFlag())
        // {
        //   controller.Reset();
        //   if (state.mode == State::FORWARD)
        //   {
        //     controller.SetBase();
        //     state.mode = State::TURN_LEFT90;
        //   }
        //   else if (state.mode == State::TURN_LEFT90)
        //   {
        //     // static int cnt = 0;
        //     // if (cnt > 2)
        //     // {
        //     controller.Brake();
        //     state.mode = State::OUTPUT;
        //     state.interruption = State::NOT_INTERRUPT;
        //     // }
        //     // else
        //     // {
        //     //   cnt++;
        //     //   state.mode = State::FORWARD;
        //     // }
        //   }
        // }

        if (state.mode == State::PIVOT_TURN180)
        {
          controller.PivotTurn180();
          if (controller.GetFlag())
          {
            controller.Reset();
            static int cnt_pivot = 0;
            cnt_pivot++;
            // state.mode == State::WAIT;
            if (cnt_pivot >= 10)
            {
              controller.Brake();
              state.mode = State::OUTPUT;
              state.interruption = State::NOT_INTERRUPT;
            }
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
          // controller.OutputLog();
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
    if (state.mode == State::INITIALIZE)
    {
      Initialize();
    }

    else
    {
      controller.UpdateIMU();

      // wallData = controller.getWallData(ir_data);
      // robotPos = controller.getRobotPosition();
      //壁情報を更新 次に進むべき方向を計算
      // agent.update(robotPos, wallData);

      // FINISHEDになったら計測走行にうつる
      // if (agent.getState() == Agent::FINISHED)
      // {
      //ロボットを停止させ、スタートする向きに戻す
      // robotPositionInit();
      //最短経路の計算 割と時間がかかる(数秒)
      // agent.calcRunSequence(false);
      // const OperationList &runSequence = agent.getRunSequence();
      // for (size_t i = 0; i < runSequence.size(); i++)
      // {
      // Operationの実行が終わるまで待つ(nマス進んだ,右に曲がった)
      // while (!operationFinished())
      //   ;

      // i番目のを実行
      // robotMove(runSequence[i]); // robotMode関数はOperation型を受け取ってそれを実行する関数
      // }
      // }
      //ゴールにたどり着いた瞬間に一度だけmazeのバックアップをとる
      // if (prev_State == Agent::SEARCHING_NOT_GOAL && agent.getState() == SEARCHING_REACHED_GOAL)
      // {
      //   maze_backup = maze;
      // }
      // prev_State = agent.getState();

      // Agentの状態が探索中の場合は次に進むべき方向を取得する
      // Direction nextDir = agent.getNextDirection();
      // robotMove(nextDir);  //robotMove関数はDirection型を受け取ってロボットをそっちに動かす関数

      if (state.mode == State::OUTPUT)
      {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0)
        {
          controller.OutputLog();
          // step_identification.OutputLog();
        }
      }

      else if (state.mode == State::WAIT)
      {
        HAL_Delay(500);
        state.mode = State::PIVOT_TURN90;
        state.interruption = State::INTERRUPT;
      }
    }
  } // end while
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
