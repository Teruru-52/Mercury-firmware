/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
  typedef struct _GPIO_Value
  {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_PIN_x;
  } GPIO_Value;

  extern GPIO_Value SIDE_RIGHT_LED;
  extern GPIO_Value FRONT_RIGHT_LED;
  extern GPIO_Value FRONT_LEFT_LED;
  extern GPIO_Value SIDE_LEFT_LED;
  extern GPIO_Value BACK_RIGHT_LED;
  extern GPIO_Value BACK_LEFT_LED;

  extern GPIO_Value USER_SW;
  extern GPIO_Value SPI_CS;
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
  GPIO_PinState Read_GPIO(GPIO_Value GPIO);
  void Toggle_GPIO(GPIO_Value GPIO);
  void Write_GPIO(GPIO_Value GPIO, GPIO_PinState PinState);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
