/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
#define ADC_CH1 ADC_CHANNEL_12
#define ADC_CH2 ADC_CHANNEL_13
#define ADC_CH3 ADC_CHANNEL_14
#define ADC_CH4 ADC_CHANNEL_15


#define ADC_CHANNEL_REF 5
#define ADC_CHANNEL_TEMP 4
  extern uint16_t adc1ValBuf[ADC_CHANNEL_REF + 1];


/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */
float AdcGetChipTemperature();

float AdcGetVoltage(uint32_t _channel);

uint16_t AdcGetRaw(uint32_t _channel);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

