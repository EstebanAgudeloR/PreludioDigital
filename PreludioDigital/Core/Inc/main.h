/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef enum {
	ACTIVE_BUFFER_IS_PIN,
	ACTIVE_BUFFER_IS_PON
} ActiveBuffer_t;

typedef struct {
	uint8_t*  buffer; // Pointer to the buffer that is ready for processing
	uint16_t  size;   // The number of bytes in that buffer
} DataPacket;

typedef enum {
	CMD_ID_GET_INFO,
	CMD_ID_HELP,
	CMD_ID_UNKNOWN,
	CMD_ID_GET_CONFIG,
	CMD_ID_RESET,
	CMD_ID_SET_DIFFICULTY,
	CMD_ID_SET_TIME,
	CMD_ID_SET_ROUNDS,
	CMD_ID_SET_CONFIG,
	CMD_ID_START_FREE,
	CMD_ID_START_MEMORY,
	CMD_ID_PLAY_SONG
} CommandID_t;

typedef struct {
	const char* command_str;
	CommandID_t command_id;
} Command_t;

typedef struct {
    uint32_t sampling_rate;     // e.g. 44100, 48000...
    uint16_t fft_size;          // e.g. 1024, 2048
} AnalyzerConfig;
typedef enum {
    STATE_IDLE,
    STATE_PARSE_COMMAND,
    STATE_EXECUTE_COMMAND,
} SystemState_t;
typedef enum {
	GAME_WAIT_START,
	GAME_READY,
	GAME_NEW_PATTERN,
	GAME_WAIT_INPUT,
	GAME_EVALUATE,
	GAME_FEEDBACK,
	GAME_NEXT_ROUND,
	 GAME_WAIT_FINGERS,
	GAME_OVER,
	GAME_FREE_PLAY,
	GAME_MEMORY
} GameState_t;

typedef enum {
    MEMORY_INIT,
	MEMORY_WAIT_BEFORE_SHOW,
    MEMORY_SHOW_SEQUENCE,
    MEMORY_WAIT_INPUT,
    MEMORY_EVALUATE_INPUT,
    MEMORY_GAME_OVER
} MemoryGameState_t;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t estado_anterior;
    uint16_t arr;
    uint16_t ccr;
} BotonSonido_t;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define userLed_Pin GPIO_PIN_1
#define userLed_GPIO_Port GPIOH
#define Boton_2_Pin GPIO_PIN_2
#define Boton_2_GPIO_Port GPIOC
#define Boton_2_EXTI_IRQn EXTI2_IRQn
#define Boton_1_Pin GPIO_PIN_3
#define Boton_1_GPIO_Port GPIOC
#define Boton_1_EXTI_IRQn EXTI3_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Boton_3_Pin GPIO_PIN_4
#define Boton_3_GPIO_Port GPIOC
#define Boton_3_EXTI_IRQn EXTI4_IRQn
#define Led_2_Pin GPIO_PIN_5
#define Led_2_GPIO_Port GPIOC
#define Boton_5_Pin GPIO_PIN_10
#define Boton_5_GPIO_Port GPIOB
#define Boton_5_EXTI_IRQn EXTI15_10_IRQn
#define Led_3_Pin GPIO_PIN_12
#define Led_3_GPIO_Port GPIOB
#define Led_1_Pin GPIO_PIN_6
#define Led_1_GPIO_Port GPIOC
#define Led_4_Pin GPIO_PIN_11
#define Led_4_GPIO_Port GPIOA
#define Led_5_Pin GPIO_PIN_12
#define Led_5_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Boton_4_Pin GPIO_PIN_5
#define Boton_4_GPIO_Port GPIOB
#define Boton_4_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
