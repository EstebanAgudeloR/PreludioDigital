/* USER CODE BEGIN Header */
/**
 ******************************************************************************
Desarrollado por : Esteban Agudelo Rincón - año 2025 - Taller V
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include  "stdio.h"
#include  "stdint.h"
#include  "string.h"
#include  <stdlib.h>
#include "arm_math.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_PIN_PON_SIZE 64
#define RECORD_HOLDER "ESTEBAN"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

SystemState_t system_state = STATE_IDLE;
GameState_t game_state = GAME_WAIT_START;


uint8_t Buffer_PIN[BUFFER_PIN_PON_SIZE];
uint8_t Buffer_PON[BUFFER_PIN_PON_SIZE];

volatile DataPacket data_ready_packet = { .buffer = NULL, .size = 0 };

volatile ActiveBuffer_t dma_active_buffer = ACTIVE_BUFFER_IS_PIN;


const Command_t command_table[] = {
		{ "get_info",       CMD_ID_GET_INFO },
		{ "help",           CMD_ID_HELP },
		{ "get_config",     CMD_ID_GET_CONFIG },
		{ "reset", CMD_ID_RESET },
		{ "set_difficulty", CMD_ID_SET_DIFFICULTY },
		{ "set_time",       CMD_ID_SET_TIME },
		{ "set_rounds",     CMD_ID_SET_ROUNDS },
		{ "set_config", CMD_ID_SET_CONFIG },
		{ "start_free", CMD_ID_START_FREE },
		{ "start_memory", CMD_ID_START_MEMORY },
		{ "play_song",      CMD_ID_PLAY_SONG }
};


BotonSonido_t botones_sonido[5] = {
		{ Boton_1_GPIO_Port, Boton_1_Pin, 0, 3823, 1911 }, // Do (C6)
		{ Boton_2_GPIO_Port, Boton_2_Pin, 0, 3405, 1702 }, // Re (D6)
		{ Boton_3_GPIO_Port, Boton_3_Pin, 0, 3033, 1516 }, // Mi (E6)
		{ Boton_4_GPIO_Port, Boton_4_Pin, 0, 2863, 1431 }, // Fa (F6)
		{ Boton_5_GPIO_Port, Boton_5_Pin, 0, 2551, 1275 }, // Sol (G6)
};

static MemoryGameState_t memory_state = MEMORY_INIT;
static uint8_t memory_sequence[50];     // secuencia de LEDs (valores de 0 a 4)
static uint8_t memory_sequence_length = 0;
static uint8_t memory_input_index = 0;
static uint32_t last_note_time = 0;
static uint8_t showing_index = 0;
static uint8_t showing_led_on = 0;

/* variables para el juego/entrenamiento*/
uint8_t current_round =0;
uint8_t total_rounds =0;
uint16_t tiempo_maximo_respuesta_ms = 0;
uint8_t dificultad_actual =0;
uint8_t aciertos =0;
uint8_t errores =0;
uint8_t patron_leds[5] ;
uint8_t respuesta_usuario[5] ;
uint8_t mensaje_inicio_mostrado = 0;
uint8_t mostrar_config_en_lcd = 0;
uint32_t tiempo_inicio_mostrar_config = 0;
uint8_t play_song_mostrar_lcd = 0;

const int num_commands = sizeof(command_table) / sizeof(command_table[0]);

const char* cancion_cumple[] = {
		"1 1  2  1  4 3",
		"1 1  2  1  5 4",
		"1 1 1  5  4  3 2",
		"5 5  4  1  2 1"
};

const char* cancion_oda[] = {
		"3 3 4 5 5 4 3 2",
		"1 1 2 3 3  2 2",
		"3 3 4 5 5 4 3 2",
		"1 1 2 3  2  1 1"
};
const char* cancion_hermano[] = {
		"1 2 3 1   1 2 3 1",
		"3 4 5     3 4 5",
		"5 5 5 5   5 4 4 4",
		"3 2 1     3 2 1"
};
const char* cancion_hotcross[] = {
		"3 2 1    3 2 1",
		"1 1 1 1  2 2 2 2",
		"3  2  1",
		"",
};
const char* cancion_estrellita[] = {
		"1 1 4 4 5 5 4",       // Estrellita, ¿dónde estás?
		"3 3 2 2 1 1",         // Me pregunto qué serás
		"4 4 3 3 2 2 1",       // En el cielo y en el mar
		"4 4 3 3 2 2 1"        // Una joya sin igual
};
const char* ejercicio_hanon1[] = {
		"1 2 3 4 5 4 3 2",   // Subida y bajada
		"1 2 3 4 5 4 3 2",
		"1 2 3 4 5 4 3 2",
		"1 2 3 4 5 4 3 2"
};
const char* ejercicio_hanon2[] = {
		"1 3 2 4 3 5",   // Salto cruzado
		"3 4 2 3 1",
		"1 3 2 4 3 5",   // Salto cruzado
		"3 4 2 3 1",
};
const char* ejercicio_hanon3[] = {
		"1 1 2 2 3 3 4 4",   // Doble golpe por dedo
		"5 5 4 4 3 3 2 2",
		"1 2 3 4 5 5 5 5",
		"4 4 3 3 2 2 1 1"
};

const char* ejercicio_hanon4[] = {
		"1 5 2 5 3 5 4 5",   // ida
		"4 5 3 5 2 5 1 ",   // vuelta
		"1 5 2 5 3 5 4 5",
		"4 5 3 5 2 5 1 ",
};
const char* ejercicio_hanon5[] = {
		"1 2 3 4 5 4 3 4 5",   // mano derecha típica
		"4 3 4 5",   // inverso simétrico
		"4 3 2 1 ",   // salto progresivo
		"REPITE!"    // rítmico en cruz
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void command_reset_handler(char* params);

void SystemFSM_Update(void);
void GameFSM_Update(void);


void SystemClock_Config(void);
void parse_and_execute_command(uint8_t* buffer, uint16_t size);
void dispatch_command(CommandID_t id, char* command, char* params);
/* prototipo de las funciones de los comandos*/
CommandID_t find_command_id(const char* command_str);
void command_help_handler(char* params);
void command_get_info_handler(char* params);
void command_unrecognized_handler(char* command);
void command_get_config_handler(char* params);
void command_set_config_handler(char* params);
void command_play_song_handler(char* params);
bool ConfiguracionLista();
void GameFSM_Update_FreePlaySonoro(void);
void GameFSM_Update_MemoryGame(void);
bool AreAllButtonsPressed(void);

void Buzzer_SuccessTone(void);
void Buzzer_ErrorTone(void);
void Buzzer_JingleWelcome(void);
void Buzzer_JingleGameOver(void);
void Buzzer_StartupPulse(void);
void ReproducirEjercicioHanon1(void);
void ReproducirEjercicioHanon2(void);
void ReproducirEjercicioHanon3(void);
void ReproducirEjercicioHanon4(void);
void ReproducirEjercicioHanon5(void);
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
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM10_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2); //Blinky

	//	HAL_TIM_Base_Start(&htim3);  //Trigger source for ADC

	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_Base_Start(&htim3);  // Reiniciar TIM3 con los nuevos valores

	char* msg1 =
			"\r\n"
			"===========================================\r\n"
			"         BIENVENIDO A PRELUDIO DIGITAL     \r\n"
			"===========================================\r\n"
			" Entrenador de independencia de dedos (STM32)\r\n"
			" para pianistas y músicos.\r\n"
			"\r\n"
			" Antes de iniciar, configure la rutina usando:\r\n"
			"   set_config D1 T2000 R10\r\n"
			"\r\n"
			" D1 → dificultad (1, 2, 3, X)\r\n"
			" 2000 → tiempo máximo de respuesta (ms)\r\n"
			" R10 → número de rondas (5, 10, 15, 20)\r\n"
			"\r\n"
			" Luego, coloca los 5 dedos sobre los botones.\r\n"
			" El juego comenzará automáticamente.\r\n"
			" MODO LIBRE:  Usa el comando 'start_free'\r\n"
			" Escucha y siente los sonidos al presionar.\r\n"
			"\r\n"
			" Entrenamiento de memoria: Usa el comando 'start_memory'\r\n"
			" Repite secuencias de luces y sonidos.\r\n"
			" Ideal para trabajar concentración y memoria.\r\n"
			"play_song para tocar o hacer entrenamientos de HANON"
			"\r\n"
			" Escriba 'help' para ver los comandos disponibles.\r\n"
			"\r\n";
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg1, strlen(msg1));

	LiquidCrystal_I2C_Init(0x21, 20, 4);

	// Enciende la luz de fondo del LCD
	LiquidCrystal_I2C_Backlight();

	// Limpia completamente la pantalla del LCD
	LiquidCrystal_I2C_Clear();

	LiquidCrystal_I2C_Clear();
	LiquidCrystal_I2C_SetCursor(0, 0);
	LiquidCrystal_I2C_Print(" PRELUDIO DIGITAL ");
	LiquidCrystal_I2C_SetCursor(0, 1);
	LiquidCrystal_I2C_Print(" STM32 Entrenador ");
	LiquidCrystal_I2C_SetCursor(0, 2);
	LiquidCrystal_I2C_Print(" Independencia de ");
	LiquidCrystal_I2C_SetCursor(0, 3);
	LiquidCrystal_I2C_Print("    dedos (Piano) ");

	HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_SET);


	Buzzer_JingleWelcome();
	HAL_Delay(2500);
	Buzzer_SuccessTone();
	LiquidCrystal_I2C_Clear();
	LiquidCrystal_I2C_SetCursor(0, 0);
	LiquidCrystal_I2C_Print("Configura por UART:");
	LiquidCrystal_I2C_SetCursor(0, 1);
	LiquidCrystal_I2C_Print("Dificultad, Tiempo");
	LiquidCrystal_I2C_SetCursor(0, 2);
	LiquidCrystal_I2C_Print("y # Rondas");
	LiquidCrystal_I2C_SetCursor(0, 3);
	LiquidCrystal_I2C_Print("Ej: D1 T2000 R10");
	HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_RESET);
	HAL_Delay(2500);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */





	// A variable to keep track of which buffer the DMA is currently filling

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Buffer_PIN, BUFFER_PIN_PON_SIZE);


	while (1)
	{

		SystemFSM_Update();
		GameFSM_Update();

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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 16000-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 250-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

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
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 3;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 3824;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void)
{

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 0;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 65535;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */

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
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(userLed_GPIO_Port, userLed_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin|Led_4_Pin|Led_5_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, Led_2_Pin|Led_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : userLed_Pin */
	GPIO_InitStruct.Pin = userLed_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(userLed_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : Boton_2_Pin Boton_1_Pin Boton_3_Pin */
	GPIO_InitStruct.Pin = Boton_2_Pin|Boton_1_Pin|Boton_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin Led_4_Pin Led_5_Pin */
	GPIO_InitStruct.Pin = LD2_Pin|Led_4_Pin|Led_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Led_2_Pin Led_1_Pin */
	GPIO_InitStruct.Pin = Led_2_Pin|Led_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : Boton_5_Pin Boton_4_Pin */
	GPIO_InitStruct.Pin = Boton_5_Pin|Boton_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : Led_3_Pin */
	GPIO_InitStruct.Pin = Led_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Led_3_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
//	adcValue= HAL_ADC_GetValue(&hadc1);  //Read and update the ADC result
//	HAL_GPIO_TogglePin(adcTest_GPIO_Port , adcTest_Pin); //Toogle interrupt rate indicador pin
//}
//



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		if (dma_active_buffer == ACTIVE_BUFFER_IS_PIN)
		{
			data_ready_packet.buffer = Buffer_PIN;
			data_ready_packet.size = Size;
			dma_active_buffer = ACTIVE_BUFFER_IS_PON;
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Buffer_PON, BUFFER_PIN_PON_SIZE);
		}
		else // dma_active_buffer == ACTIVE_BUFFER_IS_PON
		{
			data_ready_packet.buffer = Buffer_PON;
			data_ready_packet.size = Size;
			dma_active_buffer = ACTIVE_BUFFER_IS_PIN;
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Buffer_PIN, BUFFER_PIN_PON_SIZE);
		}
	}
}

void parse_and_execute_command(uint8_t* buffer, uint16_t size)
{
	if (size == 0 || buffer == NULL) {
		return; // Nothing to process
	}

	// 1️ Ensure buffer is null-terminated
	if (size >= BUFFER_PIN_PON_SIZE) {
		buffer[BUFFER_PIN_PON_SIZE - 1] = '\0';
	} else {
		buffer[size] = '\0';
	}

	// 2️ Trim leading spaces/newlines
	char* start_ptr = (char*)buffer;
	while (*start_ptr == ' ' || *start_ptr == '\r' || *start_ptr == '\n' || *start_ptr == '\t') {
		start_ptr++;
	}

	if (*start_ptr == '\0') {
		// Only spaces / empty line — ignore
		return;
	}

	// 3️ Extract command
	char* command_str = strtok(start_ptr, " \r\n");
	char* params = strtok(NULL, "");

	if (command_str == NULL || strlen(command_str) == 0) {
		return; // Nothing valid
	}

	// 4️ Sanity check: Max length for command string (prevent garbage)
	if (strlen(command_str) > 32) {
		char* msg = "ERROR: Comando demasiado largo o invalido.\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		return;
	}

	// 5️ Find command id
	CommandID_t id = find_command_id(command_str);

	// 6️ Dispatch
	dispatch_command(id, command_str, params);
}



CommandID_t find_command_id(const char* command_str)
{
	for (int i = 0; i < num_commands; i++) {
		if (strcmp(command_str, command_table[i].command_str) == 0) {
			return command_table[i].command_id;
		}
	}
	return CMD_ID_UNKNOWN;
}



void dispatch_command(CommandID_t id, char* command, char* params)
{
	switch (id)
	{


	case CMD_ID_HELP:
		command_help_handler(params);
		break;

	case CMD_ID_GET_INFO:
		command_get_info_handler(params);
		break;

	case CMD_ID_GET_CONFIG:
		command_get_config_handler(params);
		break;
	case CMD_ID_SET_ROUNDS:
		command_set_config_handler(params);
		break;
	case CMD_ID_START_FREE:
		game_state = GAME_FREE_PLAY;
		HAL_UART_Transmit(&huart2, (uint8_t*)"Modo libre activado.\r\n", 24, HAL_MAX_DELAY);
		break;
	case CMD_ID_START_MEMORY:
		game_state = GAME_MEMORY;
		HAL_UART_Transmit(&huart2, (uint8_t*)"Juego de memoria iniciado.\r\n", 29, HAL_MAX_DELAY);
		break;
	case CMD_ID_SET_CONFIG:
		command_set_config_handler(params);
		break;
	case CMD_ID_PLAY_SONG:
		command_play_song_handler(params);
		break;
	case CMD_ID_RESET:
		command_reset_handler(params);
		break;

	case CMD_ID_UNKNOWN:
	default:
		command_unrecognized_handler(command);
		break;

	}
}


void command_get_info_handler(char* params)
{
	(void)params; // Not used

	const char* msg =
			"\r\n=========== PRELUDIO DIGITAL ===========\r\n"
			" Proyecto     : Entrenador digital de dedos para pianistas\r\n"
			" Autor        : Esteban Agudelo Rincón\r\n"
			" MCU          : STM32F411RE (Nucleo-F411RE)\r\n"
			" Firmware Ver : v1.0\r\n"
			" Fecha Build  : 23/07/2025\r\n"
			"\r\n"
			" Descripción:\r\n"
			"  Sistema interactivo basado en LEDs y botones para\r\n"
			"  entrenar la independencia, precisión y coordinación\r\n"
			"  de los dedos, especialmente útil para pianistas.\r\n"
			"\r\n"
			" Características:\r\n"
			"  - 5 botones (uno por dedo) y 5 LEDs.\r\n"
			"  - Máquina de estados robusta para control del juego.\r\n"
			"  - Selección por UART de dificultad, tiempo y rondas.\r\n"
			"  - Retroalimentación sonora y visual (LCD, buzzer).\r\n"
			"  - Niveles: D1 (fácil) a DX (muy difícil)\r\n"
			"  - Tiempos: desde 10000ms (fácil) hasta 100ms (DIOS).\r\n"
			"  - Incluye modo libre ('start_free') para práctica sin reglas.\r\n"
			"    → Ideal también con fines terapéuticos o motrices.\r\n"
			"  - Incluye ('start_memory') para practicar la memoria, habilidad fundamental en piano.\r\n"
		    "  - Incluye ('play_song') para música y ejercicios de HANON\r\n"
			"\r\n"
			" Contexto:\r\n"
			"  Diseñado como herramienta de estudio musical sin\r\n"
			"  instrumento. Basado en ejercicios clásicos usados\r\n"
			"  por docentes para entrenar la destreza manual.\r\n"
			" El dispositivo también funciona tipo juego interactivo"
			"\r\n"
			"========================================\r\n";

	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void command_unrecognized_handler(char* command)
{
	char msg[128];
	sprintf(msg, "ERROR: Comando no reconocido: '%s'\r\nEscriba 'help' para ver la lista de comandos.\r\n", command);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, strlen(msg));
}
void command_help_handler(char* params)
{
	(void)params; // Unused parameter
	char* help_msg =
			"\r\n========== COMANDOS DISPONIBLES ==========\r\n"
			"\r\n"
			" help\r\n"
			"   Muestra este menú de ayuda.\r\n"
			"\r\n"
			" set_config <D> <T> <R>\r\n"
			"   Configura todo el juego en un solo paso.\r\n"
			"   Ejemplo: set_config D2 T3000 R15\r\n"
			"\r\n"
			"   D<valor> → dificultad de dedos:\r\n"
			"     D1  = 1 dedo (fácil)\r\n"
			"     D2  = 4 dedos (medio)\r\n"
			"     D3  = 2 y 3 dedos (difícil)\r\n"
			"     DX  = aleatorio  (Muy difícil)\r\n"
			"\r\n"
			"   T<valor> → tiempo máximo de respuesta (ms):\r\n"
			"     T10000 = Muy fácil\r\n"
			"     T4000  = Fácil\r\n"
			"     T3000  = Moderado\r\n"
			"     T2000  = Difícil\r\n"
			"     T1000  = Muy difícil\r\n"
			"     T500   = Profesional\r\n"
			"     T400   = Legendario \r\n"
			"     T300   = IA\r\n"
			"     T200   = Robot\r\n"
			"     T100   = DIOS\r\n"
			"\r\n"
			"   R<valor> → número de rondas:\r\n"
			"     R5, R10, R15, R20\r\n"

			"\r\n"
			" get_config\r\n"
			"   Muestra la configuración actual del juego.\r\n"
			"\r\n"
			" get_info\r\n"
			"   Muestra información del sistema STM32.\r\n"
			"\r\n"
			" start_free\r\n"
			"   Inicia el modo libre. Presione los botones para ver respuesta.\r\n"
			"   Mantén los 5 botones para salir.\r\n"
			" start_memory\r\n"
			"   Inicia el minijuego de memoria visual y auditiva.\r\n"
			"   Reproduce la secuencia de luces y sonidos correctamente.\r\n"
			"   Mantén los 5 botones para salir del juego.\r\n"
			"\r\n"
			"play_song <nombre>: Muestra canción para tocar en modo libre\r\n"
			"  Canciones: cumple, oda, hermano, hotcross, estrella\r\n"
			"  Ejercicios: hanon1, hanon2, hanon3, hanon4, hanon5\r\n"
			" reset\r\n"
			"   Reinicia el sistema.\r\n"
			"\r\n"
			"===========================================\r\n";

	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)help_msg, strlen(help_msg));
}

void command_get_config_handler(char* params)
{
	(void)params; // Not used

	const char* nivel_dificultad_str;
	switch (dificultad_actual) {
	case 1: nivel_dificultad_str = "1 dedo (Fácil)"; break;
	case 2: nivel_dificultad_str = "4 dedos (Medio)"; break;
	case 3: nivel_dificultad_str = "2-3 dedos (Difícil)"; break;
	default: nivel_dificultad_str = "Aleatorio (Muy Difícil)"; break;
	}

	const char* tiempo_str;
	switch (tiempo_maximo_respuesta_ms) {
	case 10000: tiempo_str = "10000 ms (Muy fácil)"; break;
	case 5000:  tiempo_str = "5000 ms (Fácil)"; break;
	case 3000:  tiempo_str = "3000 ms (Moderado)"; break;
	case 2000:  tiempo_str = "2000 ms (Difícil)"; break;
	case 1000:  tiempo_str = "1000 ms (Muy difícil)"; break;
	case 500:   tiempo_str = "500 ms (Profesional)"; break;
	case 400:   tiempo_str = "400 ms (Legendario)"; break;
	case 300:   tiempo_str = "300 ms (IA)"; break;
	case 200:   tiempo_str = "200 ms (Robot)"; break;
	case 100:   tiempo_str = "100 ms (DIOS)"; break;
	default:    tiempo_str = "Desconocido"; break;
	}

	char msg[256];
	sprintf(msg,
			"\r\n=== CONFIGURACIÓN ACTUAL ===\r\n"
			" Dificultad de dedos : %s\r\n"
			" Tiempo de respuesta : %s\r\n"
			" Número de rondas    : %d\r\n"
			"=============================\r\n",
			nivel_dificultad_str,
			tiempo_str,
			total_rounds
	);

	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}
void SystemFSM_Update(void)
{
	static uint8_t* proc_buffer = NULL;
	static uint16_t proc_size = 0;

	switch (system_state)
	{
	case STATE_IDLE:
		if (data_ready_packet.size > 0)
		{
			system_state = STATE_PARSE_COMMAND;
		}
		break;

	case STATE_PARSE_COMMAND:
		proc_buffer = data_ready_packet.buffer;
		proc_size = data_ready_packet.size;

		data_ready_packet.buffer = NULL;
		data_ready_packet.size = 0;

		system_state = STATE_EXECUTE_COMMAND;
		break;

	case STATE_EXECUTE_COMMAND:
		parse_and_execute_command(proc_buffer, proc_size);
		system_state = STATE_IDLE;
		break;

	default:
		system_state = STATE_IDLE;
		break;
	}
}



void command_reset_handler(char* params)
{
	(void)params; // Not used

	char* msg = "INFO: Reiniciando el sistema...\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	HAL_Delay(100);  // Wait a bit for UART to finish

	NVIC_SystemReset();  // Perform the reset
}
void Buzzer_SuccessTone(void)
{
	// Tono agudo 1 (~6.6 kHz)
	__HAL_TIM_SET_AUTORELOAD(&htim3, 1538);  // 400 * 4MHz/615kHz ≈ 1538
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 769);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_Delay(80);

	// Tono agudo 2 (~8.4 kHz)
	__HAL_TIM_SET_AUTORELOAD(&htim3, 1153);  // 300 * factor
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 576);
	HAL_Delay(80);

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}
void Buzzer_ErrorTone(void)
{
	__HAL_TIM_SET_AUTORELOAD(&htim3, 6154);  // 3200 * factor
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 3077);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_Delay(350);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}
void Buzzer_JingleWelcome(void)
{
	struct {
		uint16_t arr;
		uint16_t ccr;
		uint16_t duration_ms;
	} notas[] = {
			{ 3077, 1538, 120 },  // Do''
			{ 2746, 1373, 120 },  // Re''
			{ 2168, 1084, 140 },  // Sol''
			{ 1615,  807, 280 },  // Do'''
			{ 2168, 1084, 80  },  // Eco: Sol''
			{ 3077, 1538, 100 }   // Eco: Do''
	};

	for (int i = 0; i < 6; i++) {
		__HAL_TIM_SET_AUTORELOAD(&htim3, notas[i].arr);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, notas[i].ccr);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		HAL_Delay(notas[i].duration_ms);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		HAL_Delay(i == 3 ? 100 : 50);
	}
}
void Buzzer_JingleGameOver(void)
{
	struct {
		uint16_t arr;
		uint16_t ccr;
		uint16_t duration_ms;
	} notas[] = {
			{ 3900, 1950, 120 },  // C  (600 * 6.5)
			{ 3250, 1625, 120 },  // D
			{ 2600, 1300, 120 },  // E
			{ 1950,  975, 100 },  // G (rápido)
			{ 4550, 2275, 100 },  // Bb (cambio agudo)
			{ 2925, 1462, 100 },  // F#
			{ 2470, 1235, 100 },  // A
			{ 1625,  812, 150 },  // HIGH C
			{ 3900, 1950, 80  },  // C
			{ 2470, 1235, 80  },  // A
			{ 1625,  812, 250 }   // HIGH C
	};

	for (int i = 0; i < sizeof(notas) / sizeof(notas[0]); i++) {
		__HAL_TIM_SET_AUTORELOAD(&htim3, notas[i].arr);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, notas[i].ccr);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		HAL_Delay(notas[i].duration_ms);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		HAL_Delay(50);
	}
}
void Buzzer_StartupPulse(void)
{
	// Tono medio (~3.125 kHz con nuevo prescaler)
	__HAL_TIM_SET_AUTORELOAD(&htim3, 1300);  // 1600 * 6.5 ≈ 10400 → corregido a tono medio
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 650);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_Delay(100);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}
void ReproducirCancionCumple(void)
{
	// Melodía simplificada: DO DO RE DO FA MI
	struct {
		uint16_t arr;
		uint16_t ccr;
		uint16_t duracion_ms;
	} notas[] = {
			{ 3823, 1911, 200 }, // DO (C6)
			{ 3823, 1911, 200 }, // DO
			{ 3405, 1702, 250 }, // RE (D6)
			{ 3823, 1911, 250 }, // DO
			{ 2863, 1431, 250 }, // FA (F6)
			{ 2551, 1275, 350 }, // MI (E6)
	};

	for (int i = 0; i < sizeof(notas) / sizeof(notas[0]); i++) {
		__HAL_TIM_SET_AUTORELOAD(&htim3, notas[i].arr);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, notas[i].ccr);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		HAL_Delay(notas[i].duracion_ms);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		HAL_Delay(50); // breve pausa entre notas
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){ //BLINKY cada 250 ms
		HAL_GPIO_TogglePin(userLed_GPIO_Port, userLed_Pin);
	}
}

void command_set_config_handler(char* params)
{
	if (params == NULL) {
		const char* err = "ERROR: Formato invalido. Ej: set_config D1 T2000 R10\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
		return;
	}

	// Verificar si ya existía una configuración previa válida
	bool ya_configurado = ConfiguracionLista();

	// Copiar para no modificar el buffer original
	char buffer[64];
	strncpy(buffer, params, sizeof(buffer));
	buffer[sizeof(buffer) - 1] = '\0';

	// Separar tokens
	char* token_d = strtok(buffer, " ");
	char* token_t = strtok(NULL, " ");
	char* token_r = strtok(NULL, " ");

	if (!token_d || !token_t || !token_r) {
		const char* err = "ERROR: Parametros incompletos. Use: D1 T2000 R10\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
		return;
	}

	// Procesar dificultad
	if (strlen(token_d) != 2 || token_d[0] != 'D') {
		const char* err = "ERROR: Dificultad invalida. Use D1, D2, D3 o DX\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
		return;
	}

	char d = token_d[1];
	if (d == '1' || d == '2' || d == '3') {
		dificultad_actual = d - '0';
	} else if (d == 'X' || d == 'x') {
		dificultad_actual = 'X';
	} else {
		const char* err = "ERROR: Nivel invalido. Use D1, D2, D3 o DX\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
		return;
	}

	// Procesar tiempo
	if (strlen(token_t) < 2 || token_t[0] != 'T') {
		const char* err = "ERROR: Tiempo invalido. Use T1000, T2000, etc.\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
		return;
	}
	int t = atoi(&token_t[1]);
	if (t == 500 || t == 300 || t == 200 || t == 400 || t == 100 ||t == 800 ||   t == 1000 || t == 2000 || t == 3000 || t == 4000 || t == 10000) {
		tiempo_maximo_respuesta_ms = t;
	} else {
		const char* err = "ERROR: Tiempo no valido. Use 100, 200, 300, 500, 1000, 2000, 3000, 4000 o 10000\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
		return;
	}

	// Procesar rondas
	if (strlen(token_r) < 2 || token_r[0] != 'R') {
		const char* err = "ERROR: Rondas invalidas. Use R5, R10, etc.\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
		return;
	}
	int r = atoi(&token_r[1]);
	if (r == 5 || r == 10 || r == 15 || r == 20) {
		total_rounds = r;
	} else {
		const char* err = "ERROR: Rondas no validas. Use 5, 10, 15 o 20\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
		return;
	}

	// Confirmación por UART
	char msg[128];
	snprintf(msg, sizeof(msg),
			"Configuracion lista:\r\nDificultad: %c\r\nTiempo: %d ms\r\nRondas: %d\r\n",
			(d == 'x') ? 'X' : d, tiempo_maximo_respuesta_ms, total_rounds);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	// Si ya estaba configurado antes, muestra mensaje especial en LCD
	if (ya_configurado)
	{
		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 1);
		LiquidCrystal_I2C_Print("Cambiando config...");
		HAL_Delay(1500);
	}

	// Forzar a que vuelva a mostrarse el mensaje de "Config OK"
	mensaje_inicio_mostrado = 0;
	mostrar_config_en_lcd = 1;
	tiempo_inicio_mostrar_config = HAL_GetTick();
}
bool ConfiguracionLista(void)
{
	return (
			(dificultad_actual == 1 || dificultad_actual == 2 || dificultad_actual == 3 || dificultad_actual == 'X') &&
			(tiempo_maximo_respuesta_ms == 500 || tiempo_maximo_respuesta_ms == 800 ||tiempo_maximo_respuesta_ms == 1000 ||
					tiempo_maximo_respuesta_ms == 2000 || tiempo_maximo_respuesta_ms == 3000 ||
					tiempo_maximo_respuesta_ms == 4000 || tiempo_maximo_respuesta_ms == 10000 || tiempo_maximo_respuesta_ms == 400 || tiempo_maximo_respuesta_ms == 300 || tiempo_maximo_respuesta_ms == 200 || tiempo_maximo_respuesta_ms == 100) &&
					(total_rounds == 5 || total_rounds == 10 || total_rounds == 15 || total_rounds == 20)
	);
}
void command_play_song_handler(char* params)
{
	Buzzer_SuccessTone();
	if (params == NULL) {
		const char* msg = "ERROR: Debe especificar el nombre de la canción.\r\nEjemplo: play_song cumple\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		return;
	}

	if (strncmp(params, "cumple", 6) == 0) {

		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print("Feliz Cumple! ");
		HAL_UART_Transmit(&huart2, (uint8_t*)"Reproduciendo: Cumpleaños feliz\r\n", 34, HAL_MAX_DELAY);

		HAL_Delay(2000);

		LiquidCrystal_I2C_Clear();
		for (int i = 0; i < 4; i++) {

			LiquidCrystal_I2C_SetCursor(0, i);
			LiquidCrystal_I2C_Print(cancion_cumple[i]);
			HAL_Delay(500);
		}
	}
	else if (strncmp(params, "oda", 3) == 0) {

		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print(" Oda a la Alegria ");
		HAL_UART_Transmit(&huart2, (uint8_t*)"Reproduciendo: Oda a la Alegría\r\n", 34, HAL_MAX_DELAY);

		HAL_Delay(2000);

		LiquidCrystal_I2C_Clear();
		for (int i = 0; i < 4; i++) {

			LiquidCrystal_I2C_SetCursor(0, i);
			LiquidCrystal_I2C_Print(cancion_oda[i]);
			HAL_Delay(500);
		}
	}
	else if (strncmp(params, "hermano", 7) == 0) {

		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print("Hermano Juan");
		HAL_UART_Transmit(&huart2, (uint8_t*)"Reproduciendo: Hermano Juan\r\n", 30, HAL_MAX_DELAY);

		HAL_Delay(2000);  // Espera para mostrar título
		LiquidCrystal_I2C_Clear();

		for (int i = 0; i < 4; i++) {

			LiquidCrystal_I2C_SetCursor(0, i);
			LiquidCrystal_I2C_Print(cancion_hermano[i]);
			HAL_Delay(500);
		}

		play_song_mostrar_lcd = 1;
		game_state = GAME_FREE_PLAY;
	}
	else if (strncmp(params, "hotcross", 8) == 0) {

		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print(" Hot Cross Buns");
		HAL_UART_Transmit(&huart2, (uint8_t*)"Reproduciendo: Hot Cross Buns\r\n", 33, HAL_MAX_DELAY);

		HAL_Delay(2000);  // Mostrar título
		LiquidCrystal_I2C_Clear();

		for (int i = 0; i < 4; i++) {

			LiquidCrystal_I2C_SetCursor(0, i);
			LiquidCrystal_I2C_Print(cancion_hotcross[i]);
			HAL_Delay(500);
		}

		play_song_mostrar_lcd = 1;
		game_state = GAME_FREE_PLAY;
	}
	else if (strncmp(params, "estrella", 8) == 0) {

		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print(" Estrellita ");
		HAL_UART_Transmit(&huart2, (uint8_t*)"Reproduciendo: Estrellita\r\n", 28, HAL_MAX_DELAY);

		HAL_Delay(2000);  // Espera para mostrar el título
		LiquidCrystal_I2C_Clear();

		for (int i = 0; i < 4; i++) {

			LiquidCrystal_I2C_SetCursor(0, i);
			LiquidCrystal_I2C_Print(cancion_estrellita[i]);
			HAL_Delay(500);
		}

		play_song_mostrar_lcd = 1;
		game_state = GAME_FREE_PLAY;
	}
	else if (strncmp(params, "hanon1", 6) == 0) {

		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print("Ejercicio: Hanon 1");
		HAL_UART_Transmit(&huart2, (uint8_t*)"Reproduciendo: Hanon 1\r\n", 25, HAL_MAX_DELAY);
		HAL_Delay(1000);  // Mostrar título antes de reproducir

		ReproducirEjercicioHanon1();

		HAL_Delay(500);
		LiquidCrystal_I2C_Clear();
		for (int i = 0; i < 4; i++) {
			LiquidCrystal_I2C_SetCursor(0, i);
			LiquidCrystal_I2C_Print(ejercicio_hanon1[i]);
			HAL_Delay(500);
		}
		play_song_mostrar_lcd = 1;
		game_state = GAME_FREE_PLAY;
	}

	else if (strncmp(params, "hanon2", 6) == 0) {

		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print("Ejercicio: Hanon 2");
		HAL_UART_Transmit(&huart2, (uint8_t*)"Reproduciendo: Hanon 2\r\n", 25, HAL_MAX_DELAY);
		HAL_Delay(1000);
		ReproducirEjercicioHanon2();
		HAL_Delay(500);
		LiquidCrystal_I2C_Clear();
		for (int i = 0; i < 4; i++) {
			LiquidCrystal_I2C_SetCursor(0, i);
			LiquidCrystal_I2C_Print(ejercicio_hanon2[i]);
			HAL_Delay(500);
		}
		play_song_mostrar_lcd = 1;
		game_state = GAME_FREE_PLAY;
	}

	else if (strncmp(params, "hanon3", 6) == 0) {

		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print("Ejercicio: Hanon 3");
		HAL_UART_Transmit(&huart2, (uint8_t*)"Reproduciendo: Hanon 3\r\n", 25, HAL_MAX_DELAY);
		HAL_Delay(1000);
		ReproducirEjercicioHanon3();
		HAL_Delay(500);
		LiquidCrystal_I2C_Clear();
		for (int i = 0; i < 4; i++) {
			LiquidCrystal_I2C_SetCursor(0, i);
			LiquidCrystal_I2C_Print(ejercicio_hanon3[i]);
			HAL_Delay(500);
		}
		play_song_mostrar_lcd = 1;
		game_state = GAME_FREE_PLAY;
	}
	else if (strncmp(params, "hanon4", 6) == 0) {

		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print("Ejercicio: Hanon 4");
		HAL_UART_Transmit(&huart2, (uint8_t*)"Reproduciendo: Hanon 4\r\n", 25, HAL_MAX_DELAY);
		HAL_Delay(1000);
		ReproducirEjercicioHanon4();
		HAL_Delay(500);
		LiquidCrystal_I2C_Clear();
		for (int i = 0; i < 4; i++) {
			LiquidCrystal_I2C_SetCursor(0, i);
			LiquidCrystal_I2C_Print(ejercicio_hanon4[i]);
			HAL_Delay(500);
		}
		play_song_mostrar_lcd = 1;
		game_state = GAME_FREE_PLAY;
	}
	else if (strncmp(params, "hanon5", 6) == 0) {

		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print("Ejercicio: Hanon 5");
		HAL_UART_Transmit(&huart2, (uint8_t*)"Reproduciendo: Hanon 5\r\n", 25, HAL_MAX_DELAY);
		HAL_Delay(1000);
			ReproducirEjercicioHanon5();
			HAL_Delay(500);
		LiquidCrystal_I2C_Clear();
		for (int i = 0; i < 4; i++) {
			LiquidCrystal_I2C_SetCursor(0, i);
			LiquidCrystal_I2C_Print(ejercicio_hanon5[i]);
			HAL_Delay(500);
		}
		play_song_mostrar_lcd = 1;
		game_state = GAME_FREE_PLAY;
	}
	else {
		const char* msg = "ERROR: Canción no reconocida. Use 'cumple' o 'oda'.\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		return;
	}

	// Luego de mostrar la canción, entrar en modo libre
	play_song_mostrar_lcd = 1;  //evita que modo libre sobreescriba pantalla
	game_state = GAME_FREE_PLAY;
}

void GameFSM_Update(void)
{
	static uint32_t last_update_time = 0;
	static uint8_t led_index = 0;
	static int8_t direction = 1; // 1: derecha, -1: izquierda
	const uint32_t led_delay_ms = 100;

	switch (game_state)
	{
	case GAME_WAIT_START:
	{
		// Solo entra al patrón de luces si la configuración ya fue establecida
		if (!ConfiguracionLista())
		{
			static uint8_t lcd_mostrado = 0;
			static uint8_t led_idx = 0;
			static int8_t led_dir = 1;
			static uint32_t last_led_time = 0;

			// Mostrar mensaje solo una vez
			if (!lcd_mostrado)
			{
				LiquidCrystal_I2C_Clear();
				LiquidCrystal_I2C_SetCursor(0, 0);
				LiquidCrystal_I2C_Print("Configura por UART:");
				LiquidCrystal_I2C_SetCursor(0, 1);
				LiquidCrystal_I2C_Print("Dificultad, Tiempo");
				LiquidCrystal_I2C_SetCursor(0, 2);
				LiquidCrystal_I2C_Print("y # Rondas");
				LiquidCrystal_I2C_SetCursor(0, 3);
				LiquidCrystal_I2C_Print("Ej: D1 T2000 R10");

				// Encendido inicial completo
				HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_SET);
				Buzzer_StartupPulse();
				HAL_Delay(1000);

				lcd_mostrado = 1;
				last_led_time = HAL_GetTick();
				led_idx = 0;
				led_dir = 1;
			}

			// Patrón dinámico mientras se espera la configuración
			if (HAL_GetTick() - last_led_time >= 700)  // delay suave
			{
				last_led_time = HAL_GetTick();

				// Apagar todos
				HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_RESET);

				// Encender solo uno
				switch (led_idx)
				{
				case 0: HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_SET); break;
				case 1: HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_SET); break;
				case 2: HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_SET); break;
				case 3: HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_SET); break;
				case 4: HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_SET); break;
				}

				led_idx += led_dir;
				if (led_idx == 4 || led_idx == 0)
					led_dir *= -1;
			}


			return;  // aún no continúa al resto del estado
		}

		// Ya está configurado: muestra mensaje de inicio
		if (!mensaje_inicio_mostrado)
		{
			Buzzer_SuccessTone();
			LiquidCrystal_I2C_Clear();
			LiquidCrystal_I2C_SetCursor(0, 0);
			LiquidCrystal_I2C_Print("Config OK!");
			LiquidCrystal_I2C_SetCursor(0, 1);
			LiquidCrystal_I2C_Print("Oprime los 5");
			LiquidCrystal_I2C_SetCursor(0, 2);
			LiquidCrystal_I2C_Print("botones para iniciar");
			LiquidCrystal_I2C_SetCursor(0, 3);
			LiquidCrystal_I2C_Print("Mantenlos Pulsados!");
			mensaje_inicio_mostrado = 1;
		}

		// Patrón de luces
		if (HAL_GetTick() - last_update_time >= led_delay_ms)
		{
			last_update_time = HAL_GetTick();

			// Apagar todos los LEDs
			HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_RESET);

			// Encender LED actual
			switch (led_index)
			{
			case 0: HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_SET); break;
			case 1: HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_SET); break;
			case 2: HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_SET); break;
			case 3: HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_SET); break;
			case 4: HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_SET); break;
			}

			// Actualizar dirección
			led_index += direction;
			if (led_index == 4 || led_index == 0)
				direction *= -1;
		}

		// Leer botones
		GPIO_PinState b1 = HAL_GPIO_ReadPin(Boton_1_GPIO_Port, Boton_1_Pin);
		GPIO_PinState b2 = HAL_GPIO_ReadPin(Boton_2_GPIO_Port, Boton_2_Pin);
		GPIO_PinState b3 = HAL_GPIO_ReadPin(Boton_3_GPIO_Port, Boton_3_Pin);
		GPIO_PinState b4 = HAL_GPIO_ReadPin(Boton_4_GPIO_Port, Boton_4_Pin);
		GPIO_PinState b5 = HAL_GPIO_ReadPin(Boton_5_GPIO_Port, Boton_5_Pin);

		if (b1 == GPIO_PIN_SET && b2 == GPIO_PIN_SET && b3 == GPIO_PIN_SET &&
				b4 == GPIO_PIN_SET && b5 == GPIO_PIN_SET)
		{
			HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_RESET);

			LiquidCrystal_I2C_Clear();
			LiquidCrystal_I2C_SetCursor(0, 1);
			LiquidCrystal_I2C_Print("Iniciando rutina...");
			HAL_Delay(500);

			game_state = GAME_READY;
		}
	}
	break;
	case GAME_READY:
	{
		// Verifica si todos los parámetros están configurados
		if ((dificultad_actual == 1 || dificultad_actual == 2 || dificultad_actual == 3 || dificultad_actual == 'X') &&
				(tiempo_maximo_respuesta_ms == 1000 || tiempo_maximo_respuesta_ms == 800 ||tiempo_maximo_respuesta_ms == 2000 ||
						tiempo_maximo_respuesta_ms == 3000 || tiempo_maximo_respuesta_ms == 4000 || tiempo_maximo_respuesta_ms == 500 || tiempo_maximo_respuesta_ms == 10000|| tiempo_maximo_respuesta_ms == 400 || tiempo_maximo_respuesta_ms == 300 || tiempo_maximo_respuesta_ms == 200 || tiempo_maximo_respuesta_ms == 100) &&
						(total_rounds == 5 || total_rounds == 10 || total_rounds == 15 || total_rounds == 20))
		{
			// Inicializa variables
			current_round = 0;
			aciertos = 0;
			errores = 0;

			LiquidCrystal_I2C_Clear();
			LiquidCrystal_I2C_SetCursor(0, 1);
			LiquidCrystal_I2C_Print("Listo para empezar");
			HAL_Delay(500);

			game_state = GAME_NEW_PATTERN;  // Avanza al siguiente estado
		}
		else
		{
			// Esperando que el usuario configure los parámetros
			LiquidCrystal_I2C_Clear();
			LiquidCrystal_I2C_SetCursor(0, 0);
			LiquidCrystal_I2C_Print("Configura por UART:");
			LiquidCrystal_I2C_SetCursor(0, 1);
			LiquidCrystal_I2C_Print("Dificultad, Tiempo");
			LiquidCrystal_I2C_SetCursor(0, 2);
			LiquidCrystal_I2C_Print("y # Rondas");
			LiquidCrystal_I2C_SetCursor(0, 3);
			LiquidCrystal_I2C_Print("Ej: D1 T2000 R10");
			HAL_Delay(1500);
		}
	}
	break;
	case GAME_NEW_PATTERN:
	{
		// Limpia patrón anterior
		for (int i = 0; i < 5; i++) {
			patron_leds[i] = 0;
		}

		// Determinar cuántos LEDs encender según dificultad
		uint8_t num_leds = 0;
		switch (dificultad_actual)
		{
		case 1: num_leds = 1; break;
		case 2: num_leds = 4; break;
		case 3: num_leds = (rand() % 2) + 2; break; // 2 o 3
		case 'X': // Aleatorio entre niveles válidos
		{
			uint8_t opciones[] = {1, 2, 3, 4};
			num_leds = opciones[rand() % 4];
		}
		break;
		default:
			num_leds = 1; // Fallback por seguridad
			break;
		}

		// Seleccionar aleatoriamente los LEDs a encender
		uint8_t indices[5] = {0, 1, 2, 3, 4};
		for (int i = 0; i < 5; i++) {        // Fisher–Yates (o Knuth shuffle)
			int j = rand() % (5 - i) + i;
			uint8_t temp = indices[i];
			indices[i] = indices[j];
			indices[j] = temp;
		}

		for (int i = 0; i < num_leds; i++) {
			patron_leds[indices[i]] = 1;
		}

		// Encender los LEDs según el patrón
		HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, patron_leds[0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, patron_leds[1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, patron_leds[2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, patron_leds[3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, patron_leds[4] ? GPIO_PIN_SET : GPIO_PIN_RESET);

		// Mensaje en pantalla
		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print("Ronda ");
		char ronda[8];
		sprintf(ronda, "%d/%d", current_round + 1, total_rounds);
		LiquidCrystal_I2C_Print(ronda);

		LiquidCrystal_I2C_SetCursor(0, 2);
		LiquidCrystal_I2C_Print("Levanta LED(s) ON");

		// Aquí iría la lógica para temporizador si se desea
		game_state = GAME_WAIT_INPUT;
	}
	break;
	case GAME_WAIT_INPUT:
	{
		static uint32_t tiempo_inicio = 0;
		static uint8_t iniciado = 0;

		if (!iniciado)
		{
			tiempo_inicio = HAL_GetTick();  // Guardamos el tiempo inicial
			iniciado = 1;
		}

		// Leer el estado actual de los botones (0 si está presionado)
		respuesta_usuario[0] = (HAL_GPIO_ReadPin(Boton_1_GPIO_Port, Boton_1_Pin) == GPIO_PIN_RESET) ? 1 : 0;
		respuesta_usuario[1] = (HAL_GPIO_ReadPin(Boton_2_GPIO_Port, Boton_2_Pin) == GPIO_PIN_RESET) ? 1 : 0;
		respuesta_usuario[2] = (HAL_GPIO_ReadPin(Boton_3_GPIO_Port, Boton_3_Pin) == GPIO_PIN_RESET) ? 1 : 0;
		respuesta_usuario[3] = (HAL_GPIO_ReadPin(Boton_4_GPIO_Port, Boton_4_Pin) == GPIO_PIN_RESET) ? 1 : 0;
		respuesta_usuario[4] = (HAL_GPIO_ReadPin(Boton_5_GPIO_Port, Boton_5_Pin) == GPIO_PIN_RESET) ? 1 : 0;

		// Verificamos si el usuario ha soltado los dedos donde hay LED encendido
		uint8_t match = 1;
		for (int i = 0; i < 5; i++)
		{
			if (patron_leds[i] != respuesta_usuario[i])
			{
				match = 0;
				break;
			}
		}

		// Si ya hizo la acción correctamente → avanzar a evaluación
		if (match)
		{
			iniciado = 0;
			game_state = GAME_EVALUATE;
			break;
		}

		// Si se pasó el tiempo máximo → avanzar a evaluación con penalización
		if (HAL_GetTick() - tiempo_inicio >= tiempo_maximo_respuesta_ms)
		{
			iniciado = 0;
			game_state = GAME_EVALUATE;
			break;
		}

		// Aún esperando entrada
	}
	break;
	case GAME_EVALUATE:
	{
		uint8_t correcto = 1;

		// Comparamos la respuesta con el patrón original
		for (int i = 0; i < 5; i++)
		{
			if (respuesta_usuario[i] != patron_leds[i])
			{
				correcto = 0;
				break;
			}
		}

		if (correcto)
		{
			aciertos++;
		}
		else
		{
			errores++;
		}

		game_state = GAME_FEEDBACK;
	}
	break;
	case GAME_WAIT_FINGERS:
	{
		// Mostrar mensaje solo una vez
		static uint8_t mostrado = 0;
		if (!mostrado)
		{
			LiquidCrystal_I2C_Clear();
			LiquidCrystal_I2C_SetCursor(0, 0);
			LiquidCrystal_I2C_Print("Siguiente ronda...");
			LiquidCrystal_I2C_SetCursor(0, 1);
			LiquidCrystal_I2C_Print("Coloca los 5 dedos");
			LiquidCrystal_I2C_SetCursor(0, 2);
			LiquidCrystal_I2C_Print("y manten presionado");
			mostrado = 1;
		}

		// Detectar los 5 botones presionados
		GPIO_PinState b1 = HAL_GPIO_ReadPin(Boton_1_GPIO_Port, Boton_1_Pin);
		GPIO_PinState b2 = HAL_GPIO_ReadPin(Boton_2_GPIO_Port, Boton_2_Pin);
		GPIO_PinState b3 = HAL_GPIO_ReadPin(Boton_3_GPIO_Port, Boton_3_Pin);
		GPIO_PinState b4 = HAL_GPIO_ReadPin(Boton_4_GPIO_Port, Boton_4_Pin);
		GPIO_PinState b5 = HAL_GPIO_ReadPin(Boton_5_GPIO_Port, Boton_5_Pin);

		if (b1 == GPIO_PIN_SET && b2 == GPIO_PIN_SET && b3 == GPIO_PIN_SET &&
				b4 == GPIO_PIN_SET && b5 == GPIO_PIN_SET)
		{
			HAL_Delay(200); // Anti-rebote opcional

			HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_RESET);

			mostrado = 0; // Reiniciar para siguiente ronda
			game_state = GAME_NEW_PATTERN;
		}
	}
	break;
	case GAME_FEEDBACK:
	{
		// Mostrar resultado en el LCD
		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);

		if (respuesta_usuario[0] == patron_leds[0] &&
				respuesta_usuario[1] == patron_leds[1] &&
				respuesta_usuario[2] == patron_leds[2] &&
				respuesta_usuario[3] == patron_leds[3] &&
				respuesta_usuario[4] == patron_leds[4])
		{

			LiquidCrystal_I2C_Print(" ¡CORRECTO! ");
			Buzzer_SuccessTone();
		}
		else
		{

			LiquidCrystal_I2C_Print("   ERROR :(   ");
			LiquidCrystal_I2C_SetCursor(0, 2);
			LiquidCrystal_I2C_Print("Intenta de nuevo");
			Buzzer_ErrorTone();
		}

		// Apagar todos los LEDs
		HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_RESET);

		HAL_Delay(1000);  // Pausa para que usuario vea el resultado

		current_round++;

		if (current_round >= total_rounds)
		{
			game_state = GAME_OVER;
		}
		else
		{
			game_state = GAME_WAIT_FINGERS;
		}
	}
	break;
	case GAME_OVER:
	{
		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print("RUTINA COMPLETADA");

		// Mostrar resumen de aciertos
		char resultado[21];
		snprintf(resultado, sizeof(resultado), "Aciertos: %d/%d", aciertos, total_rounds);
		LiquidCrystal_I2C_SetCursor(0, 1);
		LiquidCrystal_I2C_Print(resultado);

		// Mostrar errores
		snprintf(resultado, sizeof(resultado), "Errores : %d", errores);
		LiquidCrystal_I2C_SetCursor(0, 2);
		LiquidCrystal_I2C_Print(resultado);

		// Calcular tasa de acierto
		float porcentaje = 0.0f;
		if (total_rounds > 0) {
			porcentaje = ((float)aciertos / (float)total_rounds) * 100.0f;
		}

		const char* mensaje_final;
		if (porcentaje == 0.0f)           mensaje_final = "MUY SUAVE Zzz..";
		else if (porcentaje < 30.0f)      mensaje_final = "SUAVE";
		else if (porcentaje < 50.0f)      mensaje_final = "DECENTE";
		else if (porcentaje < 70.0f)      mensaje_final = "BIEN";
		else if (porcentaje < 90.0f)      mensaje_final = "MUY BIEN!";
		else                              mensaje_final = "PERFECTO!!";

		// Mostrar evaluación en línea 3
		LiquidCrystal_I2C_SetCursor(0, 3);
		LiquidCrystal_I2C_Print(mensaje_final);

		Buzzer_JingleGameOver();

		HAL_Delay(3000);  // Espera para que el usuario vea el resumen

		mensaje_inicio_mostrado = 0;
		game_state = GAME_WAIT_START;
	}
	break;
	case GAME_FREE_PLAY:

		GameFSM_Update_FreePlaySonoro();
		break;

	case GAME_MEMORY:
		GameFSM_Update_MemoryGame();
		break;


	default:
		break;
	}
}
void GameFSM_Update_FreePlaySonoro(void)
{
	static uint8_t mostrado = 0;
	static int8_t nota_activa = -1;  // -1 = sin sonido

	if (!mostrado && !play_song_mostrar_lcd)
	{
		Buzzer_SuccessTone();
		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print(" MODO LIBRE ACTIVO ");
		LiquidCrystal_I2C_SetCursor(0, 2);
		LiquidCrystal_I2C_Print(" Presiona botones :)");
		mostrado = 1;
	}

	// Para cada botón, detectar estado actual
	for (int i = 0; i < 5; i++)
	{
		GPIO_PinState estado = HAL_GPIO_ReadPin(botones_sonido[i].port, botones_sonido[i].pin);
		uint8_t presionado = (estado == GPIO_PIN_SET);

		// Flanco de subida: no estaba antes, ahora sí
		if (presionado && !botones_sonido[i].estado_anterior)
		{
			// Reproducir esta nota
			__HAL_TIM_SET_AUTORELOAD(&htim3, botones_sonido[i].arr);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, botones_sonido[i].ccr);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
			nota_activa = i;
		}

		// Flanco de bajada (se soltó la nota activa)
		if (!presionado && botones_sonido[i].estado_anterior && i == nota_activa)
		{
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
			nota_activa = -1;
		}

		botones_sonido[i].estado_anterior = presionado;
	}

	// Actualizar LEDs
	HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, HAL_GPIO_ReadPin(Boton_1_GPIO_Port, Boton_1_Pin));
	HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, HAL_GPIO_ReadPin(Boton_2_GPIO_Port, Boton_2_Pin));
	HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, HAL_GPIO_ReadPin(Boton_3_GPIO_Port, Boton_3_Pin));
	HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, HAL_GPIO_ReadPin(Boton_4_GPIO_Port, Boton_4_Pin));
	HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, HAL_GPIO_ReadPin(Boton_5_GPIO_Port, Boton_5_Pin));

	// Salida del modo libre
	GPIO_PinState b1 = HAL_GPIO_ReadPin(Boton_1_GPIO_Port, Boton_1_Pin);
	GPIO_PinState b2 = HAL_GPIO_ReadPin(Boton_2_GPIO_Port, Boton_2_Pin);
	GPIO_PinState b3 = HAL_GPIO_ReadPin(Boton_3_GPIO_Port, Boton_3_Pin);
	GPIO_PinState b4 = HAL_GPIO_ReadPin(Boton_4_GPIO_Port, Boton_4_Pin);
	GPIO_PinState b5 = HAL_GPIO_ReadPin(Boton_5_GPIO_Port, Boton_5_Pin);

	if (b1 == GPIO_PIN_SET && b2 == GPIO_PIN_SET && b3 == GPIO_PIN_SET &&
			b4 == GPIO_PIN_SET && b5 == GPIO_PIN_SET)
	{
		HAL_Delay(300);

		// Apagar sonido y LEDs
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_RESET);

		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 1);
		LiquidCrystal_I2C_Print("Saliendo de libre...");
		play_song_mostrar_lcd = 0;
		HAL_Delay(2000);

		// Mostrar mensaje inicial de configuración
		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print("Configura por UART:");
		LiquidCrystal_I2C_SetCursor(0, 1);
		LiquidCrystal_I2C_Print("Dificultad, Tiempo");
		LiquidCrystal_I2C_SetCursor(0, 2);
		LiquidCrystal_I2C_Print("y # Rondas");
		LiquidCrystal_I2C_SetCursor(0, 3);
		LiquidCrystal_I2C_Print("Ej: D1 T2000 R10");

		mensaje_inicio_mostrado = 0;
		mostrado = 0;
		game_state = GAME_WAIT_START;
	}
}

void GameFSM_Update_MemoryGame(void)
{
	switch (memory_state)
	{
	case MEMORY_INIT:
	{
		memory_sequence_length = 1;
		memory_input_index = 0;
		memory_sequence[0] = rand() % 5;

		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print(" Entrena tu memoria ");
		LiquidCrystal_I2C_SetCursor(0, 2);
		LiquidCrystal_I2C_Print("  Preparate... ");
		//		Buzzer_StartupPulse();

		// Transición limpia: espera antes de iniciar la secuencia
		HAL_Delay(1500);
		memory_state = MEMORY_WAIT_BEFORE_SHOW;
		last_note_time = HAL_GetTick();
		break;
	}
	case MEMORY_WAIT_BEFORE_SHOW:
	{
		// Espera breve antes de iniciar secuencia
		if (HAL_GetTick() - last_note_time >= 800) // espera 800 ms más
		{
			showing_index = 0;
			showing_led_on = 0;
			last_note_time = HAL_GetTick();
			memory_state = MEMORY_SHOW_SEQUENCE;
		}
		break;
	}


	case MEMORY_SHOW_SEQUENCE:
	{
		uint32_t current_time = HAL_GetTick();

		if (!showing_led_on && current_time - last_note_time >= 300)
		{
			uint8_t led = memory_sequence[showing_index];
			switch (led)
			{
			case 0: HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_SET); break;
			case 1: HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_SET); break;
			case 2: HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_SET); break;
			case 3: HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_SET); break;
			case 4: HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_SET); break;
			}

			__HAL_TIM_SET_AUTORELOAD(&htim3, botones_sonido[led].arr);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, botones_sonido[led].ccr);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

			showing_led_on = 1;
			last_note_time = current_time;
		}
		else if (showing_led_on && current_time - last_note_time >= 400)
		{
			HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_RESET);

			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

			showing_index++;
			showing_led_on = 0;
			last_note_time = current_time;
		}

		if (showing_index >= memory_sequence_length)
		{
			memory_input_index = 0;
			HAL_Delay(300);
			LiquidCrystal_I2C_Clear();
			LiquidCrystal_I2C_SetCursor(0, 1);
			LiquidCrystal_I2C_Print("Repite la secuencia");
			memory_state = MEMORY_WAIT_INPUT;
		}
		break;
	}

	case MEMORY_WAIT_INPUT:
	{
		static int8_t boton_presionado = -1;

		if (AreAllButtonsPressed())
		{
			// Salida forzada del juego
			LiquidCrystal_I2C_Clear();
			LiquidCrystal_I2C_SetCursor(0, 1);
			LiquidCrystal_I2C_Print("  Cancelando..  ");
			Buzzer_JingleGameOver();  // Puedes usar otro sonido si prefieres
			HAL_Delay(2000);
			LiquidCrystal_I2C_Clear();
			LiquidCrystal_I2C_SetCursor(0, 0);
			LiquidCrystal_I2C_Print("Configura por UART:");
			LiquidCrystal_I2C_SetCursor(0, 1);
			LiquidCrystal_I2C_Print("Dificultad, Tiempo");
			LiquidCrystal_I2C_SetCursor(0, 2);
			LiquidCrystal_I2C_Print("y # Rondas");
			LiquidCrystal_I2C_SetCursor(0, 3);
			LiquidCrystal_I2C_Print("Ej: D1 T2000 R10");

			mensaje_inicio_mostrado = 0;
			memory_state = MEMORY_INIT;
			game_state = GAME_WAIT_START;
			return;
		}
		for (uint8_t i = 0; i < 5; i++)
		{
			GPIO_PinState estado = HAL_GPIO_ReadPin(botones_sonido[i].port, botones_sonido[i].pin);

			// Si aún no hay botón activo y se presiona uno
			if (estado == GPIO_PIN_SET && !botones_sonido[i].estado_anterior && boton_presionado == -1)
			{
				// Comenzar sonido y encender LED
				__HAL_TIM_SET_AUTORELOAD(&htim3, botones_sonido[i].arr);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, botones_sonido[i].ccr);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

				switch (i)
				{
				case 0: HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_SET); break;
				case 1: HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_SET); break;
				case 2: HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_SET); break;
				case 3: HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_SET); break;
				case 4: HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_SET); break;
				}

				boton_presionado = i;  // Guardar cuál botón fue
			}

			// Si el botón que estaba presionado se suelta
			if (estado == GPIO_PIN_RESET && botones_sonido[i].estado_anterior && boton_presionado == i)
			{
				// Apagar LED y sonido
				HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Led_3_GPIO_Port, Led_3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Led_4_GPIO_Port, Led_4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Led_5_GPIO_Port, Led_5_Pin, GPIO_PIN_RESET);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

				// Evaluar entrada
				if (i == memory_sequence[memory_input_index])
				{
					memory_input_index++;
					if (memory_input_index >= memory_sequence_length)
					{
						if (memory_sequence_length < sizeof(memory_sequence))
						{
							memory_sequence[memory_sequence_length] = rand() % 5;
							memory_sequence_length++;

							LiquidCrystal_I2C_Clear();
							LiquidCrystal_I2C_SetCursor(0, 1);
							LiquidCrystal_I2C_Print("  CORRECTO! ");
							Buzzer_SuccessTone();
							HAL_Delay(1000);

							showing_index = 0;
							showing_led_on = 0;
							last_note_time = HAL_GetTick();
							memory_state = MEMORY_SHOW_SEQUENCE;
						}
						else
						{
							memory_state = MEMORY_GAME_OVER;
						}
					}
				}
				else
				{
					memory_state = MEMORY_GAME_OVER;
				}

				boton_presionado = -1;  // Limpiar estado
			}

			// Guardar estado actual para detección de flanco
			botones_sonido[i].estado_anterior = (estado == GPIO_PIN_SET);
		}
		break;
	}

	case MEMORY_GAME_OVER:
	{
		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print("  Juego terminado  ");
		LiquidCrystal_I2C_SetCursor(0, 1);
		char msg[21];
		snprintf(msg, sizeof(msg), "Puntos: %d", memory_sequence_length - 1);
		LiquidCrystal_I2C_Print(msg);
		LiquidCrystal_I2C_SetCursor(0, 2);
		LiquidCrystal_I2C_Print("-Mejor puntuacion-");
		LiquidCrystal_I2C_SetCursor(0, 3);
		LiquidCrystal_I2C_Print("ESTEBAN : 16");
		Buzzer_JingleGameOver();
		HAL_Delay(4000);

		mensaje_inicio_mostrado = 0;
		memory_state = MEMORY_INIT;
		game_state = GAME_WAIT_START;
		LiquidCrystal_I2C_Clear();
		LiquidCrystal_I2C_SetCursor(0, 0);
		LiquidCrystal_I2C_Print("Configura por UART:");
		LiquidCrystal_I2C_SetCursor(0, 1);
		LiquidCrystal_I2C_Print("Dificultad, Tiempo");
		LiquidCrystal_I2C_SetCursor(0, 2);
		LiquidCrystal_I2C_Print("y # Rondas");
		LiquidCrystal_I2C_SetCursor(0, 3);
		LiquidCrystal_I2C_Print("Ej: D1 T2000 R10");

		break;
	}

	default:
		memory_state = MEMORY_INIT;
		break;
	}
}
bool AreAllButtonsPressed(void)
{
	for (uint8_t i = 0; i < 5; i++)
	{
		if (HAL_GPIO_ReadPin(botones_sonido[i].port, botones_sonido[i].pin) == GPIO_PIN_RESET)
			return false;
	}
	return true;
}
void ReproducirEjercicioHanon1(void)
{
	// Secuencia: 1 2 3 4 5 4 3 2
	uint8_t secuencia[] = {1, 2, 3, 4, 5, 4, 3, 2 , 1};

	for (uint8_t i = 0; i < sizeof(secuencia); i++) {
		uint8_t dedo = secuencia[i];
		uint16_t arr = 0, ccr = 0;

		switch(dedo) {
		case 1: arr = 3823; ccr = 1911; break; // Do
		case 2: arr = 3405; ccr = 1702; break; // Re
		case 3: arr = 3033; ccr = 1516; break; // Mi
		case 4: arr = 2863; ccr = 1431; break; // Fa
		case 5: arr = 2551; ccr = 1275; break; // Sol
		}

		__HAL_TIM_SET_AUTORELOAD(&htim3, arr);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

		HAL_Delay(250);

		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		HAL_Delay(100);
	}
}
void ReproducirEjercicioHanon2(void)
{
	uint8_t secuencia[] = {1, 3, 2, 4, 3, 5};

	for (uint8_t i = 0; i < sizeof(secuencia); i++) {
		uint8_t dedo = secuencia[i];
		uint16_t arr = 0, ccr = 0;

		switch(dedo) {
		case 1: arr = 3823; ccr = 1911; break; // Do
		case 2: arr = 3405; ccr = 1702; break; // Re
		case 3: arr = 3033; ccr = 1516; break; // Mi
		case 4: arr = 2863; ccr = 1431; break; // Fa
		case 5: arr = 2551; ccr = 1275; break; // Sol
		}

		__HAL_TIM_SET_AUTORELOAD(&htim3, arr);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

		HAL_Delay(250);

		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		HAL_Delay(100);
	}
}
void ReproducirEjercicioHanon3(void)
{
	uint8_t secuencia[] = {1, 1, 2, 2, 3, 3, 4, 4};

	for (uint8_t i = 0; i < sizeof(secuencia); i++) {
		uint8_t dedo = secuencia[i];
		uint16_t arr = 0, ccr = 0;

		switch(dedo) {
		case 1: arr = 3823; ccr = 1911; break; // Do
		case 2: arr = 3405; ccr = 1702; break; // Re
		case 3: arr = 3033; ccr = 1516; break; // Mi
		case 4: arr = 2863; ccr = 1431; break; // Fa
		case 5: arr = 2551; ccr = 1275; break; // Sol
		}

		__HAL_TIM_SET_AUTORELOAD(&htim3, arr);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

		HAL_Delay(250);

		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		HAL_Delay(100);
	}
}
void ReproducirEjercicioHanon4(void)
{
	uint8_t secuencia[] = {1, 5, 2, 5, 3, 5, 4, 5};

	for (uint8_t i = 0; i < sizeof(secuencia); i++) {
		uint8_t dedo = secuencia[i];
		uint16_t arr = 0, ccr = 0;

		switch(dedo) {
		case 1: arr = 3823; ccr = 1911; break; // Do
		case 2: arr = 3405; ccr = 1702; break; // Re
		case 3: arr = 3033; ccr = 1516; break; // Mi
		case 4: arr = 2863; ccr = 1431; break; // Fa
		case 5: arr = 2551; ccr = 1275; break; // Sol
		}

		__HAL_TIM_SET_AUTORELOAD(&htim3, arr);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

		HAL_Delay(250);

		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		HAL_Delay(100);
	}
}
void ReproducirEjercicioHanon5(void)
{
	uint8_t secuencia[] = {1, 2, 3, 4, 5, 4, 3, 4, 5};

	for (uint8_t i = 0; i < sizeof(secuencia); i++) {
		uint8_t dedo = secuencia[i];
		uint16_t arr = 0, ccr = 0;

		switch(dedo) {
			case 1: arr = 3823; ccr = 1911; break; // Do
			case 2: arr = 3405; ccr = 1702; break; // Re
			case 3: arr = 3033; ccr = 1516; break; // Mi
			case 4: arr = 2863; ccr = 1431; break; // Fa
			case 5: arr = 2551; ccr = 1275; break; // Sol
		}

		__HAL_TIM_SET_AUTORELOAD(&htim3, arr);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

		HAL_Delay(250);

		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		HAL_Delay(100);
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
