/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
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
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;
osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* --- ECU Data Structures --- */
typedef struct {
    uint32_t engine_rpm;
    uint8_t throttle_position;
} SensorData_t;

/* --- Global RTOS Handles --- */
QueueHandle_t xSensorQueue;
EventGroupHandle_t xWatchdogGroup;

/* Bit mapping for Watchdog */
#define SENSOR_ALIVE_BIT   ( 1 << 0 )
#define CORE_ALIVE_BIT     ( 1 << 1 )
#define ACTUATOR_ALIVE_BIT ( 1 << 2 )
#define ALL_TASKS_ALIVE    ( SENSOR_ALIVE_BIT | CORE_ALIVE_BIT | ACTUATOR_ALIVE_BIT )
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
static void prvSensorTask(void* pvParameters);
static void prvProcessingTask(void* pvParameters);
static void prvActuatorTask(void* pvParameters);
static void prvWatchdogTask(void* pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Retargets printf() to output over the ST-Link USB cable (USART2) */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
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
  MX_CAN1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  printf("\r\n\r\n--- Physical STM32 ECU Booting ---\r\n");

  /* --- 1. Configure Hardware CAN Filter to accept loopback messages --- */
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* Start the physical CAN peripheral */
  HAL_CAN_Start(&hcan1);

  /* --- 2. Create RTOS Objects --- */
  xSensorQueue = xQueueCreate(5, sizeof(SensorData_t));
  xWatchdogGroup = xEventGroupCreate();

  if (xSensorQueue != NULL && xWatchdogGroup != NULL)
  {
      /* --- 3. Create Custom Native FreeRTOS Tasks --- */
      xTaskCreate(prvWatchdogTask, "Watchdog", 256, NULL, 4, NULL);
      xTaskCreate(prvSensorTask, "Sensor", 256, NULL, 3, NULL);
      xTaskCreate(prvProcessingTask, "ECU_Core", 256, NULL, 2, NULL);
      xTaskCreate(prvActuatorTask, "Actuator", 256, NULL, 1, NULL);
  }
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Code here is never executed once the RTOS starts */
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_CAN1_Init(void)
{
  /* USER CODE BEGIN CAN1_Init 0 */
  /* USER CODE END CAN1_Init 0 */
  /* USER CODE BEGIN CAN1_Init 1 */
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /* USER CODE END CAN1_Init 2 */
}

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

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* --- Custom FreeRTOS Tasks --- */

static void prvSensorTask(void* pvParameters)
{
    SensorData_t currentData = { 1000, 10 };

    for (;; )
    {
        currentData.engine_rpm += 50;
        if (currentData.throttle_position < 100) currentData.throttle_position += 2;

        printf("[SENSOR] Reading Throttle: %d%% | RPM: %lu\r\n", currentData.throttle_position, currentData.engine_rpm);

        xQueueSend(xSensorQueue, &currentData, 0);
        xEventGroupSetBits(xWatchdogGroup, SENSOR_ALIVE_BIT);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void prvProcessingTask(void* pvParameters)
{
    SensorData_t receivedData;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t txPayload[8];

    /* Setup CAN Frame Header */
    TxHeader.StdId = 0x1A4;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    for (;; )
    {
        if (xQueueReceive(xSensorQueue, &receivedData, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            uint32_t calc_inj = (receivedData.engine_rpm / 100) * receivedData.throttle_position;

            /* Pack payload */
            txPayload[0] = (uint8_t)(receivedData.engine_rpm >> 8);
            txPayload[1] = (uint8_t)(receivedData.engine_rpm & 0xFF);
            txPayload[2] = receivedData.throttle_position;
            txPayload[3] = (uint8_t)(calc_inj >> 8);
            txPayload[4] = (uint8_t)(calc_inj & 0xFF);
            txPayload[5] = 0x00; txPayload[6] = 0x00; txPayload[7] = 0x00;

            /* INJECT DIRECTLY TO PHYSICAL HARDWARE */
            if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
                HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txPayload, &TxMailbox);
            }
        }

        xEventGroupSetBits(xWatchdogGroup, CORE_ALIVE_BIT);
    }
}

static void prvActuatorTask(void* pvParameters)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    for (;; )
    {
        /* READ DIRECTLY FROM PHYSICAL SILICON FIFO */
        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
        {
            if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
            {
                printf("   -> [HDWR CAN] ID: 0x%03X | DLC: %lu | Data: ", (unsigned int)RxHeader.StdId, RxHeader.DLC);
                for (int i = 0; i < RxHeader.DLC; i++) {
                    printf("%02X ", RxData[i]);
                }
                printf("\r\n\r\n");
            }
        }

        xEventGroupSetBits(xWatchdogGroup, ACTUATOR_ALIVE_BIT);
        vTaskDelay(pdMS_TO_TICKS(50)); /* Prevent task starvation while polling */
    }
}

static void prvWatchdogTask(void* pvParameters)
{
    for (;; )
    {
        EventBits_t uxBits = xEventGroupWaitBits(
            xWatchdogGroup,
            ALL_TASKS_ALIVE,
            pdTRUE,
            pdTRUE,
            pdMS_TO_TICKS(2000)
        );

        if ((uxBits & ALL_TASKS_ALIVE) == ALL_TASKS_ALIVE)
        {
            printf("\r\n*** [SUPERVISOR] All ECUs Healthy - Watchdog Reset ***\r\n\r\n");
        }
        else
        {
            printf("\r\n!!! [FATAL ERROR] TASK HUNG! INITIATING SYSTEM RESET !!!\r\n\r\n");
            NVIC_SystemReset(); /* Hardware system reset command */
        }
    }
}
/* USER CODE END 4 */

void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
