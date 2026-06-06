#include <stdio.h>
#include <stdint.h>

#include "FreeRTOS.h"     
#include "task.h"
#include "queue.h"
#include "event_groups.h"  

/* --- ECU Data Structures --- */
typedef struct {
    uint32_t engine_rpm;
    uint8_t throttle_position; // 0 to 100%
} SensorData_t;

typedef struct {
    uint32_t fuel_injection_time_ms;
} ActuatorCmd_t;

/* --- CAN Bus Data Structure --- */
typedef struct {
    uint32_t messageID;  // 11-bit Standard CAN ID (e.g., 0x1A4)
    uint8_t DLC;         // Data Length Code (How many bytes: 0 to 8)
    uint8_t payload[8];  // The actual raw data bytes
} CAN_Frame_t;

/* --- Global Queue Handles --- */
QueueHandle_t xSensorQueue;
QueueHandle_t xCANQueue;

/* --- Watchdog Event Group --- */
EventGroupHandle_t xWatchdogGroup;

/* Bit mapping for each task */
#define SENSOR_ALIVE_BIT   ( 1 << 0 ) // 001
#define CORE_ALIVE_BIT     ( 1 << 1 ) // 010
#define ACTUATOR_ALIVE_BIT ( 1 << 2 ) // 100
#define ALL_TASKS_ALIVE    ( SENSOR_ALIVE_BIT | CORE_ALIVE_BIT | ACTUATOR_ALIVE_BIT )

/* --- Task Prototypes --- */
static void prvSensorTask(void* pvParameters);
static void prvProcessingTask(void* pvParameters);
static void prvActuatorTask(void* pvParameters);
static void prvWatchdogTask(void* pvParameters); /* FIXED: Added Watchdog prototype */

/* We keep this function name so main.c still calls it automatically */
void main_blinky(void)
{
    printf("--- Automotive ECU Simulator Booting ---\r\n");

    /* FIXED: Removed the duplicated initialization code.
       We now only initialize everything once, correctly. */

       /* 1. Create the Queues and Event Groups */
    xSensorQueue = xQueueCreate(5, sizeof(SensorData_t));
    xCANQueue = xQueueCreate(5, sizeof(CAN_Frame_t));
    xWatchdogGroup = xEventGroupCreate();

    if (xSensorQueue != NULL && xCANQueue != NULL && xWatchdogGroup != NULL)
    {
        /* 2. Create Tasks (Watchdog gets Priority 4 - The highest) */
        xTaskCreate(prvWatchdogTask, "Watchdog", configMINIMAL_STACK_SIZE, NULL, 4, NULL);

        xTaskCreate(prvSensorTask, "Sensor", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
        xTaskCreate(prvProcessingTask, "ECU_Core", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
        xTaskCreate(prvActuatorTask, "Actuator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

        /* 3. Hand over control to the RTOS Scheduler */
        vTaskStartScheduler();
    }

    /* The program should never reach here unless there is insufficient RAM */
    for (;; );
}

/* --- Task Implementations --- */

static void prvSensorTask(void* pvParameters)
{
    SensorData_t currentData = { 1000, 10 };
    const TickType_t xDelay = pdMS_TO_TICKS(500);

    for (;; )
    {
        currentData.engine_rpm += 50;
        if (currentData.throttle_position < 100) currentData.throttle_position += 2;

        printf("[SENSOR] Reading Throttle: %d%% | RPM: %lu\r\n", currentData.throttle_position, currentData.engine_rpm);
        xQueueSend(xSensorQueue, &currentData, 0);

        /* KICK THE DOG: Tell Watchdog the Sensor is alive */
        xEventGroupSetBits(xWatchdogGroup, SENSOR_ALIVE_BIT);

        vTaskDelay(xDelay);
    }
}

static void prvProcessingTask(void* pvParameters)
{
    SensorData_t receivedData;
    CAN_Frame_t txFrame;

    for (;; )
    {
        /* Changed portMAX_DELAY to 1000ms. If no data arrives in 1 sec, it skips the IF block and kicks the dog anyway */
        if (xQueueReceive(xSensorQueue, &receivedData, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            uint32_t calc_inj = (receivedData.engine_rpm / 100) * receivedData.throttle_position;

            txFrame.messageID = 0x1A4;
            txFrame.DLC = 8;
            txFrame.payload[0] = (uint8_t)(receivedData.engine_rpm >> 8);
            txFrame.payload[1] = (uint8_t)(receivedData.engine_rpm & 0xFF);
            txFrame.payload[2] = receivedData.throttle_position;
            txFrame.payload[3] = (uint8_t)(calc_inj >> 8);
            txFrame.payload[4] = (uint8_t)(calc_inj & 0xFF);
            txFrame.payload[5] = 0x00; txFrame.payload[6] = 0x00; txFrame.payload[7] = 0x00;

            xQueueSend(xCANQueue, &txFrame, 0);
        }

        /* KICK THE DOG */
        xEventGroupSetBits(xWatchdogGroup, CORE_ALIVE_BIT);
    }
}

static void prvActuatorTask(void* pvParameters)
{
    CAN_Frame_t rxFrame;

    for (;; )
    {
        /* Changed to 1000ms timeout */
        if (xQueueReceive(xCANQueue, &rxFrame, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            printf("   -> [CAN BUS] ID: 0x%03X | DLC: %d | Data: ", rxFrame.messageID, rxFrame.DLC);
            for (int i = 0; i < rxFrame.DLC; i++) {
                printf("%02X ", rxFrame.payload[i]);
            }
            printf("\r\n\r\n");
        }

        /* KICK THE DOG */
        xEventGroupSetBits(xWatchdogGroup, ACTUATOR_ALIVE_BIT);
    }
}

static void prvWatchdogTask(void* pvParameters)
{
    for (;; )
    {
        /* Wait up to 2000ms for ALL tasks to set their alive bits. */
        EventBits_t uxBits = xEventGroupWaitBits(
            xWatchdogGroup,
            ALL_TASKS_ALIVE,
            pdTRUE,          /* Clear bits on exit */
            pdTRUE,          /* Wait for ALL bits */
            pdMS_TO_TICKS(2000)
        );

        /* Check if the timeout occurred before all bits were set */
        if ((uxBits & ALL_TASKS_ALIVE) == ALL_TASKS_ALIVE)
        {
            /* Success! Every task reported in. */
            printf("\r\n*** [SUPERVISOR] All ECUs Healthy - Watchdog Reset ***\r\n\r\n");
        }
        else
        {
            /* Failure! Someone missed their deadline. */
            printf("\r\n!!! [FATAL ERROR] TASK HUNG! INITIATING EMERGENCY ECU REBOOT !!!\r\n\r\n");

            vTaskSuspendAll();
            for (;;);
        }
    }
}

/* Dummy implementation to satisfy the Windows simulator keyboard hook */
void vBlinkyKeyboardInterruptHandler(const uint32_t xKey)
{
    (void)xKey;
}