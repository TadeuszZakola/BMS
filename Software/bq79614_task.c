#include "bq79614.h"
#include "freertos.h"
#include "task.h"

extern UART_HandleTypeDef huart4;

float cell_voltages[2][14];
float temperatures[2][8];

static BQ79614_Device bq79614;

void BQ79614_Task(void *pvParameters) {
    if (BQ79614_Init(&bq79614, &huart4) != HAL_OK) {
        while (1);
    }
    if (BQ79614_InitUART(&bq79614) != HAL_OK) {
        while (1);
    }
    if (BQ79614_InitStack(&bq79614) != HAL_OK) {
        while (1);
    }
    if (BQ79614_InitVoltageMeasurement(&bq79614) != HAL_OK) {
        while (1);
    }
    if (BQ79614_InitOVUV(&bq79614, 3050, 4300) != HAL_OK) {
        while (1);
    }
    if (BQ79614_InitTemperatureMeasurements(&bq79614) != HAL_OK) {
        while (1);
    }
    if (BQ79614_InitOTUT(&bq79614, 70, 30) != HAL_OK) {
        while (1);
    }
    if (BQ79614_StartMeasurements(&bq79614) != HAL_OK) {
        while (1);
    }

    while (1) {
        BQ79614_UpdateData(&bq79614);
        BQ79614_UpdateStatus(&bq79614);

        for (size_t ic = 0; ic < BQ79614_STACK_SIZE; ic++) {
            for (size_t cell = 0; cell < BQ79614_CELL_COUNT; cell++) {
                cell_voltages[ic][cell] = bq79614.stack_device_data[ic].voltages[cell];
            }
            for (size_t temp = 0; temp < BQ79614_TEMP_COUNT; temp++) {
                temperatures[ic][temp] = bq79614.stack_device_data[ic].temperatures[temp];
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}