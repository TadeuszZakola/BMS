#include "bq79614.h"
#include "crc16ibm.h"
#include <string.h>

static uint8_t BQ79614_InitByteWrite(BQ79614_Device *dev, BQ79614_ReqType req_type, uint8_t data_size) {
    uint8_t cmd = (req_type == BQ79614_REQ_BROADCAST ? 0x80 : 0x00) |
                  (req_type == BQ79614_REQ_STACK ? 0x40 : 0x00) |
                  (data_size - 1);
    return cmd;
}

static uint8_t BQ79614_InitByteRead(BQ79614_Device *dev, BQ79614_ReqType req_type) {
    uint8_t cmd = 0x20 |
                  (req_type == BQ79614_REQ_BROADCAST ? 0x80 : 0x00) |
                  (req_type == BQ79614_REQ_STACK ? 0x40 : 0x00);
    return cmd;
}

HAL_StatusTypeDef BQ79614_Init(BQ79614_Device *dev, UART_HandleTypeDef *huart) {
    dev->huart = huart;
    dev->read_size = 0;
    memset(dev->out, 0, sizeof(dev->out));
    memset(dev->in, 0, sizeof(dev->in));
    memset(dev->stack_device_status, 0, sizeof(dev->stack_device_status));
    memset(dev->stack_device_data, 0, sizeof(dev->stack_device_data));
    dev->semaphore = xSemaphoreCreateBinary();
    if (dev->semaphore == NULL) return HAL_ERROR;
    xSemaphoreGive(dev->semaphore);
    return HAL_OK;
}

HAL_StatusTypeDef BQ79614_InitUART(BQ79614_Device *dev) {
    UART_HandleTypeDef *huart = dev->huart;
    huart->Init.BaudRate = BQ79614_DEFAULT_BAUDRATE;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(huart) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef BQ79614_InitStack(BQ79614_Device *dev) {
    if (BQ79614_WakeUp(dev) != HAL_OK) return HAL_ERROR;
    BQ79614_Control1 ctrl = { .send_wake = 1 };
    uint8_t data = *(uint8_t*)&ctrl;
    return BQ79614_Write(dev, BQ79614_REQ_BROADCAST, &data, 1, BQ79614_REG_CONTROL1, 0);
}

HAL_StatusTypeDef BQ79614_InitVoltageMeasurement(BQ79614_Device *dev) {
    BQ79614_AdcCtrl1 adc_ctrl = { .main_mode = BQ79614_SCAN_ROUND_ROBIN, .main_go = 1 };
    uint8_t data = *(uint8_t*)&adc_ctrl;
    return BQ79614_Write(dev, BQ79614_REQ_BROADCAST, &data, 1, BQ79614_REG_ADCCTRL1, 0);
}

HAL_StatusTypeDef BQ79614_InitOVUV(BQ79614_Device *dev, uint32_t undervoltage, uint32_t overvoltage) {
    if (undervoltage < 1200 || undervoltage > 3100 || overvoltage < 4175 || overvoltage > 4475) return HAL_ERROR;
    uint8_t uv_data = (uint8_t)((undervoltage - 1200) / 25);
    uint8_t ov_data = (uint8_t)((overvoltage - 4175) / 5);
    HAL_StatusTypeDef status = BQ79614_Write(dev, BQ79614_REQ_BROADCAST, &uv_data, 1, 0x0001, 0);
    if (status != HAL_OK) return status;
    status = BQ79614_Write(dev, BQ79614_REQ_BROADCAST, &ov_data, 1, 0x0003, 0);
    if (status != HAL_OK) return status;
    BQ79614_OVUVCtrl ovuv_ctrl = { .ovuv_mode = BQ79614_SCAN_ROUND_ROBIN, .ovuv_go = 1 };
    uint8_t data = *(uint8_t*)&ovuv_ctrl;
    return BQ79614_Write(dev, BQ79614_REQ_BROADCAST, &data, 1, BQ79614_REG_OVUVCTRL, 0);
}

HAL_StatusTypeDef BQ79614_InitTemperatureMeasurements(BQ79614_Device *dev) {
    BQ79614_GPIOConf1 conf1 = { .gpio1 = BQ79614_GPIO_ADC_OTUT, .gpio2 = BQ79614_GPIO_ADC_OTUT };
    BQ79614_GPIOConf2 conf2 = { .gpio3 = BQ79614_GPIO_ADC_OTUT, .gpio4 = BQ79614_GPIO_ADC_OTUT };
    BQ79614_GPIOConf3 conf3 = { .gpio5 = BQ79614_GPIO_ADC_OTUT, .gpio6 = BQ79614_GPIO_ADC_OTUT };
    BQ79614_GPIOConf4 conf4 = { .gpio7 = BQ79614_GPIO_ADC_OTUT, .gpio8 = BQ79614_GPIO_ADC_OTUT };
    uint8_t data1 = *(uint8_t*)&conf1;
    uint8_t data2 = *(uint8_t*)&conf2;
    uint8_t data3 = *(uint8_t*)&conf3;
    uint8_t data4 = *(uint8_t*)&conf4;
    HAL_StatusTypeDef status = BQ79614_Write(dev, BQ79614_REQ_BROADCAST, &data1, 1, BQ79614_REG_GPIOCONF1, 0);
    if (status != HAL_OK) return status;
    status = BQ79614_Write(dev, BQ79614_REQ_BROADCAST, &data2, 1, BQ79614_REG_GPIOCONF2, 0);
    if (status != HAL_OK) return status;
    status = BQ79614_Write(dev, BQ79614_REQ_BROADCAST, &data3, 1, BQ79614_REG_GPIOCONF3, 0);
    if (status != HAL_OK) return status;
    return BQ79614_Write(dev, BQ79614_REQ_BROADCAST, &data4, 1, BQ79614_REG_GPIOCONF4, 0);
}

HAL_StatusTypeDef BQ79614_InitOTUT(BQ79614_Device *dev, uint8_t undertemperature, uint8_t overtemperature) {
    if (undertemperature < 66 || undertemperature > 80 || (undertemperature % 2) != 0 || overtemperature < 10 || overtemperature > 39) return HAL_ERROR;
    BQ79614_OTUTThresh thresh = { .ot_thr = overtemperature - 10, .ut_thr = (undertemperature - 66) / 2 };
    uint8_t data = *(uint8_t*)&thresh;
    HAL_StatusTypeDef status = BQ79614_Write(dev, BQ79614_REQ_BROADCAST, &data, 1, BQ79614_REG_OTUTTHRESH, 0);
    if (status != HAL_OK) return status;
    BQ79614_OTUTCtrl otut_ctrl = { .otut_mode = BQ79614_SCAN_ROUND_ROBIN, .otut_go = 1 };
    data = *(uint8_t*)&otut_ctrl;
    return BQ79614_Write(dev, BQ79614_REQ_BROADCAST, &data, 1, BQ79614_REG_OTUTCTRL, 0);
}

HAL_StatusTypeDef BQ79614_StartMeasurements(BQ79614_Device *dev) {
    BQ79614_AdcCtrl1 adc_ctrl = { .main_mode = BQ79614_SCAN_ROUND_ROBIN, .main_go = 1 };
    uint8_t data = *(uint8_t*)&adc_ctrl;
    return BQ79614_Write(dev, BQ79614_REQ_BROADCAST, &data, 1, BQ79614_REG_ADCCTRL1, 0);
}

HAL_StatusTypeDef BQ79614_UpdateStatus(BQ79614_Device *dev) {
    uint8_t data[2 * BQ79614_STACK_SIZE];
    HAL_StatusTypeDef status = BQ79614_Read(dev, BQ79614_REQ_STACK, data, 2, 0x0040, 0);
    if (status != HAL_OK) return status;
    for (size_t i = 0; i < BQ79614_STACK_SIZE; i++) {
        uint16_t status = (data[i * 2] << 8) | data[i * 2 + 1];
        for (size_t j = 0; j < BQ79614_CELL_COUNT; j++) {
            dev->stack_device_status[i].ovuv[j] = (status >> j) & 0x1;
        }
        for (size_t j = 0; j < BQ79614_TEMP_COUNT; j++) {
            dev->stack_device_status[i].otut[j] = (status >> (j + 16)) & 0x1;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef BQ79614_UpdateData(BQ79614_Device *dev) {
    uint8_t volt_data[2 * BQ79614_CELL_COUNT * BQ79614_STACK_SIZE];
    HAL_StatusTypeDef status = BQ79614_Read(dev, BQ79614_REQ_STACK, volt_data, 2 * BQ79614_CELL_COUNT, BQ79614_REG_CELL1_VOLTAGE, 0);
    if (status != HAL_OK) return status;
    for (size_t i = 0; i < BQ79614_STACK_SIZE; i++) {
        for (size_t j = 0; j < BQ79614_CELL_COUNT; j++) {
            uint16_t raw = (volt_data[i * BQ79614_CELL_COUNT * 2 + j * 2] << 8) | volt_data[i * BQ79614_CELL_COUNT * 2 + j * 2 + 1];
            dev->stack_device_data[i].voltages[j] = raw * BQ79614_V_LSB_ADC * 1000000.0;
        }
    }
    uint8_t temp_data[2 * BQ79614_TEMP_COUNT * BQ79614_STACK_SIZE];
    status = BQ79614_Read(dev, BQ79614_REQ_STACK, temp_data, 2 * BQ79614_TEMP_COUNT, BQ79614_REG_GPIO1, 0);
    if (status != HAL_OK) return status;
    for (size_t i = 0; i < BQ79614_STACK_SIZE; i++) {
        for (size_t j = 0; j < BQ79614_TEMP_COUNT; j++) {
            uint16_t raw = (temp_data[i * BQ79614_TEMP_COUNT * 2 + j * 2] << 8) | temp_data[i * BQ79614_TEMP_COUNT * 2 + j * 2 + 1];
            dev->stack_device_data[i].temperatures[j] = raw * BQ79614_V_LSB_ADC * 1000000.0 / 1000.0; // Assuming temperature in mV, scaled to degC
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef BQ79614_WakeUp(BQ79614_Device *dev) {
    UART_HandleTypeDef *huart = dev->huart;
    huart->Init.BaudRate = BQ79614_BAUDRATE_WAKEUP;
    if (HAL_UART_Init(huart) != HAL_OK) return HAL_ERROR;
    uint8_t wake_data = 0x00;
    if (HAL_UART_Transmit(dev->huart, &wake_data, 1, 1000) != HAL_OK) return HAL_ERROR;
    vTaskDelay(pdMS_TO_TICKS(3));
    huart->Init.BaudRate = BQ79614_DEFAULT_BAUDRATE;
    if (HAL_UART_Init(huart) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef BQ79614_Write(BQ79614_Device *dev, BQ79614_ReqType req_type, uint8_t *data, size_t size, uint16_t reg_address, uint8_t address) {
    if (size > 8) return HAL_ERROR;
    if (xSemaphoreTake(dev->semaphore, pdMS_TO_TICKS(1000)) != pdTRUE) return HAL_BUSY;
    dev->out[0] = BQ79614_InitByteWrite(dev, req_type, size);
    dev->out[1] = address;
    dev->out[2] = (reg_address >> 8) & 0xFF;
    dev->out[3] = reg_address & 0xFF;
    memcpy(&dev->out[4], data, size);
    uint16_t crc = CRC16_Fast(dev->out, 4 + size);
    dev->out[4 + size] = (crc >> 8) & 0xFF;
    dev->out[5 + size] = crc & 0xFF;
    if (HAL_UART_Transmit(dev->huart, dev->out, 6 + size, 1000) != HAL_OK) {
        xSemaphoreGive(dev->semaphore);
        return HAL_ERROR;
    }
    xSemaphoreGive(dev->semaphore);
    return HAL_OK;
}

HAL_StatusTypeDef BQ79614_Read(BQ79614_Device *dev, BQ79614_ReqType req_type, uint8_t *data, size_t count, uint16_t reg_address, uint8_t address) {
    if (xSemaphoreTake(dev->semaphore, pdMS_TO_TICKS(1000)) != pdTRUE) return HAL_BUSY;
    size_t read_count = (req_type == BQ79614_REQ_SINGLE) ? 1 : BQ79614_STACK_SIZE;
    size_t total_bytes = count * read_count;
    dev->out[0] = BQ79614_InitByteRead(dev, req_type);
    dev->out[1] = address;
    dev->out[2] = (reg_address >> 8) & 0xFF;
    dev->out[3] = reg_address & 0xFF;
    dev->out[4] = (count - 1) & 0xFF;
    uint16_t crc = CRC16_Fast(dev->out, 5);
    dev->out[5] = (crc >> 8) & 0xFF;
    dev->out[6] = crc & 0xFF;
    if (HAL_UART_Transmit(dev->huart, dev->out, 7, 1000) != HAL_OK) {
        xSemaphoreGive(dev->semaphore);
        return HAL_ERROR;
    }
    dev->read_size = total_bytes + 2 * read_count; // Data + CRC per device
    if (HAL_UART_Receive(dev->huart, dev->in, dev->read_size, 1000) != HAL_OK) {
        xSemaphoreGive(dev->semaphore);
        return HAL_TIMEOUT;
    }
    for (size_t i = 0; i < read_count; i++) {
        uint8_t *chunk = &dev->in[i * (count + 2)];
        uint16_t received_crc = (chunk[count] << 8) | chunk[count + 1];
        if (CRC16_Fast(chunk, count) != received_crc) {
            xSemaphoreGive(dev->semaphore);
            return HAL_ERROR;
        }
        memcpy(&data[i * count], chunk, count);
    }
    xSemaphoreGive(dev->semaphore);
    return HAL_OK;
}