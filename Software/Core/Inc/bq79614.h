#ifndef BQ79614_H
#define BQ79614_H

#include "main.h"
#include "freertos.h"
#include "semphr.h"

#define BQ79614_STACK_SIZE 2
#define BQ79614_CELL_COUNT 14
#define BQ79614_TEMP_COUNT 8
#define BQ79614_DEFAULT_BAUDRATE 1000000
#define BQ79614_T_WAKEUP 2600
#define BQ79614_T_RX_TIMEOUT 500
#define BQ79614_BAUDRATE_WAKEUP ((uint32_t)(1000000.0 / (double)BQ79614_T_WAKEUP * 6.0))
#define BQ79614_RX_TIMEOUT_BAUD ((uint32_t)((double)BQ79614_T_RX_TIMEOUT * (double)BQ79614_DEFAULT_BAUDRATE / 1000000.0))
#define BQ79614_V_LSB_ADC 190.73e-6

typedef enum {
    BQ79614_REQ_SINGLE = 0b00,
    BQ79614_REQ_STACK = 0b01,
    BQ79614_REQ_BROADCAST = 0b10
} BQ79614_ReqType;

typedef enum {
    BQ79614_SCAN_STOP = 0,
    BQ79614_SCAN_ROUND_ROBIN = 1,
    BQ79614_SCAN_ONCE = 2,
    BQ79614_SCAN_SINGLE_CHANNEL = 3
} BQ79614_ScanMode;

typedef enum {
    BQ79614_GPIO_HIGHZ = 0,
    BQ79614_GPIO_ADC_OTUT = 1
} BQ79614_GpioMode;

typedef struct {
    uint8_t addr_wr : 1;
    uint8_t soft_reset : 1;
    uint8_t goto_sleep : 1;
    uint8_t goto_shutdown : 1;
    uint8_t send_slptoact : 1;
    uint8_t send_wake : 1;
    uint8_t send_shutdown : 1;
    uint8_t dir_sel : 1;
} BQ79614_Control1;

typedef struct {
    BQ79614_ScanMode main_mode : 2;
    uint8_t main_go : 1;
    uint8_t lpf_cell_en : 1;
    uint8_t lpf_bb_en : 1;
    uint8_t reserved : 3;
} BQ79614_AdcCtrl1;

typedef struct {
    BQ79614_ScanMode ovuv_mode : 2;
    uint8_t ovuv_go : 1;
    uint8_t ovuv_lock : 4;
    uint8_t vcbdone_thr_lock : 1;
} BQ79614_OVUVCtrl;

typedef struct {
    BQ79614_GpioMode gpio1 : 3;
    BQ79614_GpioMode gpio2 : 3;
    uint8_t spi_en : 1;
    uint8_t fault_in_en : 1;
} BQ79614_GPIOConf1;

typedef struct {
    BQ79614_GpioMode gpio3 : 3;
    BQ79614_GpioMode gpio4 : 3;
    uint8_t reserved : 1;
    uint8_t spare : 1;
} BQ79614_GPIOConf2;

typedef struct {
    BQ79614_GpioMode gpio5 : 3;
    BQ79614_GpioMode gpio6 : 3;
    uint8_t spare : 2;
} BQ79614_GPIOConf3;

typedef struct {
    BQ79614_GpioMode gpio7 : 3;
    BQ79614_GpioMode gpio8 : 3;
    uint8_t spare : 2;
} BQ79614_GPIOConf4;

typedef struct {
    uint8_t ot_thr : 5;
    uint8_t ut_thr : 3;
} BQ79614_OTUTThresh;

typedef struct {
    BQ79614_ScanMode otut_mode : 2;
    uint8_t otut_go : 1;
    uint8_t otut_lock : 3;
    uint8_t vcbdone_thr_lock : 1;
    uint8_t reserved : 1;
} BQ79614_OTUTCtrl;

#define BQ79614_REG_CONTROL1 0x0309
#define BQ79614_REG_ADCCTRL1 0x030D
#define BQ79614_REG_OVUVCTRL 0x032C
#define BQ79614_REG_GPIOCONF1 0x000E
#define BQ79614_REG_GPIOCONF2 0x000F
#define BQ79614_REG_GPIOCONF3 0x0010
#define BQ79614_REG_GPIOCONF4 0x0011
#define BQ79614_REG_OTUTTHRESH 0x000B
#define BQ79614_REG_OTUTCTRL 0x032D
#define BQ79614_REG_CELL1_VOLTAGE 0x0014
#define BQ79614_REG_GPIO1 0x0032

typedef struct {
    int ovuv[14];
    int otut[8];
} BQ79614_StackDeviceStatus;

typedef struct {
    float voltages[14];
    float temperatures[8];
} BQ79614_StackDeviceData;

typedef struct {
    UART_HandleTypeDef *huart;
    SemaphoreHandle_t semaphore;
    size_t read_size;
    uint8_t out[256];
    uint8_t in[256];
    BQ79614_StackDeviceStatus stack_device_status[2];
    BQ79614_StackDeviceData stack_device_data[2];
} BQ79614_Device;

HAL_StatusTypeDef BQ79614_Init(BQ79614_Device *dev, UART_HandleTypeDef *huart);
HAL_StatusTypeDef BQ79614_InitUART(BQ79614_Device *dev);
HAL_StatusTypeDef BQ79614_InitStack(BQ79614_Device *dev);
HAL_StatusTypeDef BQ79614_InitVoltageMeasurement(BQ79614_Device *dev);
HAL_StatusTypeDef BQ79614_InitOVUV(BQ79614_Device *dev, uint32_t undervoltage, uint32_t overvoltage);
HAL_StatusTypeDef BQ79614_InitTemperatureMeasurements(BQ79614_Device *dev);
HAL_StatusTypeDef BQ79614_InitOTUT(BQ79614_Device *dev, uint8_t undertemperature, uint8_t overtemperature);
HAL_StatusTypeDef BQ79614_StartMeasurements(BQ79614_Device *dev);
HAL_StatusTypeDef BQ79614_UpdateStatus(BQ79614_Device *dev);
HAL_StatusTypeDef BQ79614_UpdateData(BQ79614_Device *dev);
HAL_StatusTypeDef BQ79614_WakeUp(BQ79614_Device *dev);
HAL_StatusTypeDef BQ79614_Write(BQ79614_Device *dev, BQ79614_ReqType req_type, uint8_t *data, size_t size, uint16_t reg_address, uint8_t address);
HAL_StatusTypeDef BQ79614_Read(BQ79614_Device *dev, BQ79614_ReqType req_type, uint8_t *data, size_t count, uint16_t reg_address, uint8_t address);

#endif
