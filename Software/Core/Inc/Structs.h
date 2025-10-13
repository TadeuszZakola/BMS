#ifndef STRUCTS_H
#define STRUCTS_H

// Configuration constants
#define N_DEVICES            3
#define N_CELLS_PER_DEVICE   13
#define N_TEMPS_PER_DEVICE   8

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "structs.h"      // for bq79600_t, open_bq79600_instance, bq79600_rx_callback
#include "usb_device.h"   // for USB-related buffers
#include <math.h>         // for logf(), expf()
#include <string.h>       // for memcpy()

/* USER CODE BEGIN Includes */

// Structures ---------------------------------------------------------------

typedef struct {
    int   temperature[N_TEMPS_PER_DEVICE]; // degC
    float vcells[N_CELLS_PER_DEVICE];      // mV
    float t_ref;
    float dietemp;                         // degC
    int   timestamp;
    uint8_t DEV_STAT_RAW;
    uint8_t UV_RAW_1;
    uint8_t UV_RAW_2;
    uint8_t OV_RAW_1;
    uint8_t OV_RAW_2;
    uint8_t UT_RAW;
    uint8_t OT_RAW;
    uint8_t BAL_STAT_RAW;
    uint8_t CB_COMPLETE1_RAW;
    uint8_t CB_COMPLETE2_RAW;
} module_t;

// Declare here, define in a .c file
extern module_t modules[N_DEVICES - 1];

typedef struct {
    int MAIN_ADC_RUN;
    int AUX_ADC_RUN;
    int CS_RUN;
    int OVUV_RUN;
    int OTUT_RUN;
} DEV_STATS;

typedef struct {
    int BQ_Number;
    int BQ_Overvoltage_Error;
    int BQ_Undervoltage_Error;
    int BQ_Autoadressing_Error;
    int BQ_Communication_Error;
    int OV_ERROR[N_CELLS_PER_DEVICE];
    int UV_ERROR[N_CELLS_PER_DEVICE];
    int OT_ERROR[N_TEMPS_PER_DEVICE];
    int UT_ERROR[N_TEMPS_PER_DEVICE];


    int INVALID_CBCONF;
    int OT_PAUSE_DET;
    int CB_INPAUSE;
    int MB_RUN;
    int CB_RUN;
    int ABORTFLT;
    int MB_DONE;
    int CB_DONE;
    DEV_STATS Device_Stat;
    int   Bq_Timestamp;
    float Bq_Voltages[N_CELLS_PER_DEVICE];   // mV
    int   Bq_Temperatures[N_TEMPS_PER_DEVICE]; // degC
    int  CB_Done[N_CELLS_PER_DEVICE]; // if cell ballancing is done on this cell then = 1 else 0
    float T_ref;                               // mV
    float dietemp;                             // degC
} BQ_Data;

typedef struct {
    BQ_Data Device[N_DEVICES - 1];
} BQ_Data_Combined;


/* USER CODE END Includes */

#ifdef __cplusplus
}
#endif

#endif /* STRUCTS_H */

