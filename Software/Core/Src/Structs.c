#include "structs.h"


/* -------------------------------------------------------------------------- */
/* Global Instances                                                           */
/* -------------------------------------------------------------------------- */

// Global array of modules
module_t modules[N_DEVICES - 1] = {0};

// Optional: global combined data holder
BQ_Data_Combined bq_data = {0};


/* -------------------------------------------------------------------------- */
/* Helper Functions                                                           */
/* -------------------------------------------------------------------------- */

// Convert raw bytes to float using endian swap + sign extension
float raw_to_float(void *raw)
{
    return (float)(int16_t)(
        (((*(uint16_t *)raw) & 0x00FF) << 8) |
        (((*(uint16_t *)raw) & 0xFF00) >> 8)
    );
}
extern UART_HandleTypeDef huart4;
// UART RX event callback (HAL weak function override)

