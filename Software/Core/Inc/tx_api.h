#ifndef TX_API_H
#define TX_API_H

#include "main.h"

#define TX_SUCCESS 0
#define TX_SEMAPHORE uint32_t

#define tx_semaphore_create(sem, name, count) (*(sem) = (count), HAL_OK)
#define tx_semaphore_get(sem, ticks) (*(sem) > 0 ? (--*(sem), HAL_OK) : HAL_TIMEOUT)
#define tx_semaphore_put(sem) (++*(sem), HAL_OK)
#define tx_thread_sleep(ticks) HAL_Delay(ticks)

#endif /* TX_API_H */
