#ifndef RADAR_TASK_H
#define RADAR_TASK_H

#include "presence_detection.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* Shared Status */
typedef struct {
    presence_state_t state;
    float32_t range;
} radar_status_t;

/* Task Prototypes */
void RadarTask(void *pvParameters);
cy_rslt_t init_dual_radars(void);

/* Accessors for status */
void get_radar_status(radar_status_t *r1, radar_status_t *r2);

/* Global Synchronization Objects (defined in radar_task.c) */
extern SemaphoreHandle_t xRadar1Sem;
extern SemaphoreHandle_t xRadar2Sem;
extern SemaphoreHandle_t xSpiMutex;
extern SemaphoreHandle_t xStatusMutex;

#endif /* RADAR_TASK_H */
