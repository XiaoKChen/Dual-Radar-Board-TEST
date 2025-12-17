#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "radar_task.h"
#include "usb_logging.h"

#define LED1 P10_3
#define USB_CONFIG_DELAY (50U)

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void SystemTask(void *pvParameters);
void PrintTask(void *pvParameters);

/*******************************************************************************
* Main Function
********************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(P7_1, P7_0, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Create Synchronization Objects */
    xRadar1Sem = xSemaphoreCreateBinary();
    xRadar2Sem = xSemaphoreCreateBinary();
    xSpiMutex = xSemaphoreCreateMutex();
    xStatusMutex = xSemaphoreCreateMutex();
    
    /* Create System Init Task */
    xTaskCreate(SystemTask, "System", 1024, NULL, 3, NULL);
    
    /* Start Scheduler */
    /* Note: status_printf not available until USB init in SystemTask, 
       but we can't use it before scheduler starts anyway if it uses mutexes/delays.
       Standard printf goes to UART via retarget-io */
    printf("Starting FreeRTOS Scheduler...\r\n");
    vTaskStartScheduler();
    
    for(;;) {}
}

/* Task Definitions */
void SystemTask(void *pvParameters) {
    (void)pvParameters;
    cy_rslt_t result;

    /* Initialize the User LED */
    cyhal_gpio_init(LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);

    /* Initialize USB and Logging */
    init_usb_logging();

    /* Wait for USB configuration */
    wait_for_usb_configured();

    status_printf("USB configured.\r\n");
    status_printf("Dual Radar Presence Detection\r\n");
    status_printf("Range Resolution: %.3f m\r\n", PRESENCE_RANGE_RESOLUTION);
    status_printf("Max Range Bin: %d (%.2f m)\r\n", PRESENCE_MAX_RANGE_BIN, 
                  PRESENCE_MAX_RANGE_BIN * PRESENCE_RANGE_RESOLUTION);

    /* Initialize dual radars */
    result = init_dual_radars();
    if (result != CY_RSLT_SUCCESS)
    {
        status_printf("Radar initialization failed. Error: 0x%08lX\r\n", (unsigned long)result);
        for(;;) {}
    }
    
    status_printf("Dual Radars and Presence Detection Initialized.\r\n");
    status_printf("Starting continuous presence detection...\r\n");
    status_printf("Macro Threshold: %.2f, Micro Threshold: %.2f\r\n",
                  PRESENCE_MACRO_THRESHOLD, PRESENCE_MICRO_THRESHOLD);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Create Tasks */
    xTaskCreate(RadarTask, "Radar", 4096, NULL, 2, NULL);
    xTaskCreate(PrintTask, "Print", 1024, NULL, 1, NULL);
    
    /* Delete self */
    vTaskDelete(NULL);
}

void PrintTask(void *pvParameters) {
    (void)pvParameters;
    radar_status_t r1, r2;

    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        get_radar_status(&r1, &r2);
        
        status_printf("--------------------------------------------------\r\n");
        if (r1.state == PRESENCE_STATE_ABSENCE) {
            status_printf("[R1] No human\r\n");
        } else {
            status_printf("[R1] Human detect, Range: %.2f m\r\n", r1.range);
        }
        
        if (r2.state == PRESENCE_STATE_ABSENCE) {
            status_printf("[R2] No human\r\n");
        } else {
            status_printf("[R2] Human detect, Range: %.2f m\r\n", r2.range);
        }
    }
}
