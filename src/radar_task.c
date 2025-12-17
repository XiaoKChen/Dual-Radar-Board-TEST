#define XENSIV_BGT60TRXX_CONF_IMPL
#include "radar_task.h"
#include "usb_logging.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "xensiv_bgt60trxx_mtb.h"

/* Radar Pins */
#define PIN_RADAR1_CS       Radar_CS_1
#define PIN_RADAR1_IRQ      Radar_IRQ_1
#define PIN_RADAR2_CS       Radar_CS_2
#define PIN_RADAR2_IRQ      Radar_IRQ_2
#define PIN_RADAR_SPI_SCLK  Radar_SPI_CLK
#define PIN_RADAR_SPI_MOSI  Radar_SPI_MOSI
#define PIN_RADAR_SPI_MISO  Radar_SPI_MISO
#define PIN_RADAR_RST       Radar_RST

#define XENSIV_BGT60TRXX_SPI_FREQUENCY      (12000000UL)
#define LED1 P10_3

/* Global variables */
static cyhal_spi_t cyhal_spi;
static xensiv_bgt60trxx_mtb_t sensor1;
static xensiv_bgt60trxx_mtb_t sensor2;

/* FreeRTOS Synchronization Objects */
SemaphoreHandle_t xRadar1Sem;
SemaphoreHandle_t xRadar2Sem;
SemaphoreHandle_t xSpiMutex;
SemaphoreHandle_t xStatusMutex;

/* Shared Status */
static radar_status_t status_r1;
static radar_status_t status_r2;

/* Buffers */
static uint16_t samples[PRESENCE_NUM_SAMPLES * PRESENCE_NUM_CHIRPS * XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS];
static float32_t frame[PRESENCE_NUM_SAMPLES * PRESENCE_NUM_CHIRPS * XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS];
static float32_t avg_chirp[PRESENCE_NUM_SAMPLES];

/* Presence detection contexts */
static presence_context_t presence_ctx_1;
static presence_context_t presence_ctx_2;

/* Interrupt handlers */
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
void radar1_irq_handler(void *args, cyhal_gpio_event_t event)
#else
void radar1_irq_handler(void *args, cyhal_gpio_irq_event_t event)
#endif
{
    CY_UNUSED_PARAMETER(args);
    CY_UNUSED_PARAMETER(event);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xRadar1Sem != NULL) {
        xSemaphoreGiveFromISR(xRadar1Sem, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
void radar2_irq_handler(void *args, cyhal_gpio_event_t event)
#else
void radar2_irq_handler(void *args, cyhal_gpio_irq_event_t event)
#endif
{
    CY_UNUSED_PARAMETER(args);
    CY_UNUSED_PARAMETER(event);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xRadar2Sem != NULL) {
        xSemaphoreGiveFromISR(xRadar2Sem, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void get_radar_status(radar_status_t *r1, radar_status_t *r2) {
    xSemaphoreTake(xStatusMutex, portMAX_DELAY);
    *r1 = status_r1;
    *r2 = status_r2;
    xSemaphoreGive(xStatusMutex);
}

static void run_presence_detection(xensiv_bgt60trxx_mtb_t *sensor_obj, 
                                   SemaphoreHandle_t sem, 
                                   presence_context_t *ctx,
                                   radar_status_t *status,
                                   uint32_t time_ms) 
{
    /* Start frame acquisition */
    int32_t res = xensiv_bgt60trxx_start_frame(&sensor_obj->dev, true);
    if (res != XENSIV_BGT60TRXX_STATUS_OK) {
        status_printf("Start frame failed: %ld\r\n", (long)res);
        return;
    }
    
    /* Wait for interrupt with timeout */
    if (xSemaphoreTake(sem, pdMS_TO_TICKS(3000)) != pdTRUE) {
        /* Timeout */
        status_printf("Timeout waiting for interrupt\r\n");
        xensiv_bgt60trxx_start_frame(&sensor_obj->dev, false);
        
        /* Update status to indicate failure/absence */
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        status->state = PRESENCE_STATE_ABSENCE;
        status->range = 0.0f;
        xSemaphoreGive(xStatusMutex);
        return;
    }
    
    /* Read FIFO data */
    int32_t read_res = xensiv_bgt60trxx_get_fifo_data(&sensor_obj->dev, samples, sizeof(samples)/sizeof(samples[0]));
    
    /* Stop frame acquisition */
    xensiv_bgt60trxx_start_frame(&sensor_obj->dev, false);

    if (read_res != XENSIV_BGT60TRXX_STATUS_OK) {
        status_printf("FIFO read failed: %ld\r\n", (long)read_res);
        
        /* Update status to indicate failure/absence */
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
        status->state = PRESENCE_STATE_ABSENCE;
        status->range = 0.0f;
        xSemaphoreGive(xStatusMutex);
        return;
    }
    
    /* Data preprocessing */
    uint16_t *bgt60_buffer_ptr = samples;
    float32_t *frame_ptr = &frame[0];
    for (int32_t sample = 0; sample < sizeof(samples)/sizeof(samples[0]); ++sample)
    {
        *frame_ptr++ = ((float32_t)(*bgt60_buffer_ptr++) / 4096.0F);
    }
    
    /* Calculate average chirp */
    arm_fill_f32(0, avg_chirp, PRESENCE_NUM_SAMPLES);
    
    for (int chirp = 0; chirp < PRESENCE_NUM_CHIRPS; chirp++)
    {
        arm_add_f32(avg_chirp, &frame[PRESENCE_NUM_SAMPLES * chirp], avg_chirp, PRESENCE_NUM_SAMPLES);
    }
    
    arm_scale_f32(avg_chirp, 1.0f / PRESENCE_NUM_CHIRPS, avg_chirp, PRESENCE_NUM_SAMPLES);
    
    /* Process frame */
    presence_result_t result = presence_process_frame(ctx, avg_chirp, time_ms);
    
    /* Update shared status */
    xSemaphoreTake(xStatusMutex, portMAX_DELAY);
    status->state = result.state;
    status->range = result.range_m;
    xSemaphoreGive(xStatusMutex);
}

cy_rslt_t init_dual_radars(void) {
    cy_rslt_t rslt;
    cy_rslt_t result;

    /* Initialize SPI */
    status_printf("Initializing SPI...\r\n");
    result = cyhal_spi_init(&cyhal_spi,
                            PIN_RADAR_SPI_MOSI,
                            PIN_RADAR_SPI_MISO,
                            PIN_RADAR_SPI_SCLK,
                            NC,
                            NULL,
                            8,
                            CYHAL_SPI_MODE_00_MSB,
                            false);
    
    if (result != CY_RSLT_SUCCESS)
    {
        status_printf("SPI initialization failed. Error: 0x%08lX\r\n", (unsigned long)result);
        return result;
    }
    
    /* Reduce drive strength */
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_RADAR_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_RADAR_SPI_MOSI), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_RADAR_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_RADAR_SPI_MOSI), CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_RADAR_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_RADAR_SPI_SCLK), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_RADAR_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_RADAR_SPI_SCLK), CY_GPIO_DRIVE_1_8);

    result = cyhal_spi_set_frequency(&cyhal_spi, XENSIV_BGT60TRXX_SPI_FREQUENCY);
    if (result != CY_RSLT_SUCCESS)
    {
        status_printf("SPI frequency setting failed.\r\n");
        return result;
    }
    
    /* Initialize GPIOs */
    cyhal_gpio_init(PIN_RADAR1_CS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
    cyhal_gpio_init(PIN_RADAR2_CS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
    cyhal_gpio_init(PIN_RADAR_RST, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);

    /* Hard Reset */
    cyhal_gpio_write(PIN_RADAR_RST, 1);
    cyhal_system_delay_ms(1);
    cyhal_gpio_write(PIN_RADAR_RST, 0);
    cyhal_system_delay_ms(1);
    cyhal_gpio_write(PIN_RADAR_RST, 1);
    cyhal_system_delay_ms(10);

    status_printf("Initializing Radar 1...\r\n");
    cyhal_system_delay_ms(50);
    
    cyhal_gpio_write(PIN_RADAR2_CS, 1);
    
    /* Setup Sensor 1 */
    sensor1.iface.spi = &cyhal_spi;
    sensor1.iface.selpin = PIN_RADAR1_CS;
    sensor1.iface.rstpin = PIN_RADAR_RST;
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
    sensor1.iface.irqpin.pin = NC;
#else
    sensor1.iface.irqpin = NC;
#endif
    
    rslt = xensiv_bgt60trxx_init(&sensor1.dev, &sensor1.iface, false);
    if (rslt == XENSIV_BGT60TRXX_STATUS_OK) {
        rslt = xensiv_bgt60trxx_config(&sensor1.dev, register_list, XENSIV_BGT60TRXX_CONF_NUM_REGS);
    }
    if (rslt == XENSIV_BGT60TRXX_STATUS_OK) {
        rslt = xensiv_bgt60trxx_mtb_interrupt_init(&sensor1, PRESENCE_NUM_SAMPLES * PRESENCE_NUM_CHIRPS * XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS, PIN_RADAR1_IRQ, CYHAL_ISR_PRIORITY_DEFAULT, radar1_irq_handler, NULL);
    }
    
    if (rslt != CY_RSLT_SUCCESS) {
        status_printf("Radar 1 init failed: 0x%08lX\r\n", (unsigned long)rslt);
        return rslt;
    }
    
    status_printf("Radar 1 initialized successfully\r\n");
    cyhal_system_delay_ms(50);
    
    cyhal_gpio_write(PIN_RADAR1_CS, 1);
    cyhal_system_delay_ms(10);
    
    status_printf("Initializing Radar 2...\r\n");
    cyhal_system_delay_ms(50);

    /* Setup Sensor 2 */
    sensor2.iface.spi = &cyhal_spi;
    sensor2.iface.selpin = PIN_RADAR2_CS;
    sensor2.iface.rstpin = PIN_RADAR_RST;
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
    sensor2.iface.irqpin.pin = NC;
#else
    sensor2.iface.irqpin = NC;
#endif

    rslt = xensiv_bgt60trxx_init(&sensor2.dev, &sensor2.iface, false);
    if (rslt == XENSIV_BGT60TRXX_STATUS_OK) {
        rslt = xensiv_bgt60trxx_config(&sensor2.dev, register_list, XENSIV_BGT60TRXX_CONF_NUM_REGS);
    }
    if (rslt == XENSIV_BGT60TRXX_STATUS_OK) {
        rslt = xensiv_bgt60trxx_mtb_interrupt_init(&sensor2, PRESENCE_NUM_SAMPLES * PRESENCE_NUM_CHIRPS * XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS, PIN_RADAR2_IRQ, CYHAL_ISR_PRIORITY_DEFAULT, radar2_irq_handler, NULL);
    }
    
    if (rslt != CY_RSLT_SUCCESS) {
        status_printf("Radar 2 init failed: 0x%08lX\r\n", (unsigned long)rslt);
        return rslt;
    }
    
    status_printf("Radar 2 initialized successfully\r\n");
    cyhal_system_delay_ms(50);
    
    cyhal_gpio_write(PIN_RADAR1_CS, 1);
    cyhal_gpio_write(PIN_RADAR2_CS, 1);

    /* Initialize presence detection */
    status_printf("Initializing Presence Detection...\r\n");
    presence_init(&presence_ctx_1);
    presence_init(&presence_ctx_2);

    return rslt;
}

void RadarTask(void *pvParameters) {
    (void)pvParameters;
    
    uint32_t frame_period_ms = (uint32_t)(XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S * 1000.0f);
    uint32_t system_time_ms = 0;

    for(;;) {
        system_time_ms += frame_period_ms;
        
        xSemaphoreTake(xSpiMutex, portMAX_DELAY);
        cyhal_gpio_write(LED1, 1);
        run_presence_detection(&sensor1, xRadar1Sem, &presence_ctx_1, &status_r1, system_time_ms);
        xSemaphoreGive(xSpiMutex);
        
        vTaskDelay(pdMS_TO_TICKS(50));
        
        xSemaphoreTake(xSpiMutex, portMAX_DELAY);
        run_presence_detection(&sensor2, xRadar2Sem, &presence_ctx_2, &status_r2, system_time_ms);
        cyhal_gpio_write(LED1, 0);
        xSemaphoreGive(xSpiMutex);
        
        vTaskDelay(pdMS_TO_TICKS(frame_period_ms));
    }
}
