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

/* RX2-RX3 antenna spacing in units of wavelength. BGT60TR13C standard PCB
 * layout uses λ/2 spacing (0.5). Adjust if your board differs. */
#define RADAR_RX_SPACING_LAMBDA             (0.5f)

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
static float32_t avg_rx2[PRESENCE_NUM_SAMPLES];
static float32_t avg_rx3[PRESENCE_NUM_SAMPLES];
static float32_t rx3_windowed[PRESENCE_NUM_SAMPLES];
static float32_t rx3_fft_output[PRESENCE_NUM_SAMPLES * 2];

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
        status->distance_m = 0.0f;
        status->angle_deg = 0.0f;
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
        status->distance_m = 0.0f;
        status->angle_deg = 0.0f;
        xSemaphoreGive(xStatusMutex);
        return;
    }
    
    /* Normalize raw ADC samples to float [0, 1] */
    uint16_t *bgt60_buffer_ptr = samples;
    float32_t *frame_ptr = &frame[0];
    for (int32_t sample = 0; sample < sizeof(samples)/sizeof(samples[0]); ++sample)
    {
        *frame_ptr++ = ((float32_t)(*bgt60_buffer_ptr++) / 4096.0F);
    }

    /* FIFO layout for enabled RX=[2,3]: per chirp, [RX2 samples][RX3 samples].
       De-interleave and average chirps per channel. */
    arm_fill_f32(0, avg_rx2, PRESENCE_NUM_SAMPLES);
    arm_fill_f32(0, avg_rx3, PRESENCE_NUM_SAMPLES);

    for (int chirp = 0; chirp < PRESENCE_NUM_CHIRPS; chirp++)
    {
        int32_t base = chirp * XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS * PRESENCE_NUM_SAMPLES;
        arm_add_f32(avg_rx2, &frame[base + 0 * PRESENCE_NUM_SAMPLES], avg_rx2, PRESENCE_NUM_SAMPLES);
        arm_add_f32(avg_rx3, &frame[base + 1 * PRESENCE_NUM_SAMPLES], avg_rx3, PRESENCE_NUM_SAMPLES);
    }

    arm_scale_f32(avg_rx2, 1.0f / PRESENCE_NUM_CHIRPS, avg_rx2, PRESENCE_NUM_SAMPLES);
    arm_scale_f32(avg_rx3, 1.0f / PRESENCE_NUM_CHIRPS, avg_rx3, PRESENCE_NUM_SAMPLES);

    /* Run presence detection on RX2 (primary channel). After return,
       ctx->macro_fft_buffer[] holds the complex RX2 range FFT. */
    presence_result_t result = presence_process_frame(ctx, avg_rx2, time_ms);

    /* Compute azimuth angle from RX2/RX3 phase difference at the peak range bin.
       Only valid when a target was detected (range_bin > 0). */
    float32_t angle_deg = 0.0f;
    if (result.range_bin > 0) {
        for (int32_t i = 0; i < PRESENCE_NUM_SAMPLES; i++) {
            rx3_windowed[i] = avg_rx3[i] * ctx->hamming_window[i];
        }
        arm_rfft_fast_f32(&ctx->rfft_instance, rx3_windowed, rx3_fft_output, 0);

        int32_t k = result.range_bin;
        float32_t rx2_re = crealf(ctx->macro_fft_buffer[k]);
        float32_t rx2_im = cimagf(ctx->macro_fft_buffer[k]);
        float32_t rx3_re = rx3_fft_output[2 * k];
        float32_t rx3_im = rx3_fft_output[2 * k + 1];

        /* Cross-product form of phase difference: arg(RX3 * conj(RX2)).
           More numerically robust than subtracting two atan2 calls. */
        float32_t cross_re = rx3_re * rx2_re + rx3_im * rx2_im;
        float32_t cross_im = rx3_im * rx2_re - rx3_re * rx2_im;
        float32_t delta_phi = atan2f(cross_im, cross_re);

        float32_t sin_theta = delta_phi / (2.0f * PI * RADAR_RX_SPACING_LAMBDA);
        if (sin_theta > 1.0f) sin_theta = 1.0f;
        else if (sin_theta < -1.0f) sin_theta = -1.0f;
        angle_deg = asinf(sin_theta) * (180.0f / PI);
    }

    /* Update shared status */
    xSemaphoreTake(xStatusMutex, portMAX_DELAY);
    status->state = result.state;
    status->range = result.range_m;
    status->distance_m = result.range_m;
    status->angle_deg = angle_deg;
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

    const uint32_t frame_period_ms = (uint32_t)(XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S * 1000.0f);
    const TickType_t period_ticks = pdMS_TO_TICKS(frame_period_ms);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t system_time_ms = 0;

    for(;;) {
        system_time_ms += frame_period_ms;

        xSemaphoreTake(xSpiMutex, portMAX_DELAY);
        cyhal_gpio_write(LED1, 1);
        run_presence_detection(&sensor1, xRadar1Sem, &presence_ctx_1, &status_r1, system_time_ms);
        xSemaphoreGive(xSpiMutex);

        xSemaphoreTake(xSpiMutex, portMAX_DELAY);
        run_presence_detection(&sensor2, xRadar2Sem, &presence_ctx_2, &status_r2, system_time_ms);
        cyhal_gpio_write(LED1, 0);
        xSemaphoreGive(xSpiMutex);

        vTaskDelayUntil(&last_wake, period_ticks);
    }
}
