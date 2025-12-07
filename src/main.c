#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "USB.h"
#include "USB_CDC.h"
#include <stdio.h>
#include <inttypes.h>
#include <limits.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#define ARM_MATH_CM4
#include "arm_math.h"
#include "dsp/transform_functions.h"
#include "dsp/complex_math_functions.h"
#include "dsp/fast_math_functions.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

#include "xensiv_bgt60trxx_mtb.h"

#define XENSIV_BGT60TRXX_CONF_IMPL
#include "presence_radar_settings.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define USB_CONFIG_DELAY          (50U) /* In milliseconds */

#define RADAR_1

#if defined(RADAR_1)
    /* sensor SPI interface */
    #define PIN_XENSIV_BGT60TRXX_SPI_SCLK       Radar_SPI_CLK
    #define PIN_XENSIV_BGT60TRXX_SPI_MOSI       Radar_SPI_MOSI
    #define PIN_XENSIV_BGT60TRXX_SPI_MISO       Radar_SPI_MISO
    #define PIN_XENSIV_BGT60TRXX_SPI_CSN        Radar_CS_1

    /* sensor interrupt output pin */
    #define PIN_XENSIV_BGT60TRXX_IRQ            Radar_IRQ_1
    /* sensor HW reset pin */
    #define PIN_XENSIV_BGT60TRXX_RSTN           Radar_RST
#endif

#if defined(RADAR_2)
    /* sensor SPI interface */
    #define PIN_XENSIV_BGT60TRXX_SPI_SCLK       Radar_SPI_CLK
    #define PIN_XENSIV_BGT60TRXX_SPI_MOSI       Radar_SPI_MOSI
    #define PIN_XENSIV_BGT60TRXX_SPI_MISO       Radar_SPI_MISO
    #define PIN_XENSIV_BGT60TRXX_SPI_CSN        Radar_CS_1

    /* sensor interrupt output pin */
    #define PIN_XENSIV_BGT60TRXX_IRQ            Radar_IRQ_1
    /* sensor HW reset pin */
    #define PIN_XENSIV_BGT60TRXX_RSTN           Radar_RST
#endif
/* enable 1V8 LDO on radar wingboard*/
// #define PIN_XENSIV_BGT60TRXX_LDO_EN         NO_LDO_EN_PIN_LDO_ALWAYS_ON

#define XENSIV_BGT60TRXX_SPI_FREQUENCY      (24000000UL)

#define NUM_SAMPLES_PER_FRAME               (XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS *\
                                             XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME *\
                                             XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP)

#define BINARY_FRAME_HEADER_VERSION         (1U)
#define BINARY_FRAME_SAMPLE_SIZE_BYTES      ((uint16_t)sizeof(uint16_t))

#define LED1 P10_3
#define LED2 P10_2

void turn_on_leds(void)
{
    cyhal_gpio_init(LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
    cyhal_gpio_init(LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);

    // cyhal_gpio_init(PIN_XENSIV_BGT60TRXX_SPI_SCLK, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULL_NONE, 1);
    // cyhal_gpio_init(PIN_XENSIV_BGT60TRXX_SPI_MOSI, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULL_NONE, 1);
    // cyhal_gpio_init(PIN_XENSIV_BGT60TRXX_SPI_MISO, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULL_NONE, 1);
    cyhal_gpio_init(PIN_XENSIV_BGT60TRXX_SPI_CSN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULL_NONE, 1);
    // cyhal_gpio_init(PIN_XENSIV_BGT60TRXX_IRQ, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULL_NONE, 1);
    // cyhal_gpio_init(PIN_XENSIV_BGT60TRXX_RSTN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULL_NONE, 1);

}

typedef struct __attribute__((packed))
{
    uint8_t magic[4];
    uint16_t version;
    uint16_t sample_size_bytes;
    uint32_t frame_index;
    uint32_t sample_count;
} binary_frame_header_t;

/*******************************************************************************
* Global variables
********************************************************************************/
static USB_CDC_HANDLE usb_cdcHandle;

static cyhal_spi_t cyhal_spi;
static xensiv_bgt60trxx_mtb_t sensor;
static volatile bool data_available = false;
static bool capture_enabled = false;
static bool frame_limit_enabled = false;
static uint32_t frame_limit_total = 0U;
static uint32_t frame_limit_sent = 0U;
static bool binary_stream_active = false;
static bool radar_initialized = false;
static bool test_mode_active = false;

/* Allocate enough memory for the radar dara frame. */
static uint16_t samples[NUM_SAMPLES_PER_FRAME];

static bool parse_frame_count_argument(const char *arg, uint32_t *out_value);
static void handle_command(const char *cmd);
static void process_cli(void);
static void send_frame_binary(uint32_t frame_idx);
static void status_printf(const char *fmt, ...);
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
void xensiv_bgt60trxx_mtb_interrupt_handler(void *args, cyhal_gpio_event_t event);
#else
void xensiv_bgt60trxx_mtb_interrupt_handler(void *args, cyhal_gpio_irq_event_t event);
#endif
void usb_add_cdc(void);
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
void xensiv_bgt60trxx_mtb_interrupt_handler(void *args, cyhal_gpio_event_t event);
#else
void xensiv_bgt60trxx_mtb_interrupt_handler(void *args, cyhal_gpio_irq_event_t event);
#endif

/*********************************************************************
* Information that are used during enumeration
**********************************************************************/
static const USB_DEVICE_INFO usb_deviceInfo = {
    0x058B,                           /* VendorId    */
    0x027D,                           /* ProductId    */
    "Infineon Technologies",          /* VendorName   */
    "CDC Radar Data",                 /* ProductName  */
    "12345678"                        /* SerialNumber */
};

/* Interrupt handler to react on sensor indicating the availability of new data */
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
void xensiv_bgt60trxx_mtb_interrupt_handler(void *args, cyhal_gpio_event_t event)
#else
void xensiv_bgt60trxx_mtb_interrupt_handler(void *args, cyhal_gpio_irq_event_t event)
#endif
{
    CY_UNUSED_PARAMETER(args);
    CY_UNUSED_PARAMETER(event);
    data_available = true;
}

#define RADAR_C 3e8f
#define RADAR_FC 60e9f
#define RADAR_LAMBDA (RADAR_C / RADAR_FC)
#define RADAR_D (RADAR_LAMBDA / 2.0f)
#define RADAR_FS 720000.0f
#define RADAR_S (460e6f / 200e-6f)

static float32_t radar_cube[32][3][128]; 
static float32_t range_profile[64];
static float32_t chirp_buffer[128];
static float32_t fft_output[128];
static float32_t window[128];
static bool window_init = false;

static void init_window(void) {
    for(int i=0; i<128; i++) {
        window[i] = 0.5f * (1.0f - arm_cos_f32(2 * PI * i / 127.0f));
    }
    window_init = true;
}

static uint32_t test_counter = 0;

static void run_radar_test(void) {
    status_printf("Debug: run_radar_test started\r\n");
    
    if (xensiv_bgt60trxx_start_frame(&sensor.dev, true) != XENSIV_BGT60TRXX_STATUS_OK) {
        status_printf("Error starting frame\r\n");
        return;
    }
    
    uint32_t timeout = 2000;
    while (!data_available && timeout > 0) { 
        cyhal_system_delay_ms(1);
        timeout--;
    }
    
    if (!data_available) {
        status_printf("Timeout waiting for data\r\n");
        xensiv_bgt60trxx_start_frame(&sensor.dev, false);
        return;
    }
    data_available = false;
    
    status_printf("Debug: Data received, reading FIFO...\r\n");
    
    if (xensiv_bgt60trxx_get_fifo_data(&sensor.dev, samples, NUM_SAMPLES_PER_FRAME) != XENSIV_BGT60TRXX_STATUS_OK) {
        status_printf("Error reading FIFO\r\n");
        xensiv_bgt60trxx_start_frame(&sensor.dev, false);
        return;
    }

    if (!window_init) init_window();
    
    status_printf("Debug: Processing...\r\n");
    
    arm_rfft_fast_instance_f32 S;
    if (arm_rfft_fast_init_f32(&S, 128) != ARM_MATH_SUCCESS) {
        status_printf("Error initializing RFFT\r\n");
        xensiv_bgt60trxx_start_frame(&sensor.dev, false);
        return;
    }
    
    memset(range_profile, 0, sizeof(range_profile));
    
    int num_chirps = 32;
    int num_samples_per_chirp = 128;
    int num_rx = 3;
    
    for (int c = 0; c < num_chirps; c++) {
        for (int r = 0; r < num_rx; r++) {
            float32_t mean = 0;
            for (int s = 0; s < num_samples_per_chirp; s++) {
                int idx = (c * num_samples_per_chirp + s) * num_rx + r;
                chirp_buffer[s] = (float32_t)samples[idx];
                mean += chirp_buffer[s];
            }
            mean /= num_samples_per_chirp;
            
            for (int s = 0; s < num_samples_per_chirp; s++) {
                chirp_buffer[s] -= mean;
                chirp_buffer[s] *= window[s];
            }
            
            arm_rfft_fast_f32(&S, chirp_buffer, fft_output, 0);
            
            for (int k = 1; k < 64; k++) {
                float32_t re = fft_output[2*k];
                float32_t im = fft_output[2*k+1];
                float32_t mag2 = re*re + im*im;
                range_profile[k] += mag2;
                
                radar_cube[c][r][2*k] = re;
                radar_cube[c][r][2*k+1] = im;
            }
        }
    }
    
    float32_t max_val = 0;
    int peak_idx = 0;
    for (int k = 1; k < 64; k++) {
        if (range_profile[k] > max_val) {
            max_val = range_profile[k];
            peak_idx = k;
        }
    }
    
    float32_t range_m = peak_idx * (RADAR_C * RADAR_FS) / (2 * RADAR_S * 128.0f);
    
    arm_cfft_instance_f32 S_dop;
    if (arm_cfft_init_f32(&S_dop, 32) != ARM_MATH_SUCCESS) {
        status_printf("Error initializing CFFT\r\n");
        xensiv_bgt60trxx_start_frame(&sensor.dev, false);
        return;
    }
    
    float32_t doppler_spectrum[32][3][2];
    
    for (int r = 0; r < num_rx; r++) {
        float32_t dop_input[64];
        for (int c = 0; c < 32; c++) {
            dop_input[2*c] = radar_cube[c][r][2*peak_idx];
            dop_input[2*c+1] = radar_cube[c][r][2*peak_idx+1];
        }
        
        arm_cfft_f32(&S_dop, dop_input, 0, 1);
        
        for (int d = 0; d < 32; d++) {
            doppler_spectrum[d][r][0] = dop_input[2*d];
            doppler_spectrum[d][r][1] = dop_input[2*d+1];
        }
    }
    
    float32_t max_dop_val = 0;
    int peak_dop_idx = 0;
    for (int d = 0; d < 32; d++) {
        float32_t mag = 0;
        for (int r = 0; r < num_rx; r++) {
            float32_t re = doppler_spectrum[d][r][0];
            float32_t im = doppler_spectrum[d][r][1];
            mag += re*re + im*im;
        }
        if (mag > max_dop_val) {
            max_dop_val = mag;
            peak_dop_idx = d;
        }
    }
    
    float32_t z[3][2];
    for(int r=0; r<3; r++) {
        z[r][0] = doppler_spectrum[peak_dop_idx][r][0];
        z[r][1] = doppler_spectrum[peak_dop_idx][r][1];
    }
    
    float32_t d1_re = z[1][0]*z[0][0] + z[1][1]*z[0][1];
    float32_t d1_im = z[1][1]*z[0][0] - z[1][0]*z[0][1];
    float32_t phi1 = atan2f(d1_im, d1_re);
    
    float32_t d2_re = z[2][0]*z[1][0] + z[2][1]*z[1][1];
    float32_t d2_im = z[2][1]*z[1][0] - z[2][0]*z[1][1];
    float32_t phi2 = atan2f(d2_im, d2_re);
    
    float32_t avg_phi = (phi1 + phi2) / 2.0f;
    float32_t aoa_rad = asinf(avg_phi / PI);
    float32_t aoa_deg = aoa_rad * 180.0f / PI;
    
    status_printf("Range: %.2f m, AoA: %.2f deg\r\n", range_m, aoa_deg);
    
    xensiv_bgt60trxx_start_frame(&sensor.dev, false);
}

int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals. */
    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();

    /* Initialize retarget-io to use the debug UART port. */
    result = cy_retarget_io_init(P7_1, P7_0, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    // printf("Device started, initializing USB...\r\n");

    /* Initialize the User LED */
    cyhal_gpio_init(LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);

    /* Initializes the USB stack */
    USBD_Init();
    usb_add_cdc();
    USBD_SetDeviceInfo(&usb_deviceInfo);
    USBD_Start();

    // printf("USB started, waiting for configuration...\r\n");

    /* Wait for USB configuration with timeout */
    uint32_t timeout = 0;
    while ((USBD_GetState() & USB_STAT_CONFIGURED) != USB_STAT_CONFIGURED)
    {
        cyhal_system_delay_ms(USB_CONFIG_DELAY);
        timeout++;
        if (timeout > 200)  /* 10 seconds timeout */
        {
            // printf("USB configuration timeout!\r\n");
            break;
        }
    }

    status_printf("USB configured.\r\n");

    /* Allow time for USB to fully initialize */
    cyhal_system_delay_ms(500);

    status_printf("XENSIV BGT60TRxx Example\r\n");
    status_printf("Serial terminal connected.\r\n");

    status_printf("Debug: Initializing SPI...\r\n");
    /* Initialize the SPI interface to BGT60. */
    result = cyhal_spi_init(&cyhal_spi,
                            PIN_XENSIV_BGT60TRXX_SPI_MOSI,
                            PIN_XENSIV_BGT60TRXX_SPI_MISO,
                            PIN_XENSIV_BGT60TRXX_SPI_SCLK,
                            NC,
                            NULL,
                            8,
                            CYHAL_SPI_MODE_00_MSB,
                            false);
    
    if (result != CY_RSLT_SUCCESS)
    {
        for(;;) 
        {
            status_printf("SPI connected: NO - SPI initialization failed. Error: 0x%08lX\r\n", (unsigned long)result);
            process_cli();
            cyhal_system_delay_ms(1000);
        }
    }
    status_printf("Debug: SPI Initialized.\r\n");

    /* Reduce drive strength to improve EMI */
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CY_GPIO_DRIVE_1_8);

    status_printf("Debug: Setting SPI frequency...\r\n");
    /* Set SPI data rate to communicate with sensor */
    result = cyhal_spi_set_frequency(&cyhal_spi, XENSIV_BGT60TRXX_SPI_FREQUENCY);
    
    if (result != CY_RSLT_SUCCESS)
    {
        for(;;) 
        {
            status_printf("SPI connected: NO - SPI frequency setting failed.\r\n");
            process_cli();
            cyhal_system_delay_ms(1000);
        }
    }
    status_printf("Debug: SPI frequency set.\r\n");

    /* Enable the LDO. */
    // result = cyhal_gpio_init(PIN_XENSIV_BGT60TRXX_LDO_EN,
    //                          CYHAL_GPIO_DIR_OUTPUT,
    //                          CYHAL_GPIO_DRIVE_STRONG,
    //                          true);
    // CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Wait LDO stable */
    (void)cyhal_system_delay_ms(5);

    status_printf("Debug: Initializing Radar Sensor...\r\n");
    result = xensiv_bgt60trxx_mtb_init(&sensor,
                                       &cyhal_spi,
                                       PIN_XENSIV_BGT60TRXX_SPI_CSN,
                                       PIN_XENSIV_BGT60TRXX_RSTN,
                                       register_list,
                                       XENSIV_BGT60TRXX_CONF_NUM_REGS);
    
    if (result != CY_RSLT_SUCCESS)
    {
        for(;;) 
        {
            status_printf("SPI connected: NO - Radar initialization failed. Error: 0x%08lX\r\n", (unsigned long)result);
            process_cli();
            cyhal_system_delay_ms(1000);
        }
    }
    status_printf("Debug: Radar Sensor Initialized.\r\n");

    status_printf("Debug: Initializing Interrupts...\r\n");
    /* The sensor will generate an interrupt once the sensor FIFO level is
       NUM_SAMPLES_PER_FRAME */
    result = xensiv_bgt60trxx_mtb_interrupt_init(&sensor,
                                                 NUM_SAMPLES_PER_FRAME,
                                                 PIN_XENSIV_BGT60TRXX_IRQ,
                                                 CYHAL_ISR_PRIORITY_DEFAULT,
                                                 xensiv_bgt60trxx_mtb_interrupt_handler,
                                                 NULL);
    
    if (result != CY_RSLT_SUCCESS)
    {
        for(;;) 
        {
            status_printf("SPI connected: NO - Interrupt initialization failed. Error: 0x%08lX\r\n", (unsigned long)result);
            process_cli();
            cyhal_system_delay_ms(1000);
        }
    }
    status_printf("Debug: Interrupts Initialized.\r\n");

    status_printf("Debug: Stopping/Idling Frame...\r\n");
    /* Ensure acquisition is idle until commanded via CLI */
    if (xensiv_bgt60trxx_start_frame(&sensor.dev, false) != XENSIV_BGT60TRXX_STATUS_OK)
    {
        for(;;) 
        {
            status_printf("SPI connected: NO - Failed to initialize frame acquisition.\r\n");
            process_cli();
            cyhal_system_delay_ms(1000);
        }
    }
    status_printf("Debug: Frame Idled.\r\n");

    radar_initialized = true;
    status_printf("SPI connected: YES\r\n");
    status_printf("Ready. Type 'start' [frames] or 'stop' followed by Enter.\r\n");

    uint32_t frame_idx = 0;

    for(;;)
    {
        if (!capture_enabled) {
            process_cli();
        }
        
        if (test_mode_active) {
             test_counter += 10;
             if (test_counter >= 3000) {
                 run_radar_test();
                 test_counter = 0;
             }
             cyhal_system_delay_ms(10);
             continue;
        }

        if (!capture_enabled)
        {
            cyhal_system_delay_ms(10);
            continue;
        }

        /* Wait for the radar device to indicate the availability of the data to fetch. */
        uint32_t wait_count = 0;
        
        while ((capture_enabled == true) && (data_available == false))
        {
            // process_cli();
            cyhal_system_delay_ms(1);
            wait_count++;
            if (wait_count % 200 == 0)
            {
                 // bool irq_state = cyhal_gpio_read(PIN_XENSIV_BGT60TRXX_IRQ);
                 // status_printf("Waiting... IRQ: %d, Count: %lu\r\n", irq_state, wait_count);
                 cyhal_gpio_toggle(LED1);
            }
        }

        if (!capture_enabled)
        {
            continue;
        }

        data_available = false;

        if (xensiv_bgt60trxx_get_fifo_data(&sensor.dev, samples,
                                           NUM_SAMPLES_PER_FRAME) == XENSIV_BGT60TRXX_STATUS_OK)
        {
            send_frame_binary(frame_idx);
            frame_idx++;
            cyhal_gpio_toggle(LED1);

            if (frame_limit_enabled)
            {
                frame_limit_sent++;

                if (frame_limit_sent >= frame_limit_total)
                {
                    uint32_t completed_frames = frame_limit_total;

                    if (xensiv_bgt60trxx_start_frame(&sensor.dev, false) == XENSIV_BGT60TRXX_STATUS_OK)
                    {
                        capture_enabled = false;
                        data_available = false;
                        frame_limit_enabled = false;
                        frame_limit_total = 0U;
                        frame_limit_sent = 0U;
                        binary_stream_active = false;
                        status_printf("Capture completed (%" PRIu32 " frame%s).\r\n",
                                      completed_frames,
                                      (completed_frames == 1U) ? "" : "s");
                    }
                    else
                    {
                        status_printf("Failed to stop capture.\r\n");
                    }
                }
            }
        }
    }
}

static void status_printf(const char *fmt, ...)
{
    if (fmt == NULL)
    {
        return;
    }

    // if (binary_stream_active)
    // {
    //     return;
    // }

    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    USBD_CDC_Write(usb_cdcHandle, (uint8_t *)buffer, strlen(buffer), 0);
}

static void send_frame_binary(uint32_t frame_idx)
{
    const binary_frame_header_t header = {
        .magic = {'R', 'A', 'D', 'R'},
        .version = BINARY_FRAME_HEADER_VERSION,
        .sample_size_bytes = BINARY_FRAME_SAMPLE_SIZE_BYTES,
        .frame_index = frame_idx,
        .sample_count = NUM_SAMPLES_PER_FRAME
    };

    if (USBD_CDC_Write(usb_cdcHandle, (uint8_t *)&header, sizeof(header), 0) != sizeof(header))
    {
        binary_stream_active = false;
        capture_enabled = false;
        frame_limit_enabled = false;
        status_printf("Failed to write frame header.\r\n");
        return;
    }

    if (USBD_CDC_Write(usb_cdcHandle, (uint8_t *)samples, sizeof(samples[0]) * NUM_SAMPLES_PER_FRAME, 0) != sizeof(samples[0]) * NUM_SAMPLES_PER_FRAME)
    {
        binary_stream_active = false;
        capture_enabled = false;
        frame_limit_enabled = false;
        status_printf("Failed to write frame payload.\r\n");
        return;
    }
}

static void process_cli(void)
{
    static char cmd_buffer[32];
    static uint32_t cmd_index = 0;
    uint8_t ch = 0;
    int loop_safety = 0;

    /* Try to read a byte from USB CDC (non-blocking) */
    while (USBD_CDC_Read(usb_cdcHandle, &ch, 1, 0) == 1)
    {
        loop_safety++;
        if (loop_safety > 1000) {
             status_printf("Debug: process_cli stuck, breaking.\r\n");
             break;
        }

        if ((ch == '\r') || (ch == '\n'))
        {
            if (cmd_index > 0)
            {
                cmd_buffer[cmd_index] = '\0';
                USBD_CDC_Write(usb_cdcHandle, (uint8_t *)"\r\n", 2, 0);
                handle_command(cmd_buffer);
                cmd_index = 0;
                
                if (capture_enabled) {
                    break;
                }
            }
        }
        else
        {
            if (cmd_index < (sizeof(cmd_buffer) - 1))
            {
                cmd_buffer[cmd_index++] = (char)ch;
            }
            else
            {
                /* Command too long; reset buffer */
                cmd_index = 0;
            }
        }
    }
    // status_printf("Debug: process_cli exiting.\r\n");
    // cyhal_system_delay_ms(10);
}

static bool parse_frame_count_argument(const char *arg, uint32_t *out_value)
{
    if ((arg == NULL) || (out_value == NULL))
    {
        return false;
    }

    while ((*arg == ' ') || (*arg == '\t'))
    {
        ++arg;
    }

    if (*arg == '\0')
    {
        *out_value = 0U;
        return true;
    }

    uint32_t value = 0U;

    while ((*arg >= '0') && (*arg <= '9'))
    {
        uint32_t digit = (uint32_t)(*arg - '0');

        if (value > ((UINT32_MAX - digit) / 10U))
        {
            return false;
        }

        value = (value * 10U) + digit;
        ++arg;
    }

    while ((*arg == ' ') || (*arg == '\t'))
    {
        ++arg;
    }

    if (*arg != '\0')
    {
        return false;
    }

    *out_value = value;
    return true;
}

static void handle_command(const char *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    while ((*cmd == ' ') || (*cmd == '\t'))
    {
        ++cmd;
    }

    if ((strncmp(cmd, "start", 5) == 0) &&
        ((cmd[5] == '\0') || (cmd[5] == ' ') || (cmd[5] == '\t')))
    {
        if (!radar_initialized)
        {
            status_printf("Error: Radar not initialized.\r\n");
            return;
        }

        uint32_t requested_frames = 0U;

        if (!parse_frame_count_argument(cmd + 5, &requested_frames))
        {
            status_printf("Invalid frame count.\r\n");
            return;
        }

        if (capture_enabled)
        {
            status_printf("Capture already running.\r\n");
            return;
        }

        if (xensiv_bgt60trxx_start_frame(&sensor.dev, true) == XENSIV_BGT60TRXX_STATUS_OK)
        {
            capture_enabled = true;
            data_available = false;
            frame_limit_enabled = (requested_frames > 0U);
            frame_limit_total = requested_frames;
            frame_limit_sent = 0U;

            if (frame_limit_enabled)
            {
                status_printf("Capture started (%" PRIu32 " frame%s).\r\n",
                              requested_frames,
                              (requested_frames == 1U) ? "" : "s");
            }
            else
            {
                status_printf("Capture started (continuous).\r\n");
            }

            binary_stream_active = true;
            // status_printf("Debug: handle_command returning.\r\n");
            cyhal_gpio_write(LED1, 1); // Force LED ON
            cyhal_system_delay_ms(10);
        }
        else
        {
            status_printf("Failed to start capture.\r\n");
        }
    }
    else if ((strncmp(cmd, "stop", 4) == 0) &&
             ((cmd[4] == '\0') || (cmd[4] == ' ') || (cmd[4] == '\t')))
    {
        const char *trailing = cmd + 4;

        while ((*trailing == ' ') || (*trailing == '\t'))
        {
            ++trailing;
        }

        if (*trailing != '\0')
        {
            status_printf("Unknown command: %s\r\n", cmd);
            return;
        }

        if (test_mode_active) {
            test_mode_active = false;
            status_printf("Test mode stopped.\r\n");
            return;
        }

        if (!capture_enabled)
        {
            status_printf("Capture already stopped.\r\n");
            return;
        }

        if (xensiv_bgt60trxx_start_frame(&sensor.dev, false) == XENSIV_BGT60TRXX_STATUS_OK)
        {
            capture_enabled = false;
            data_available = false;
            frame_limit_enabled = false;
            frame_limit_total = 0U;
            frame_limit_sent = 0U;
            binary_stream_active = false;
            status_printf("Capture stopped.\r\n");
        }
        else
        {
            status_printf("Failed to stop capture.\r\n");
        }
    }
    else if ((strncmp(cmd, "test", 4) == 0) &&
             ((cmd[4] == '\0') || (cmd[4] == ' ') || (cmd[4] == '\t')))
    {
        if (!radar_initialized)
        {
            status_printf("Error: Radar not initialized.\r\n");
            return;
        }
        
        if (capture_enabled)
        {
            status_printf("Stop capture first.\r\n");
            return;
        }
        
        test_mode_active = !test_mode_active;
        if (test_mode_active) {
            status_printf("Test mode started. Range/AoA every 3s.\r\n");
            test_counter = 3000;
        } else {
            status_printf("Test mode stopped.\r\n");
        }
    }
    else if (*cmd != '\0')
    {
        status_printf("Unknown command: %s\r\n", cmd);
    }
}

void usb_add_cdc(void) {
    static U8             OutBuffer[USB_FS_BULK_MAX_PACKET_SIZE];
    USB_CDC_INIT_DATA     InitData;
    USB_ADD_EP_INFO       EPBulkIn;
    USB_ADD_EP_INFO       EPBulkOut;
    USB_ADD_EP_INFO       EPIntIn;

    memset(&InitData, 0, sizeof(InitData));
    EPBulkIn.Flags          = 0;                             /* Flags not used */
    EPBulkIn.InDir          = USB_DIR_IN;                    /* IN direction (Device to Host) */
    EPBulkIn.Interval       = 0;                             /* Interval not used for Bulk endpoints */
    EPBulkIn.MaxPacketSize  = USB_FS_BULK_MAX_PACKET_SIZE;   /* Maximum packet size (64B for Bulk in full-speed) */
    EPBulkIn.TransferType   = USB_TRANSFER_TYPE_BULK;        /* Endpoint type - Bulk */
    InitData.EPIn  = USBD_AddEPEx(&EPBulkIn, NULL, 0);

    EPBulkOut.Flags         = 0;                             /* Flags not used */
    EPBulkOut.InDir         = USB_DIR_OUT;                   /* OUT direction (Host to Device) */
    EPBulkOut.Interval      = 0;                             /* Interval not used for Bulk endpoints */
    EPBulkOut.MaxPacketSize = USB_FS_BULK_MAX_PACKET_SIZE;   /* Maximum packet size (64B for Bulk in full-speed) */
    EPBulkOut.TransferType  = USB_TRANSFER_TYPE_BULK;        /* Endpoint type - Bulk */
    InitData.EPOut = USBD_AddEPEx(&EPBulkOut, OutBuffer, sizeof(OutBuffer));

    EPIntIn.Flags           = 0;                             /* Flags not used */
    EPIntIn.InDir           = USB_DIR_IN;                    /* IN direction (Device to Host) */
    EPIntIn.Interval        = 64;                            /* Interval of 8 ms (64 * 125us) */
    EPIntIn.MaxPacketSize   = USB_FS_INT_MAX_PACKET_SIZE ;   /* Maximum packet size (64 for Interrupt) */
    EPIntIn.TransferType    = USB_TRANSFER_TYPE_INT;         /* Endpoint type - Interrupt */
    InitData.EPInt = USBD_AddEPEx(&EPIntIn, NULL, 0);

    usb_cdcHandle = USBD_CDC_Add(&InitData);
}
/* [] END OF FILE */