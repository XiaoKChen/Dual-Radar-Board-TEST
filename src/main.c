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

/* Radar 1 Pins */
#define PIN_RADAR1_CS       Radar_CS_1
#define PIN_RADAR1_IRQ      Radar_IRQ_1

/* Radar 2 Pins */
#define PIN_RADAR2_CS       Radar_CS_2
#define PIN_RADAR2_IRQ      Radar_IRQ_2

/* Shared Pins */
#define PIN_RADAR_SPI_SCLK  Radar_SPI_CLK
#define PIN_RADAR_SPI_MOSI  Radar_SPI_MOSI
#define PIN_RADAR_SPI_MISO  Radar_SPI_MISO
#define PIN_RADAR_RST       Radar_RST

#define XENSIV_BGT60TRXX_SPI_FREQUENCY      (24000000UL)

#define NUM_SAMPLES_PER_FRAME               (XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS *\
                                             XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME *\
                                             XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP)

#define LED1 P10_3
#define LED2 P10_2

#define RADAR_C 299792458.0f
#define RADAR_FC_START 61020099000.0f
#define RADAR_FC_END 61479903000.0f
#define RADAR_B (RADAR_FC_END - RADAR_FC_START)
#define RADAR_FC (0.5f * (RADAR_FC_START + RADAR_FC_END))
#define RADAR_LAMBDA (RADAR_C / RADAR_FC)
#define RADAR_D (RADAR_LAMBDA / 2.0f)
#define RADAR_FS 720000.0f

/*******************************************************************************
* Global variables
********************************************************************************/
static USB_CDC_HANDLE usb_cdcHandle;

static cyhal_spi_t cyhal_spi;
static xensiv_bgt60trxx_mtb_t sensor1;
static xensiv_bgt60trxx_mtb_t sensor2;

static volatile bool data_available_1 = false;
static volatile bool data_available_2 = false;

static bool sequence_running = true;

/* Allocate enough memory for the radar dara frame. */
static uint16_t samples[NUM_SAMPLES_PER_FRAME];

static float32_t radar_cube[32][3][128]; 
static float32_t range_profile[64];
static float32_t chirp_buffer[128];
static float32_t fft_output[128];
static float32_t window[128];
static bool window_init = false;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void status_printf(const char *fmt, ...);
static void process_usb_input(void);
static void usb_add_cdc(void);
static void init_window(void);
static void run_radar_sequence(xensiv_bgt60trxx_mtb_t *sensor_obj, volatile bool *data_flag, const char *name);
static cy_rslt_t init_dual_radars(void);

#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
void radar1_irq_handler(void *args, cyhal_gpio_event_t event);
void radar2_irq_handler(void *args, cyhal_gpio_event_t event);
#else
void radar1_irq_handler(void *args, cyhal_gpio_irq_event_t event);
void radar2_irq_handler(void *args, cyhal_gpio_irq_event_t event);
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

/* Interrupt handlers */
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
void radar1_irq_handler(void *args, cyhal_gpio_event_t event)
#else
void radar1_irq_handler(void *args, cyhal_gpio_irq_event_t event)
#endif
{
    CY_UNUSED_PARAMETER(args);
    CY_UNUSED_PARAMETER(event);
    data_available_1 = true;
}

#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
void radar2_irq_handler(void *args, cyhal_gpio_event_t event)
#else
void radar2_irq_handler(void *args, cyhal_gpio_irq_event_t event)
#endif
{
    CY_UNUSED_PARAMETER(args);
    CY_UNUSED_PARAMETER(event);
    data_available_2 = true;
}

static void init_window(void) {
    for(int i=0; i<128; i++) {
        window[i] = 0.5f * (1.0f - arm_cos_f32(2 * PI * i / 127.0f));
    }
    window_init = true;
}

static cy_rslt_t init_dual_radars(void) {
    cy_rslt_t rslt;
    
    /* Initialize GPIOs - Both CS pins HIGH (inactive) */
    cyhal_gpio_init(PIN_RADAR1_CS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
    cyhal_gpio_init(PIN_RADAR2_CS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
    cyhal_gpio_init(PIN_RADAR_RST, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);

    /* Hard Reset (Shared) - Pulse Low */
    cyhal_gpio_write(PIN_RADAR_RST, 1);
    cyhal_system_delay_ms(1);
    cyhal_gpio_write(PIN_RADAR_RST, 0);
    cyhal_system_delay_ms(1);
    cyhal_gpio_write(PIN_RADAR_RST, 1);
    cyhal_system_delay_ms(10); /* Wait for sensors to boot */

    status_printf("Initializing Radar 1...\r\n");
    cyhal_system_delay_ms(50);
    
    /* Ensure Radar 2 CS is HIGH (inactive) */
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
    
    /* Initialize Sensor 1 (Skip hardware reset as we did it manually) */
    rslt = xensiv_bgt60trxx_init(&sensor1.dev, &sensor1.iface, false);
    if (rslt == XENSIV_BGT60TRXX_STATUS_OK) {
        rslt = xensiv_bgt60trxx_config(&sensor1.dev, register_list, XENSIV_BGT60TRXX_CONF_NUM_REGS);
    }
    if (rslt == XENSIV_BGT60TRXX_STATUS_OK) {
        rslt = xensiv_bgt60trxx_mtb_interrupt_init(&sensor1, NUM_SAMPLES_PER_FRAME, PIN_RADAR1_IRQ, CYHAL_ISR_PRIORITY_DEFAULT, radar1_irq_handler, NULL);
    }
    
    if (rslt != CY_RSLT_SUCCESS) {
        status_printf("Radar 1 init failed: 0x%08lX\r\n", (unsigned long)rslt);
        return rslt;
    }
    
    status_printf("Radar 1 initialized successfully\r\n");
    cyhal_system_delay_ms(50);
    
    /* Ensure Radar 1 CS is HIGH (inactive) before initializing Radar 2 */
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

    /* Initialize Sensor 2 */
    rslt = xensiv_bgt60trxx_init(&sensor2.dev, &sensor2.iface, false);
    if (rslt == XENSIV_BGT60TRXX_STATUS_OK) {
        rslt = xensiv_bgt60trxx_config(&sensor2.dev, register_list, XENSIV_BGT60TRXX_CONF_NUM_REGS);
    }
    if (rslt == XENSIV_BGT60TRXX_STATUS_OK) {
        rslt = xensiv_bgt60trxx_mtb_interrupt_init(&sensor2, NUM_SAMPLES_PER_FRAME, PIN_RADAR2_IRQ, CYHAL_ISR_PRIORITY_DEFAULT, radar2_irq_handler, NULL);
    }
    
    if (rslt != CY_RSLT_SUCCESS) {
        status_printf("Radar 2 init failed: 0x%08lX\r\n", (unsigned long)rslt);
        return rslt;
    }
    
    status_printf("Radar 2 initialized successfully\r\n");
    cyhal_system_delay_ms(50);
    
    /* Ensure both CS pins are HIGH (inactive) */
    cyhal_gpio_write(PIN_RADAR1_CS, 1);
    cyhal_gpio_write(PIN_RADAR2_CS, 1);

    return rslt;
}

static void run_radar_sequence(xensiv_bgt60trxx_mtb_t *sensor_obj, volatile bool *data_flag, const char *name) {
    status_printf("\r\n--- %s: START ---\r\n", name);
    cyhal_system_delay_ms(50);
    
    /* Clear the data flag */
    *data_flag = false;

    /* Start frame acquisition */
    status_printf("%s: Starting frame acquisition...\r\n", name);
    cyhal_system_delay_ms(50);
    
    int32_t res = xensiv_bgt60trxx_start_frame(&sensor_obj->dev, true);
    if (res != XENSIV_BGT60TRXX_STATUS_OK) {
        status_printf("%s: ERROR starting frame: %ld\r\n", name, (long)res);
        cyhal_system_delay_ms(50);
        return;
    }
    
    status_printf("%s: Waiting for data...\r\n", name);
    cyhal_system_delay_ms(50);
    
    /* Wait for interrupt with timeout */
    uint32_t timeout = 3000;
    while (!(*data_flag) && timeout > 0) { 
        cyhal_system_delay_ms(1);
        timeout--;
    }
    
    if (!(*data_flag)) {
        status_printf("%s: TIMEOUT - No data received\r\n", name);
        cyhal_system_delay_ms(50);
        xensiv_bgt60trxx_start_frame(&sensor_obj->dev, false);
        return;
    }
    
    /* Data received, stop frame */
    status_printf("%s: Data received, stopping frame\r\n", name);
    cyhal_system_delay_ms(50);
    xensiv_bgt60trxx_start_frame(&sensor_obj->dev, false);
    
    if (xensiv_bgt60trxx_get_fifo_data(&sensor_obj->dev, samples, NUM_SAMPLES_PER_FRAME) != XENSIV_BGT60TRXX_STATUS_OK) {
        status_printf("Error reading FIFO from %s\r\n", name);
        return;
    }

    if (!window_init) init_window();
    
    // status_printf("Processing %s data...\r\n", name);
    
    arm_rfft_fast_instance_f32 S;
    if (arm_rfft_fast_init_f32(&S, 128) != ARM_MATH_SUCCESS) {
        status_printf("Error initializing RFFT\r\n");
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
    
    float32_t range_m = peak_idx * RADAR_C / (2.0f * RADAR_B);
    
    arm_cfft_instance_f32 S_dop;
    if (arm_cfft_init_f32(&S_dop, 32) != ARM_MATH_SUCCESS) {
        status_printf("Error initializing CFFT\r\n");
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
    
    status_printf("[%s] Range: %.2f m, AoA: %.2f deg\r\n", name, range_m, aoa_deg);
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

    /* Initialize the User LED */
    cyhal_gpio_init(LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);

    /* Initializes the USB stack */
    USBD_Init();
    usb_add_cdc();
    USBD_SetDeviceInfo(&usb_deviceInfo);
    USBD_Start();

    /* Wait for USB configuration with timeout */
    uint32_t timeout = 0;
    while ((USBD_GetState() & USB_STAT_CONFIGURED) != USB_STAT_CONFIGURED)
    {
        cyhal_system_delay_ms(USB_CONFIG_DELAY);
        timeout++;
        if (timeout > 200)  /* 10 seconds timeout */
        {
            break;
        }
    }

    status_printf("USB configured.\r\n");
    cyhal_system_delay_ms(500);

    status_printf("Dual Radar Example\r\n");

    status_printf("Debug: Initializing SPI...\r\n");
    /* Initialize the SPI interface to BGT60. */
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
        for(;;) {}
    }
    
    /* Reduce drive strength to improve EMI */
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_RADAR_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_RADAR_SPI_MOSI), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_RADAR_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_RADAR_SPI_MOSI), CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_RADAR_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_RADAR_SPI_SCLK), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_RADAR_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_RADAR_SPI_SCLK), CY_GPIO_DRIVE_1_8);

    status_printf("Debug: Setting SPI frequency...\r\n");
    result = cyhal_spi_set_frequency(&cyhal_spi, XENSIV_BGT60TRXX_SPI_FREQUENCY);
    if (result != CY_RSLT_SUCCESS)
    {
        status_printf("SPI frequency setting failed.\r\n");
        for(;;) {}
    }

    status_printf("Debug: Initializing Dual Radars...\r\n");
    result = init_dual_radars();
    if (result != CY_RSLT_SUCCESS)
    {
        status_printf("Radar initialization failed. Error: 0x%08lX\r\n", (unsigned long)result);
        for(;;) {}
    }
    status_printf("Dual Radars Initialized.\r\n");
    status_printf("Starting sequence loop (Radar 1 -> Radar 2 -> Wait 3s).\r\n");
    cyhal_system_delay_ms(100);

    uint32_t loop_count = 0;
    for(;;)
    {
        loop_count++;

        if (sequence_running) {
            /* Toggle LED to show activity */
            cyhal_gpio_write(LED1, 1);
            
            status_printf("\r\n=== CYCLE %lu START ===\r\n", loop_count);
            
            /* RADAR 1: ON -> Capture -> OFF -> Process */
            run_radar_sequence(&sensor1, &data_available_1, "Radar 1");
            cyhal_system_delay_ms(200); /* Delay between radars */
            
            /* RADAR 2: ON -> Capture -> OFF -> Process */
            run_radar_sequence(&sensor2, &data_available_2, "Radar 2");
            
            status_printf("\r\n=== Cycle %lu complete, waiting 3s ===\r\n", loop_count);
            
            cyhal_gpio_write(LED1, 0);
            cyhal_system_delay_ms(3000);
        } else {
            cyhal_system_delay_ms(100);
        }
    }
}

static void status_printf(const char *fmt, ...)
{
    if (fmt == NULL) return;

    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    
    if ((USBD_GetState() & USB_STAT_CONFIGURED) == USB_STAT_CONFIGURED) {
        uint32_t len = strlen(buffer);
        /* Non-blocking write - if it fails, we just drop the message */
        USBD_CDC_Write(usb_cdcHandle, (uint8_t *)buffer, len, 0);
    }
}

void usb_add_cdc(void) {
    static U8             OutBuffer[USB_FS_BULK_MAX_PACKET_SIZE];
    USB_CDC_INIT_DATA     InitData;
    USB_ADD_EP_INFO       EPBulkIn;
    USB_ADD_EP_INFO       EPBulkOut;
    USB_ADD_EP_INFO       EPIntIn;

    memset(&InitData, 0, sizeof(InitData));
    EPBulkIn.Flags          = 0;
    EPBulkIn.InDir          = USB_DIR_IN;
    EPBulkIn.Interval       = 0;
    EPBulkIn.MaxPacketSize  = USB_FS_BULK_MAX_PACKET_SIZE;
    EPBulkIn.TransferType   = USB_TRANSFER_TYPE_BULK;
    InitData.EPIn  = USBD_AddEPEx(&EPBulkIn, NULL, 0);

    EPBulkOut.Flags         = 0;
    EPBulkOut.InDir         = USB_DIR_OUT;
    EPBulkOut.Interval      = 0;
    EPBulkOut.MaxPacketSize = USB_FS_BULK_MAX_PACKET_SIZE;
    EPBulkOut.TransferType  = USB_TRANSFER_TYPE_BULK;
    InitData.EPOut = USBD_AddEPEx(&EPBulkOut, OutBuffer, sizeof(OutBuffer));

    EPIntIn.Flags           = 0;
    EPIntIn.InDir           = USB_DIR_IN;
    EPIntIn.Interval        = 64;
    EPIntIn.MaxPacketSize   = USB_FS_INT_MAX_PACKET_SIZE ;
    EPIntIn.TransferType    = USB_TRANSFER_TYPE_INT;
    InitData.EPInt = USBD_AddEPEx(&EPIntIn, NULL, 0);

    usb_cdcHandle = USBD_CDC_Add(&InitData);
}
