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

#include "presence_detection.h"
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

/*******************************************************************************
* Global variables
********************************************************************************/
static USB_CDC_HANDLE usb_cdcHandle;

static cyhal_spi_t cyhal_spi;
static xensiv_bgt60trxx_mtb_t sensor1;
static xensiv_bgt60trxx_mtb_t sensor2;

// Presence Detectors
static presence_detector_t detector1;
static presence_detector_t detector2;

static volatile bool data_available_1 = false;
static volatile bool data_available_2 = false;

static bool sequence_running = true;

/* Allocate enough memory for the radar dara frame. */
static uint16_t samples[NUM_SAMPLES_PER_FRAME];

// Averaged chirp buffer for processing
static float32_t averaged_chirp[XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP];

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void status_printf(const char *fmt, ...);
static void usb_add_cdc(void);
static void run_radar_sequence(xensiv_bgt60trxx_mtb_t *sensor_obj, volatile bool *data_flag, presence_detector_t* detector, uint32_t time_ms, const char *name);
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

    // Initialize Presence Detectors
    presence_init(&detector1);
    presence_init(&detector2);

    return rslt;
}

static void run_radar_sequence(xensiv_bgt60trxx_mtb_t *sensor_obj, volatile bool *data_flag, presence_detector_t* detector, uint32_t time_ms, const char *name) {
    /* Clear the data flag */
    *data_flag = false;

    /* Start frame acquisition */
    int32_t res = xensiv_bgt60trxx_start_frame(&sensor_obj->dev, true);
    if (res != XENSIV_BGT60TRXX_STATUS_OK) {
        // status_printf("%s: ERROR starting frame: %ld\r\n", name, (long)res);
        return;
    }
    
    /* Wait for interrupt with timeout */
    uint32_t timeout = 50; // Reduced timeout for continuous loop
    while (!(*data_flag) && timeout > 0) { 
        cyhal_system_delay_ms(1);
        timeout--;
    }
    
    if (!(*data_flag)) {
        // status_printf("%s: TIMEOUT - No data received\r\n", name);
        xensiv_bgt60trxx_start_frame(&sensor_obj->dev, false);
        return;
    }
    
    /* Data received, stop frame (not strictly necessary for soft trigger but good practice) */
    xensiv_bgt60trxx_start_frame(&sensor_obj->dev, false);
    
    if (xensiv_bgt60trxx_get_fifo_data(&sensor_obj->dev, samples, NUM_SAMPLES_PER_FRAME) != XENSIV_BGT60TRXX_STATUS_OK) {
        // status_printf("Error reading FIFO from %s\r\n", name);
        return;
    }

    // Process Data
    // Average Chirps: The settings have 32 chirps per frame.
    // XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME = 32
    // XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP = 128
    // XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS = 1
    
    // Reset average buffer
    memset(averaged_chirp, 0, sizeof(averaged_chirp));
    
    int num_chirps = XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME;
    int samples_per_chirp = XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP;
    
    float32_t overall_mean = 0.0f;

    for (int s = 0; s < samples_per_chirp; s++) {
        float32_t sum = 0.0f;
        for (int c = 0; c < num_chirps; c++) {
            // samples layout: [chirp0_s0, chirp0_s1... chirp1_s0...]
            sum += (float32_t)samples[c * samples_per_chirp + s];
        }
        averaged_chirp[s] = sum / (float32_t)num_chirps;
        overall_mean += averaged_chirp[s];
    }
    
    overall_mean /= (float32_t)samples_per_chirp;
    
    // Remove DC component (Mean Removal)
    for (int s = 0; s < samples_per_chirp; s++) {
        averaged_chirp[s] -= overall_mean;
    }
    
    // Run Presence Detection
    presence_result_t result;
    presence_process(detector, averaged_chirp, time_ms, &result);
    
    // Note: We don't print here anymore.
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

    status_printf("Dual Radar Presence Detection\r\n");

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
    status_printf("Starting presence detection loop...\r\n");
    cyhal_system_delay_ms(100);

    uint32_t current_time_ms = 0;
    uint32_t last_print_time = 0;
    const uint32_t FRAME_INTERVAL_MS = 50; // 20Hz

    for(;;)
    {
        if (sequence_running) {
            cyhal_gpio_write(LED1, 1);
            
            // Radar 1
            run_radar_sequence(&sensor1, &data_available_1, &detector1, current_time_ms, "Radar 1");
            
            // Radar 2
            run_radar_sequence(&sensor2, &data_available_2, &detector2, current_time_ms, "Radar 2");
            
            // Increment Time
            current_time_ms += FRAME_INTERVAL_MS;
            
            // Check for Print Interval (3 seconds)
            if (current_time_ms - last_print_time >= 3000) {
                
                const char* state1_str = (detector1.state == PRESENCE_STATE_ABSENCE) ? "ABSENCE" : 
                                         (detector1.state == PRESENCE_STATE_MACRO_PRESENCE) ? "MACRO" : "MICRO";
                
                const char* state2_str = (detector2.state == PRESENCE_STATE_ABSENCE) ? "ABSENCE" : 
                                         (detector2.state == PRESENCE_STATE_MACRO_PRESENCE) ? "MACRO" : "MICRO";
                
                status_printf("Time: %lu ms | R1: %s (%.2fm) | R2: %s (%.2fm)\r\n", 
                              (unsigned long)current_time_ms, 
                              state1_str, detector1.range_m, 
                              state2_str, detector2.range_m);
                
                last_print_time = current_time_ms;
            }
            
            cyhal_gpio_write(LED1, 0);
            
            // Optional: small delay to not hog CPU/Bus completely, though we want max speed to meet 50ms if possible.
            // If the acquisition takes > 50ms, this loop runs slower than real time 20Hz.
            cyhal_system_delay_ms(1); 
            
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
