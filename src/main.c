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

#include "xensiv_bgt60trxx_mtb.h"
#include "xensiv_radar_presence.h"

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

/*******************************************************************************
* Global variables
********************************************************************************/
static USB_CDC_HANDLE usb_cdcHandle;

static cyhal_spi_t cyhal_spi;
static xensiv_bgt60trxx_mtb_t sensor;
static volatile bool data_available = false;
static bool capture_enabled = false;
static bool radar_initialized = false;

/* Allocate enough memory for the radar dara frame. */
static uint16_t samples[NUM_SAMPLES_PER_FRAME];
static float32_t avg_chirp[XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP];

static xensiv_radar_presence_handle_t presence_handle;
static xensiv_radar_presence_config_t presence_config;

static void handle_command(const char *cmd);
static void process_cli(void);
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

void presence_callback(xensiv_radar_presence_handle_t handle,
                       const xensiv_radar_presence_event_t *event,
                       void *data);

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

    /* Initialize Presence Radar */
    xensiv_radar_presence_init_config(&presence_config);
    if (xensiv_radar_presence_alloc(&presence_handle, &presence_config) != XENSIV_RADAR_PRESENCE_OK)
    {
        status_printf("Error: Failed to allocate presence context.\r\n");
        for(;;);
    }
    xensiv_radar_presence_set_callback(presence_handle, presence_callback, NULL);
    status_printf("Presence detection initialized.\r\n");

    radar_initialized = true;
    status_printf("SPI connected: YES\r\n");
    status_printf("Ready. Type 'start' or 'stop' followed by Enter.\r\n");

    uint32_t frame_count = 0;

    for(;;)
    {
        if (!capture_enabled) {
            process_cli();
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
            /* Debug: Print first few raw samples */
            if (frame_count % 10 == 0) {
                status_printf("Raw: %d %d %d %d %d %d %d %d %d %d %d %d\r\n",
                    samples[0], samples[1], samples[2], samples[3], samples[4], samples[5],
                    samples[6], samples[7], samples[8], samples[9], samples[10], samples[11]);
            }

            /* Use first chirp, first antenna; scale to float */
            for (uint32_t sample_idx = 0; sample_idx < XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP; sample_idx++)
            {
                uint32_t raw_idx = (sample_idx * XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS);
                avg_chirp[sample_idx] = ((float32_t)samples[raw_idx]) / 4096.0f;
            }

            /* Process one chirp per frame */
            uint32_t time_ms = (uint32_t)(frame_count * XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S * 1000.0f);
            int32_t proc_status = xensiv_radar_presence_process_frame(presence_handle, avg_chirp, time_ms);
            if (proc_status != XENSIV_RADAR_PRESENCE_OK)
            {
                status_printf("Error: Presence process failed: %ld\r\n", (long)proc_status);
            }

            /* Debug: Print energy levels to confirm operation (once per frame) */
            float macro_energy = 0.0f;
            int macro_idx = 0;
            float micro_energy = 0.0f;
            int micro_idx = 0;
            
            xensiv_radar_presence_get_max_macro(presence_handle, &macro_energy, &macro_idx);
            xensiv_radar_presence_get_max_micro(presence_handle, &micro_energy, &micro_idx);
            
            status_printf("Frame %lu: Macro: %ld (idx %d), Micro: %ld (idx %d)\r\n", 
                          (unsigned long)frame_count, 
                          (long)(macro_energy * 1000), macro_idx, 
                          (long)(micro_energy * 1000), micro_idx);
            
            frame_count++;
            cyhal_gpio_toggle(LED1);
        }
    }
}

static void status_printf(const char *fmt, ...)
{
    if (fmt == NULL)
    {
        return;
    }

    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    USBD_CDC_Write(usb_cdcHandle, (uint8_t *)buffer, strlen(buffer), 0);
}

void presence_callback(xensiv_radar_presence_handle_t handle,
                       const xensiv_radar_presence_event_t *event,
                       void *data)
{
    (void)handle;
    (void)data;
    
    if (event->state == XENSIV_RADAR_PRESENCE_STATE_MACRO_PRESENCE)
    {
        status_printf("Presence Detected: MACRO (Range Bin: %ld)\r\n", (long)event->range_bin);
    }
    else if (event->state == XENSIV_RADAR_PRESENCE_STATE_MICRO_PRESENCE)
    {
        status_printf("Presence Detected: MICRO (Range Bin: %ld)\r\n", (long)event->range_bin);
    }
    else
    {
        status_printf("Presence: ABSENT\r\n");
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

        if (capture_enabled)
        {
            status_printf("Presence detection already running.\r\n");
            return;
        }

        if (xensiv_bgt60trxx_start_frame(&sensor.dev, true) == XENSIV_BGT60TRXX_STATUS_OK)
        {
            capture_enabled = true;
            data_available = false;
            status_printf("Presence detection started.\r\n");
            cyhal_gpio_write(LED1, 1); // Force LED ON
            cyhal_system_delay_ms(10);
        }
        else
        {
            status_printf("Failed to start presence detection.\r\n");
        }
    }
    else if ((strncmp(cmd, "stop", 4) == 0) &&
             ((cmd[4] == '\0') || (cmd[4] == ' ') || (cmd[4] == '\t')))
    {
        if (!capture_enabled)
        {
            status_printf("Presence detection already stopped.\r\n");
            return;
        }

        if (xensiv_bgt60trxx_start_frame(&sensor.dev, false) == XENSIV_BGT60TRXX_STATUS_OK)
        {
            capture_enabled = false;
            data_available = false;
            status_printf("Presence detection stopped.\r\n");
        }
        else
        {
            status_printf("Failed to stop presence detection.\r\n");
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