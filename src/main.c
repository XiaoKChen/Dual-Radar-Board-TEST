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

#define XENSIV_BGT60TRXX_CONF_IMPL
#include "presence_radar_settings.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define USB_CONFIG_DELAY          (50U) /* In milliseconds */

/* sensor SPI interface */
#define PIN_XENSIV_BGT60TRXX_SPI_SCLK       P2_2
#define PIN_XENSIV_BGT60TRXX_SPI_MOSI       P2_0
#define PIN_XENSIV_BGT60TRXX_SPI_MISO       P2_1
#define PIN_XENSIV_BGT60TRXX_SPI_CSN        P2_3

/* sensor interrupt output pin */
#define PIN_XENSIV_BGT60TRXX_IRQ            P2_5
/* sensor HW reset pin */
#define PIN_XENSIV_BGT60TRXX_RSTN           P2_7
/* enable 1V8 LDO on radar wingboard*/
// #define PIN_XENSIV_BGT60TRXX_LDO_EN         NO_LDO_EN_PIN_LDO_ALWAYS_ON

#define XENSIV_BGT60TRXX_SPI_FREQUENCY      (25000000UL)

#define NUM_SAMPLES_PER_FRAME               (XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS *\
                                             XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME *\
                                             XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP)

#define BINARY_FRAME_HEADER_VERSION         (1U)
#define BINARY_FRAME_SAMPLE_SIZE_BYTES      ((uint16_t)sizeof(uint16_t))

typedef struct __attribute__((packed))
{
    uint8_t magic[4];
    uint16_t version;
    uint16_t sample_size_bytes;
    uint32_t frame_index;
    uint32_t sample_count;
} binary_frame_header_t;

/*******************************************************************************
* Global Variables
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

/* Allocate enough memory for the radar dara frame. */
static uint16_t samples[NUM_SAMPLES_PER_FRAME];

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void usb_add_cdc(void);
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

/*******************************************************************************
* Function Name: main
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port (optional, for debug) */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* Initialize the User LED */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initializes the USB stack */
    USBD_Init();
    usb_add_cdc();
    USBD_SetDeviceInfo(&usb_deviceInfo);
    USBD_Start();

    /* Initialize Radar */
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
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Reduce drive strength to improve EMI */
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_MOSI),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_SCLK),
                        CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CY_GPIO_DRIVE_1_8);

    /* Set SPI data rate to communicate with sensor */
    result = cyhal_spi_set_frequency(&cyhal_spi, XENSIV_BGT60TRXX_SPI_FREQUENCY);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Enable the LDO. */
    // result = cyhal_gpio_init(PIN_XENSIV_BGT60TRXX_LDO_EN,
    //                          CYHAL_GPIO_DIR_OUTPUT,
    //                          CYHAL_GPIO_DRIVE_STRONG,
    //                          true);
    // CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Wait LDO stable */
    (void)cyhal_system_delay_ms(5);

    result = xensiv_bgt60trxx_mtb_init(&sensor,
                                       &cyhal_spi,
                                       PIN_XENSIV_BGT60TRXX_SPI_CSN,
                                       PIN_XENSIV_BGT60TRXX_RSTN,
                                       register_list,
                                       XENSIV_BGT60TRXX_CONF_NUM_REGS);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* The sensor will generate an interrupt once the sensor FIFO level is NUM_SAMPLES_PER_FRAME */
    result = xensiv_bgt60trxx_mtb_interrupt_init(&sensor,
                                                 NUM_SAMPLES_PER_FRAME,
                                                 PIN_XENSIV_BGT60TRXX_IRQ,
                                                 CYHAL_ISR_PRIORITY_DEFAULT,
                                                 xensiv_bgt60trxx_mtb_interrupt_handler,
                                                 NULL);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Ensure acquisition is idle until commanded via CLI */
    if (xensiv_bgt60trxx_start_frame(&sensor.dev, false) != XENSIV_BGT60TRXX_STATUS_OK)
    {
        CY_ASSERT(0);
    }

    uint32_t frame_idx = 0;

    for (;;)
    {
        /* Wait for configuration */
        if ((USBD_GetState() & USB_STAT_CONFIGURED) != USB_STAT_CONFIGURED)
        {
            cyhal_system_delay_ms(USB_CONFIG_DELAY);
            continue;
        }

        process_cli();

        if (!capture_enabled)
        {
            cyhal_system_delay_ms(1);
            continue;
        }

        /* Wait for the radar device to indicate the availability of the data to fetch. */
        /* Note: We also need to keep processing CLI in case of stop command */
        while ((capture_enabled == true) && (data_available == false))
        {
            process_cli();
            /* Check USB state to avoid hanging if disconnected */
            if ((USBD_GetState() & USB_STAT_CONFIGURED) != USB_STAT_CONFIGURED)
            {
                capture_enabled = false;
                break;
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
            cyhal_gpio_toggle(CYBSP_USER_LED);

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
    if (fmt == NULL) return;
    if (binary_stream_active) return;

    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    USBD_CDC_Write(usb_cdcHandle, buffer, strlen(buffer), 0);
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

    USBD_CDC_Write(usb_cdcHandle, &header, sizeof(header), 0);
    USBD_CDC_Write(usb_cdcHandle, samples, sizeof(samples[0]) * NUM_SAMPLES_PER_FRAME, 0);
}

static void process_cli(void)
{
    static char cmd_buffer[32];
    static uint32_t cmd_index = 0;
    char ch;

    /* Check if data is available in USB buffer */
    if (USBD_CDC_GetNumBytesInBuffer(usb_cdcHandle) > 0)
    {
        if (USBD_CDC_Read(usb_cdcHandle, &ch, 1, 0) == 1)
        {
            /* Echo back */
            USBD_CDC_Write(usb_cdcHandle, &ch, 1, 0);

            if ((ch == '\r') || (ch == '\n'))
            {
                if (cmd_index > 0)
                {
                    cmd_buffer[cmd_index] = '\0';
                    USBD_CDC_Write(usb_cdcHandle, "\r\n", 2, 0); /* New line for echo */
                    handle_command(cmd_buffer);
                    cmd_index = 0;
                }
            }
            else
            {
                if (cmd_index < (sizeof(cmd_buffer) - 1))
                {
                    cmd_buffer[cmd_index++] = ch;
                }
                else
                {
                    cmd_index = 0;
                }
            }
        }
    }
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
    else if (*cmd != '\0')
    {
        status_printf("Unknown command: %s\r\n", cmd);
    }
}

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