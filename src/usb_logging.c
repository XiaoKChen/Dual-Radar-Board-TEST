#include "usb_logging.h"
#include "USB.h"
#include "USB_CDC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static USB_CDC_HANDLE usb_cdcHandle;

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

void init_usb_logging(void) {
    static U8             OutBuffer[USB_FS_BULK_MAX_PACKET_SIZE];
    USB_CDC_INIT_DATA     InitData;
    USB_ADD_EP_INFO       EPBulkIn;
    USB_ADD_EP_INFO       EPBulkOut;
    USB_ADD_EP_INFO       EPIntIn;

    /* Initialize the USB stack */
    USBD_Init();

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
    
    USBD_SetDeviceInfo(&usb_deviceInfo);
    USBD_Start();
}

void wait_for_usb_configured(void) {
    uint32_t timeout = 0;
    while ((USBD_GetState() & USB_STAT_CONFIGURED) != USB_STAT_CONFIGURED)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        timeout++;
        if (timeout > 200)  /* 10 seconds timeout */
        {
            break;
        }
    }
}

void status_printf(const char *fmt, ...)
{
    if (fmt == NULL) return;

    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    
    /* Check if scheduler is running to use mutex */
    /* Note: Mutex protection removed for simplicity in this module, 
       ensure single writer or add mutex if needed */

    if ((USBD_GetState() & USB_STAT_CONFIGURED) == USB_STAT_CONFIGURED) {
        uint32_t len = strlen(buffer);
        /* Non-blocking write - if it fails, we just drop the message */
        USBD_CDC_Write(usb_cdcHandle, (uint8_t *)buffer, len, 0);
    }
}
