#ifndef USB_LOGGING_H
#define USB_LOGGING_H

#include <stdint.h>
#include <stdbool.h>

#include "radar_protocol.h"

void status_printf(const char *fmt, ...);
void init_usb_logging(void);
void wait_for_usb_configured(void);

/* Emits one TYPE_PRESENCE_V1 frame (24 bytes) in a single fwrite + fflush so
 * the receiver sees an atomic chunk rather than the byte-interleaving the old
 * printf path produced. records[0] is sensor 0, records[1] is sensor 1. */
void usb_log_presence_binary(const radar_sensor_record_t records[2]);

#endif /* USB_LOGGING_H */
