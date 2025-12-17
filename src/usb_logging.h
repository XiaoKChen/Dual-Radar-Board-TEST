#ifndef USB_LOGGING_H
#define USB_LOGGING_H

#include <stdint.h>
#include <stdbool.h>

void status_printf(const char *fmt, ...);
void init_usb_logging(void);
void wait_for_usb_configured(void);

#endif /* USB_LOGGING_H */
