#include "usb_logging.h"
#include <stdio.h>
#include <stdarg.h>

void init_usb_logging(void) {}

void wait_for_usb_configured(void) {}

void status_printf(const char *fmt, ...)
{
    if (fmt == NULL) return;
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}
