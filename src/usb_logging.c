#include "usb_logging.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

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

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflect, no xorout.
   Table-free — the per-frame budget is 22 bytes every 3 s, so the loop cost
   is negligible compared with avoiding a 512 B lookup table in .rodata. */
static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void usb_log_presence_binary(const radar_sensor_record_t records[2])
{
    static uint8_t seq = 0;
    uint8_t frame[RADAR_PROTO_PRESENCE_FRAME_LEN];
    const size_t rec_size = sizeof(radar_sensor_record_t);
    const size_t rec_off = 6;  /* SYNC0+SYNC1+LEN(2)+TYPE+SEQ */

    frame[0] = RADAR_PROTO_SYNC0;
    frame[1] = RADAR_PROTO_SYNC1;
    frame[2] = (uint8_t)(RADAR_PROTO_PRESENCE_PAYLOAD_LEN & 0xFFu);
    frame[3] = (uint8_t)((RADAR_PROTO_PRESENCE_PAYLOAD_LEN >> 8) & 0xFFu);
    frame[4] = RADAR_PROTO_TYPE_PRESENCE_V3;
    frame[5] = seq++;

    memcpy(&frame[rec_off],            &records[0], rec_size);
    memcpy(&frame[rec_off + rec_size], &records[1], rec_size);

    uint16_t crc = crc16_ccitt_false(frame, RADAR_PROTO_PRESENCE_FRAME_LEN - 2);
    frame[RADAR_PROTO_PRESENCE_FRAME_LEN - 2] = (uint8_t)(crc & 0xFFu);
    frame[RADAR_PROTO_PRESENCE_FRAME_LEN - 1] = (uint8_t)((crc >> 8) & 0xFFu);

    /* Single contiguous write — fixes the choppy/garbled output the R4 saw
       when the old text path emitted bytes in printf-sized chunks. */
    fwrite(frame, 1, RADAR_PROTO_PRESENCE_FRAME_LEN, stdout);
    fflush(stdout);
}
