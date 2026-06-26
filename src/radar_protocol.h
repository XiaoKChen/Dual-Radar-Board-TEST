#ifndef RADAR_PROTOCOL_H
#define RADAR_PROTOCOL_H

#include <stdint.h>

/* Wire format (little-endian, fixed 32-byte frame for TYPE_PRESENCE_V2):
 *
 *   [0xAA][0x55][LEN:u16=26][TYPE:u8=0x02][SEQ:u8]
 *   [record0: 12 bytes][record1: 12 bytes]
 *   [CRC16:u16]   CRC-16/CCITT-FALSE over bytes [SYNC0 .. last payload byte]
 *
 * LEN counts TYPE+SEQ+payload (excludes SYNC, LEN, CRC).
 * Re-sync by scanning the stream for SYNC0 SYNC1.
 *
 * V2 adds distance_mm (u16, mm) and angle_cdeg (i16, hundredths of a degree)
 * to each record. TYPE was bumped from 0x01 so an old/new firmware mismatch
 * surfaces as a clean type error instead of silently misaligning fields.
 */

#define RADAR_PROTO_SYNC0                   0xAA
#define RADAR_PROTO_SYNC1                   0x55
#define RADAR_PROTO_TYPE_PRESENCE_V2        0x02
#define RADAR_PROTO_PRESENCE_PAYLOAD_LEN    26      /* TYPE+SEQ+2 records */
#define RADAR_PROTO_PRESENCE_FRAME_LEN      32      /* total wire bytes */

#define RADAR_FLAG_MACRO_PRESENT            0x01
#define RADAR_FLAG_MICRO_PRESENT            0x02

#define RADAR_RANGE_BIN_NONE                ((uint16_t)0xFFFF)
#define RADAR_DOPPLER_BIN_NONE              ((int16_t)INT16_MIN)
#define RADAR_DISTANCE_MM_NONE              ((uint16_t)0xFFFF)
#define RADAR_ANGLE_CDEG_NONE               ((int16_t)INT16_MIN)

typedef struct __attribute__((packed)) {
    uint8_t  sensor_id;
    uint8_t  flags;
    uint16_t range_bin;
    int16_t  doppler_bin;
    uint16_t distance_mm;     /* 0..65534 mm, 0xFFFF = no target */
    int16_t  angle_cdeg;      /* hundredths of a degree, INT16_MIN = no target */
    uint16_t reserved;        /* must be 0 — keeps record 4-byte aligned */
} radar_sensor_record_t;

_Static_assert(sizeof(radar_sensor_record_t) == 12,
               "radar_sensor_record_t must be exactly 12 bytes on the wire");

#endif /* RADAR_PROTOCOL_H */
