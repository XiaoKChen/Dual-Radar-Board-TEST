/*
 * A7-RADAR-TEST - UART bridge between PSoC6 dual-radar board and host PC.
 *
 * USB Serial (Serial)  @115200 baud : commands + output to host.
 * Hardware UART (Serial1, D0/D1) @115200 baud : link to PSoC6.
 *
 * Default mode is BINARY (readable JSON out). NORMAL is byte-passthrough for
 * debug, but the PSoC's 32-byte frames are mostly non-printable bytes, so
 * the Serial Monitor will look empty — that's expected.
 *
 * Commands (line-terminated by \n or \r, case-insensitive — set Serial
 * Monitor's line ending to "Newline" or "Both NL & CR"):
 *   normal  -> forward every byte from PSoC6 verbatim to Serial.
 *   binary  -> decode PSoC6 32-byte presence frames and emit JSON.
 *   stat    -> print current mode, total bytes received, last-frame age.
 *
 * Binary frame V2 (little-endian, 32 bytes, every ~3 s):
 *   [0xAA][0x55][LEN=26:u16][TYPE=0x02:u8][SEQ:u8]
 *   [record0: 12 bytes][record1: 12 bytes]
 *   [CRC16:u16]   CRC-16/CCITT-FALSE over bytes [0..29]
 *
 * Record layout (12 bytes):
 *   id:u8 | flags:u8 | range_bin:u16 | doppler_bin:i16
 *   distance_mm:u16 | angle_cdeg:i16 | reserved:u16
 */

#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>

static const uint32_t USB_BAUD  = 115200;
static const uint32_t LINK_BAUD = 115200;

static const uint8_t  SYNC0       = 0xAA;
static const uint8_t  SYNC1       = 0x55;
static const uint8_t  TYPE_V2     = 0x02;
static const uint16_t PAYLOAD_LEN = 26;
static const uint16_t FRAME_LEN   = 32;
static const uint16_t REC_SIZE    = 12;
static const uint16_t REC0_OFF    = 6;
static const uint16_t REC1_OFF    = 6 + 12;
static const uint16_t RANGE_NONE    = 0xFFFF;
static const int16_t  DOPPLER_NONE  = INT16_MIN;
static const uint16_t DISTANCE_NONE = 0xFFFF;
static const int16_t  ANGLE_NONE    = INT16_MIN;
static const uint8_t  FLAG_MACRO  = 0x01;
static const uint8_t  FLAG_MICRO  = 0x02;

enum Mode { MODE_NORMAL, MODE_BINARY };
static Mode mode = MODE_BINARY;

/* Link liveness counters — let "stat" and the stale-link watchdog tell the
   difference between "PSoC silent" and "PSoC sending unprintable bytes". */
static uint32_t link_bytes_total      = 0;
static uint32_t link_last_byte_ms     = 0;
static uint32_t link_last_frame_ms    = 0;
static uint32_t link_last_warn_ms     = 0;
static const uint32_t STALE_WARN_MS   = 5000;

static const uint16_t CMD_BUF_SIZE = 32;
static char     cmd_buf[CMD_BUF_SIZE];
static uint16_t cmd_len = 0;

enum RxState {
    WAIT_SYNC0,
    WAIT_SYNC1,
    READ_LEN0,
    READ_LEN1,
    READ_PAYLOAD,
    READ_CRC0,
    READ_CRC1
};

static RxState  rx_state    = WAIT_SYNC0;
static uint16_t rx_len      = 0;        /* declared LEN from header */
static uint16_t rx_received = 0;        /* payload bytes received so far */
static uint16_t rx_crc      = 0;        /* CRC bytes assembled from wire */
/* frame[] mirrors the wire layout starting at SYNC0 so we can CRC it directly. */
static uint8_t  rx_frame[FRAME_LEN];

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
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

static void str_tolower_inplace(char *s) {
    for (; *s; ++s) *s = (char)tolower((unsigned char)*s);
}

static void print_stat(void) {
    uint32_t now = millis();
    Serial.print("[stat] mode=");
    Serial.print(mode == MODE_BINARY ? "binary" : "normal");
    Serial.print(" rx_bytes=");
    Serial.print((unsigned long)link_bytes_total);
    Serial.print(" last_byte_age_ms=");
    Serial.print(link_last_byte_ms ? (unsigned long)(now - link_last_byte_ms) : 0UL);
    Serial.print(" last_frame_age_ms=");
    Serial.println(link_last_frame_ms ? (unsigned long)(now - link_last_frame_ms) : 0UL);
}

static void apply_command(char *line) {
    while (*line == ' ' || *line == '\t') line++;
    size_t n = strlen(line);
    while (n > 0 && (line[n-1] == ' ' || line[n-1] == '\t')) line[--n] = '\0';
    if (n == 0) return;
    str_tolower_inplace(line);
    if (strcmp(line, "normal") == 0) {
        mode = MODE_NORMAL;
        Serial.println("[mode] normal");
    } else if (strcmp(line, "binary") == 0) {
        mode = MODE_BINARY;
        rx_state = WAIT_SYNC0;
        Serial.println("[mode] binary");
    } else if (strcmp(line, "stat") == 0 || strcmp(line, "status") == 0 || strcmp(line, "?") == 0) {
        print_stat();
    } else {
        Serial.print("[mode] unknown command: ");
        Serial.println(line);
    }
}

static void poll_usb_commands(void) {
    while (Serial.available()) {
        int c = Serial.read();
        if (c < 0) break;
        if (c == '\r' || c == '\n') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                apply_command(cmd_buf);
                cmd_len = 0;
            }
        } else if (cmd_len + 1 < CMD_BUF_SIZE) {
            cmd_buf[cmd_len++] = (char)c;
        } else {
            cmd_len = 0;  /* overrun - drop line */
        }
    }
}

static void emit_record_json(const uint8_t *rec) {
    uint8_t  id       = rec[0];
    uint8_t  flags    = rec[1];
    uint16_t range    = (uint16_t)rec[2] | ((uint16_t)rec[3] << 8);
    int16_t  doppler  = (int16_t)((uint16_t)rec[4] | ((uint16_t)rec[5] << 8));
    uint16_t dist_mm  = (uint16_t)rec[6] | ((uint16_t)rec[7] << 8);
    int16_t  ang_cdeg = (int16_t)((uint16_t)rec[8] | ((uint16_t)rec[9] << 8));

    Serial.print("{\"id\":");
    Serial.print((int)id);
    Serial.print(",\"macro\":");
    Serial.print((flags & FLAG_MACRO) ? "true" : "false");
    Serial.print(",\"micro\":");
    Serial.print((flags & FLAG_MICRO) ? "true" : "false");
    Serial.print(",\"range\":");
    if (range == RANGE_NONE) Serial.print("null");
    else                     Serial.print((unsigned int)range);
    Serial.print(",\"doppler\":");
    if (doppler == DOPPLER_NONE) Serial.print("null");
    else                         Serial.print((int)doppler);
    Serial.print(",\"distance_m\":");
    if (dist_mm == DISTANCE_NONE) Serial.print("null");
    else                          Serial.print(dist_mm / 1000.0f, 3);
    Serial.print(",\"angle_deg\":");
    if (ang_cdeg == ANGLE_NONE) Serial.print("null");
    else                        Serial.print(ang_cdeg / 100.0f, 2);
    Serial.print("}");
}

static void emit_frame_json(uint8_t seq) {
    Serial.print("{\"seq\":");
    Serial.print((unsigned int)seq);
    Serial.print(",\"s\":[");
    emit_record_json(&rx_frame[REC0_OFF]);
    Serial.print(",");
    emit_record_json(&rx_frame[REC1_OFF]);
    Serial.println("]}");
}

static void handle_byte(uint8_t b) {
    switch (rx_state) {
    case WAIT_SYNC0:
        if (b == SYNC0) { rx_frame[0] = b; rx_state = WAIT_SYNC1; }
        /* else: drop and stay in WAIT_SYNC0 - this is the re-sync path */
        break;
    case WAIT_SYNC1:
        if (b == SYNC1) { rx_frame[1] = b; rx_state = READ_LEN0; }
        else if (b == SYNC0) { rx_frame[0] = b; /* stay */ }
        else { rx_state = WAIT_SYNC0; }
        break;
    case READ_LEN0:
        rx_frame[2] = b;
        rx_len = b;
        rx_state = READ_LEN1;
        break;
    case READ_LEN1:
        rx_frame[3] = b;
        rx_len |= (uint16_t)b << 8;
        if (rx_len != PAYLOAD_LEN) {
            Serial.print("{\"err\":\"len\",\"got\":");
            Serial.print((unsigned int)rx_len);
            Serial.println("}");
            rx_state = WAIT_SYNC0;
        } else {
            rx_received = 0;
            rx_state = READ_PAYLOAD;
        }
        break;
    case READ_PAYLOAD:
        rx_frame[4 + rx_received] = b;
        rx_received++;
        if (rx_received >= rx_len) {
            rx_state = READ_CRC0;
        }
        break;
    case READ_CRC0:
        rx_frame[FRAME_LEN - 2] = b;
        rx_crc = b;
        rx_state = READ_CRC1;
        break;
    case READ_CRC1: {
        rx_frame[FRAME_LEN - 1] = b;
        rx_crc |= (uint16_t)b << 8;
        uint16_t calc = crc16_ccitt_false(rx_frame, FRAME_LEN - 2);
        uint8_t type = rx_frame[4];
        uint8_t seq  = rx_frame[5];
        if (calc == rx_crc && type == TYPE_V2) {
            link_last_frame_ms = millis();
            emit_frame_json(seq);
        } else if (calc != rx_crc) {
            Serial.print("{\"err\":\"crc\",\"seq\":");
            Serial.print((unsigned int)seq);
            Serial.println("}");
        } else {
            Serial.print("{\"err\":\"type\",\"got\":");
            Serial.print((unsigned int)type);
            Serial.println("}");
        }
        rx_state = WAIT_SYNC0;
        break;
    }
    }
}

static void poll_link(void) {
    while (Serial1.available()) {
        int c = Serial1.read();
        if (c < 0) break;
        link_bytes_total++;
        link_last_byte_ms = millis();
        if (mode == MODE_NORMAL) {
            Serial.write((uint8_t)c);
        } else {
            handle_byte((uint8_t)c);
        }
    }
}

static void poll_link_watchdog(void) {
    uint32_t now = millis();
    uint32_t reference = link_last_byte_ms ? link_last_byte_ms : link_last_warn_ms;
    if (reference == 0) reference = now;  /* first 5 s after boot are free */
    if ((now - reference) >= STALE_WARN_MS &&
        (now - link_last_warn_ms) >= STALE_WARN_MS) {
        Serial.print("{\"warn\":\"no_link\",\"age_ms\":");
        Serial.print((unsigned long)(now - reference));
        Serial.println("}");
        link_last_warn_ms = now;
    }
}

void setup() {
    Serial.begin(USB_BAUD);
    Serial1.begin(LINK_BAUD);
    Serial.println("[boot] A7-RADAR-TEST ready (default: binary).");
    Serial.println("[boot] Commands: normal | binary | stat. Set line ending to Newline.");
}

void loop() {
    poll_usb_commands();
    poll_link();
    poll_link_watchdog();
}
