/*
 * A7-RADAR-TEST — UART bridge between PSoC6 dual-radar board and host PC.
 *
 * USB Serial (Serial)  @115200 baud : commands + output to host.
 * Hardware UART (Serial1, D0/D1) @115200 baud : link to PSoC6.
 *   The PSoC6 retarget-io UART runs at CY_RETARGET_IO_BAUDRATE = 115200.
 *
 * Commands (line-terminated by \n or \r, case-insensitive):
 *   normal  -> forward every byte from PSoC6 verbatim to Serial.
 *   binary  -> parse PSoC6 presence JSON and emit "Radar N : X".
 *
 * PSoC6 emits one pair every ~3 s in PrintTask (src/main.c):
 *   {"radar":0,"distance_m":1.23,"angle_deg":12.3,"presence":true}
 *   {"radar":1,...,"presence":false}
 * We emit on every matched line because PSoC6 already throttles.
 */

#include <Arduino.h>
#include <string.h>
#include <ctype.h>

static const uint32_t USB_BAUD  = 115200;
static const uint32_t LINK_BAUD = 115200;

enum Mode { MODE_NORMAL, MODE_BINARY };
static Mode mode = MODE_NORMAL;

static const uint16_t CMD_BUF_SIZE  = 32;
static char     cmd_buf[CMD_BUF_SIZE];
static uint16_t cmd_len = 0;

static const uint16_t LINK_BUF_SIZE = 160;
static char     link_buf[LINK_BUF_SIZE];
static uint16_t link_len = 0;

static void str_tolower_inplace(char *s) {
    for (; *s; ++s) *s = (char)tolower((unsigned char)*s);
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
        Serial.println("[mode] binary");
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
            cmd_len = 0;  /* overrun — drop line */
        }
    }
}

static void parse_and_emit_binary(const char *line) {
    /* Expected: {"radar":N,...,"presence":true|false} where N is 0 or 1. */
    const char *radar_tag = strstr(line, "\"radar\":");
    if (radar_tag == NULL) return;
    const char *p = radar_tag + 8;
    while (*p == ' ') p++;
    if (*p != '0' && *p != '1') return;
    char radar_index = (char)(*p - '0');

    const char *presence_tag = strstr(line, "\"presence\":");
    if (presence_tag == NULL) return;
    const char *q = presence_tag + 11;
    while (*q == ' ') q++;

    int state;
    if (strncmp(q, "true", 4) == 0)       state = 1;
    else if (strncmp(q, "false", 5) == 0) state = 0;
    else return;

    Serial.print("Radar ");
    Serial.print((int)(radar_index + 1));
    Serial.print(" : ");
    Serial.println(state);
}

static void poll_link(void) {
    while (Serial1.available()) {
        int c = Serial1.read();
        if (c < 0) break;
        if (mode == MODE_NORMAL) {
            Serial.write((uint8_t)c);
            continue;
        }
        /* MODE_BINARY: accumulate full line, then parse */
        if (c == '\r' || c == '\n') {
            if (link_len > 0) {
                link_buf[link_len] = '\0';
                parse_and_emit_binary(link_buf);
                link_len = 0;
            }
        } else if (link_len + 1 < LINK_BUF_SIZE) {
            link_buf[link_len++] = (char)c;
        } else {
            link_len = 0;  /* overrun — drop line */
        }
    }
}

void setup() {
    Serial.begin(USB_BAUD);
    Serial1.begin(LINK_BAUD);
    /* Do not block waiting for USB CDC — UNO R4 Minima Serial is real USB CDC. */
    Serial.println("[boot] A7-RADAR-TEST ready. Commands: normal | binary");
}

void loop() {
    poll_usb_commands();
    poll_link();
}
