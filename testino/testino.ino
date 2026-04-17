// Reads presence detection output from PSoC6 via Serial1 (pins 0=RX, 1=TX)
// and forwards it to the PC over USB Serial for monitoring.
// PSoC6 baud: 115200, 8N1

#define PSOC_BAUD 115200

void setup() {
    Serial.begin(115200);
    Serial1.begin(PSOC_BAUD);
    Serial.println("PSoC6 bridge ready.");
}

void loop() {
    while (Serial1.available()) {
        Serial.write(Serial1.read());
    }
    while (Serial.available()) {
        Serial1.write(Serial.read());
    }
}
