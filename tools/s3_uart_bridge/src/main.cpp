// USB (Serial-JTAG CDC) <-> UART1 transparent bridge, 115200 8N1.
#include <Arduino.h>

static uint8_t buf[512];

void setup() {
  Serial.begin(115200);  // USB CDC
  Serial1.setRxBufferSize(4096);
  Serial1.begin(115200, SERIAL_8N1, BRIDGE_RX_PIN, BRIDGE_TX_PIN);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  int n = Serial1.available();
  if (n > 0) {
    n = Serial1.read(buf, n > (int)sizeof(buf) ? sizeof(buf) : n);
    Serial.write(buf, n);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  n = Serial.available();
  if (n > 0) {
    n = Serial.read(buf, n > (int)sizeof(buf) ? sizeof(buf) : n);
    Serial1.write(buf, n);
  }
}
