#include "my_sensor.h"
#include "my_tcp.h"

void setup() {
  Serial.begin(115200);
  initSensors();
  setupWiFiAndHandshake();  // 서버 연결 및 핸드셰이크
}

void loop() {
  sendSensorData(0x10, 0x01);  // OPCODE=0x10, RESIDENT_ID=0x01
  delay(1000);
}