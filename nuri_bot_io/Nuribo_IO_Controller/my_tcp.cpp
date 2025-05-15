#include "my_tcp.h"
#include "my_sensor.h"
#include <WiFi.h>
#include <WiFiClient.h>

const char* ssid = "AIE_509_2.4G";
const char* password = "addinedu_class1";
const char* server_ip = "192.168.0.43";
const uint16_t server_port = 9999;

const uint8_t CON_OPCODE = 0x00;

WiFiClient client;

uint32_t toBigEndian32(uint32_t val) {
  return ((val & 0x000000FF) << 24) |
         ((val & 0x0000FF00) << 8)  |
         ((val & 0x00FF0000) >> 8)  |
         ((val & 0xFF000000) >> 24);
}

uint64_t toBigEndian64(uint64_t val) {
  return ((val & 0x00000000000000FFULL) << 56) |
         ((val & 0x000000000000FF00ULL) << 40) |
         ((val & 0x0000000000FF0000ULL) << 24) |
         ((val & 0x00000000FF000000ULL) << 8 ) |
         ((val & 0x000000FF00000000ULL) >> 8 ) |
         ((val & 0x0000FF0000000000ULL) >> 24) |
         ((val & 0x00FF000000000000ULL) >> 40) |
         ((val & 0xFF00000000000000ULL) >> 56);
}

void floatToBigEndianBytes(float val, uint8_t* dest) {
  union {
    float f;
    uint32_t i;
  } u;
  u.f = val;
  uint32_t be = toBigEndian32(u.i);
  memcpy(dest, &be, 4);
}

void setupWiFiAndHandshake() {
  Serial.println("🔌 Connecting to WiFi...");
  WiFi.begin(ssid, password);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("❌ WiFi Connection Failed");
    return;
  }

  if (client.connect(server_ip, server_port)) {
    Serial.println("🤝 Connected to server for handshake");

    const char* device_str = "device";
    uint32_t resident_id_be = toBigEndian32(0x01);
    uint16_t str_len = strlen(device_str) + 1;

    uint8_t payload[15];
    size_t offset = 0;

    payload[offset++] = CON_OPCODE;
    memcpy(payload + offset, &str_len, 2); offset += 2;
    memcpy(payload + offset, device_str, 6); offset += 6;
    memcpy(payload + offset, &resident_id_be, 4); offset += 4;

    uint32_t total_len_be = toBigEndian32(offset);
    uint8_t packet[4 + offset];
    memcpy(packet, &total_len_be, 4);
    memcpy(packet + 4, payload, offset);

    client.write(packet, 4 + offset);
    Serial.println("📦 Handshake Packet Sent");
  } else {
    Serial.println("❌ Server connection failed.");
  }
}

void sendSensorData(uint8_t opcode, uint8_t resident_id) {
  updateSensorValues();  // 센서값 갱신

  uint64_t timestamp_be = toBigEndian64(millis());
  uint32_t heart_be = toBigEndian32(heartRate);
  uint32_t spo2_be = toBigEndian32(spo2);
  uint8_t temp_be[4];
  floatToBigEndianBytes(temperature, temp_be);

  uint8_t payload[22];
  size_t offset = 0;
  payload[offset++] = opcode;
  payload[offset++] = resident_id;
  memcpy(payload + offset, &heart_be, 4); offset += 4;
  memcpy(payload + offset, &spo2_be, 4); offset += 4;
  memcpy(payload + offset, temp_be, 4); offset += 4;
  memcpy(payload + offset, &timestamp_be, 8); offset += 8;

  uint32_t total_len_be = toBigEndian32(offset);
  uint8_t packet[4 + offset];
  memcpy(packet, &total_len_be, 4);
  memcpy(packet + 4, payload, offset);

  // 시리얼 출력: 센서 측정값
  Serial.print("📡 HR: ");
  Serial.print(heartRate);
  Serial.print(" | SpO2: ");
  Serial.print(spo2);
  Serial.print(" | Temp: ");
  Serial.println(temperature, 1);

  // 시리얼 출력: Payload (HEX)
  Serial.print("🧾 Payload: [");
  for (size_t i = 0; i < offset; i++) {
    if (i > 0) Serial.print(" ");
    if (payload[i] < 0x10) Serial.print("0");
    Serial.print(payload[i], HEX);
  }
  Serial.println("]");

  // 시리얼 출력: Full Packet (Length + Payload)
  Serial.print("📦 Full Packet: [");
  for (size_t i = 0; i < 4 + offset; i++) {
    if (i > 0) Serial.print(" ");
    if (packet[i] < 0x10) Serial.print("0");
    Serial.print(packet[i], HEX);
  }
  Serial.println("]");

  client.write(packet, 4 + offset);
  Serial.println("✅ Packet sent");
}