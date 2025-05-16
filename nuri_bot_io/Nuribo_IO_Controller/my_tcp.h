#ifndef MY_TCP_H
#define MY_TCP_H

#include <Arduino.h>

void setupWiFiAndHandshake();
void sendSensorData(uint8_t opcode, uint8_t resident_id);

#endif