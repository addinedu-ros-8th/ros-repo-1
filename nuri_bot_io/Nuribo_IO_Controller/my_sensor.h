#ifndef MY_SENSOR_H
#define MY_SENSOR_H

#include <Arduino.h>

extern int32_t heartRate;
extern int32_t spo2;
extern float temperature;

void initSensors();
void updateSensorValues();

#endif