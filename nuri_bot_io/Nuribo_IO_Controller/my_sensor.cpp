#include "my_sensor.h"
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_MLX90614.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// 센서 객체
MAX30105 particleSensor;
Adafruit_MLX90614 mlx;

// OLED 객체
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 데이터 버퍼
uint32_t irBuffer[100], redBuffer[100];
int8_t validHR = 0, validSPO2 = 0;

// 전역 측정값
int32_t heartRate = 0;
int32_t spo2 = 0;
float temperature = 0.0;

void initSensors() {
  Wire.begin(21, 22);

  // OLED 초기화
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("❌ OLED 초기화 실패");
    while (1);
  }

  // MLX90614 초기화
  if (!mlx.begin()) {
    Serial.println("❌ MLX90614 연결 실패");
    while (1);
  }

  // MAX30105 초기화
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("❌ MAX30105 연결 실패");
    while (1);
  }
  particleSensor.setup(60, 4, 2, 100, 411, 4096);

  // 초기 버퍼 수집
  for (int i = 0; i < 100; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }
}

// 👉 OLED 출력 함수 추가
void displaySensorData() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("HR:");
  display.println(validHR ? heartRate : 0);

  display.setCursor(0, 20);
  display.print("Temp:");
  display.println(isnan(temperature) ? 0.0 : temperature, 1);

  display.setCursor(0, 40);
  display.print("Status:");
  if (validHR && validSPO2 && !isnan(temperature)) {
    display.println("OK");
  } else {
    display.println("Nan");
  }

  display.display();
}

void updateSensorValues() {
  // 이전 데이터 이동
  for (int i = 25; i < 100; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  // 새로운 샘플 수집
  for (int i = 75; i < 100; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // 측정값 계산
  maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer,
                                         &spo2, &validSPO2,
                                         &heartRate, &validHR);

  temperature = mlx.readObjectTempC();

  // OLED 출력 호출
  displaySensorData();
}