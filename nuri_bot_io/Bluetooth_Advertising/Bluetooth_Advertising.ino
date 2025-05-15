#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>

void setup() {
  Serial.begin(115200);

  BLEDevice::init("lim_BLE"); // 디바이스 이름
  
  BLEServer *pServer = BLEDevice::createServer();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  
  BLEUUID serviceUUID("12345678-1234-5678-1234-56789abcdef0"); // UUID 추가
  pAdvertising->addServiceUUID(serviceUUID);
  
  pAdvertising->setScanResponse(true); // 스캔 응답 활성화
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMinPreferred(0x12);  

  pAdvertising->start();
  Serial.println("BLE Advertising Started!");
}

void loop() {
  delay(1000);
}