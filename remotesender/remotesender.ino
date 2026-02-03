/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

#define PIN_NEO_PIXEL 32  // The ESP32 pin GPIO16 connected to NeoPixel
#define NUM_PIXELS 32     // The number of LEDs (pixels) on NeoPixel

// Change to 0x69 if AD0 is pulled high on your board
#define MPU9250_ADDR 0x68

// Important MPU-9250 registers
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H 0x43

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readRegisters(uint8_t startReg, uint8_t count, uint8_t *buffer) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);  // repeated start

  Wire.requestFrom(MPU9250_ADDR, count);
  uint8_t i = 0;
  while (Wire.available() && i < count) {
    buffer[i++] = Wire.read();
  }
}

int16_t toInt16(uint8_t high, uint8_t low) {
  return (int16_t)((high << 8) | low);
}

Adafruit_NeoPixel Pixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
//uint8_t broadcastAddress[] = { 0x30, 0xAE, 0xA4, 0x05, 0xBD, 0x3C };
uint8_t broadcastAddress[] = { 0x98, 0xCD, 0xAC, 0x50, 0x33, 0xC8 };

typedef struct {
  uint8_t leftSpeed;
  uint8_t leftDirection;
  uint8_t rightSpeed;
  uint8_t rightDirection;
} message_struct;

message_struct message;

esp_now_peer_info_t peerInfo;


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  // Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.print(macStr);
  // Serial.print(" send status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

int xyToLed(int x, int y) {
  if (y % 2 == 0) {
    return y * 8 + x;
  } else {
    return y * 8 + (7 - x);
  }
}

void setup() {

  Pixel.begin();

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  Wire.begin();           // SDA=21, SCL=22 on ESP32
  Wire.setClock(400000);  // 400 kHz I2C

  // Wake up the MPU-9250 (clear SLEEP bit)
  writeRegister(MPU9250_PWR_MGMT_1, 0x00);
  delay(100);

  // Check WHO_AM_I
  uint8_t whoami = 0;
  readRegisters(MPU9250_WHO_AM_I, 1, &whoami);

  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoami, HEX);

  if (whoami != 0x71) {
    Serial.println("Unexpected WHO_AM_I (not 0x71). Check wiring/address.");
  } else {
    Serial.println("MPU-9250 detected.");
  }

  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));

  // register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // register first peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void ReadSensor(int16_t *xp, int16_t *yp) {
  uint8_t rawData[6];
  readRegisters(MPU9250_ACCEL_XOUT_H, 6, rawData);

  int16_t x = toInt16(rawData[0], rawData[1]);
  int16_t y = toInt16(rawData[2], rawData[3]);

  if (x > 10000) {
    x = 10000;
  }
  if (x < -10000) {
    x = -10000;
  }
  if (y > 10000) {
    y = 10000;
  }
  if (y < -10000) {
    y = -10000;
  }
  *xp = x;
  *yp = y;
}

void loop() {
  int16_t x, y, delta;
  ReadSensor(&x, &y);

  // deadzone zodat het stil kan staan
  if (abs(x) < 500) x = 0;
  if (abs(y) < 500) y = 0;

  // Clear all pixels
  Pixel.clear();
  
  // SWAPPED: Y-axis determines ROW (left/right tilt)
  int row = map(y, -10000, 10000, 3, 0);
  
  // SWAPPED: X-axis determines COLUMN (forward/back position)
  int col = map(x, -10000, 10000, 0, 7);
  
  // Compensate for snaking - flip column for odd rows so all rows go the same direction
  int actualCol = (row % 2 == 1) ? (7 - col) : col;
  int ledIndex = xyToLed(actualCol, row);

  // Light up main LED plus trail
  Pixel.setPixelColor(ledIndex, Pixel.Color(100, 0, 0)); 
  int prevCol = (row % 2 == 1) ? (actualCol < 7 ? actualCol + 1 : -1) : (actualCol > 0 ? actualCol - 1 : -1);
  int nextCol = (row % 2 == 1) ? (actualCol > 0 ? actualCol - 1 : -1) : (actualCol < 7 ? actualCol + 1 : -1);
  // dimmere lichtjes, vooran & achteraan
  // if (prevCol >= 0) Pixel.setPixelColor(xyToLed(prevCol, row), Pixel.Color(10, 10, 10));
  // if (nextCol >= 0) Pixel.setPixelColor(xyToLed(nextCol, row), Pixel.Color(10, 10, 10));
  
  Pixel.setBrightness(20);
  Pixel.show();

  // Rest of your motor control code
  uint8_t left = (uint8_t)map(abs(y), 0, 10000, 100, 250);
  uint8_t right = (uint8_t)map(abs(y), 0, 10000, 100, 250);
  delta = (int8_t)map(x, -10000, 10000, -75, 75);
  left = max(0, min(left - delta, 250));
  right = max(0, min(right + delta, 250));

  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(delta);

  message.leftDirection = message.rightDirection = y < 0;
  message.leftSpeed = left;
  message.rightSpeed = right;

  esp_err_t result = esp_now_send(0, (uint8_t *)&message, sizeof(message));
  delay(10);
}