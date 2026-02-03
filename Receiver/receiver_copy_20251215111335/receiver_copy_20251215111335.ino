// receiver.ino

#include <WiFi.h>
#include <esp_now.h>

class Motor {
public:
  uint8_t brakePin;
  uint8_t pwmPin;
  uint8_t directionPin;
  Motor(uint8_t brakePin, uint8_t pwmPin, uint8_t directionPin);
};

Motor::Motor(uint8_t brakePin, uint8_t pwmPin, uint8_t directionPin) {
  this->brakePin = brakePin;
  this->pwmPin = pwmPin;
  this->directionPin = directionPin;

  pinMode(this->brakePin, OUTPUT);
  pinMode(this->pwmPin, OUTPUT);
  pinMode(this->directionPin, OUTPUT);
}

Motor leftMotor(26, 14, 13);
Motor rightMotor(27, 25, 12);

uint8_t sender[] = { 0xc4, 0xdd, 0x57, 0x9c, 0xda, 0xfc };

typedef struct struct_message {
  uint8_t leftSpeed;
  uint8_t leftDirection;
  uint8_t rightSpeed;
  uint8_t rightDirection;
};

struct_message message;

esp_now_peer_info_t peerInfo;


void OnDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;  // sender MAC

  //Serial.print("From: ");
  for (int i = 0; i < 6; i++) {
    //Serial.printf("%02X%s", mac[i], (i < 5) ? ":" : "");
  }
  //Serial.printf("  len=%d\n", len);

  memcpy(&message, data, len);
  //Serial.printf("%d %d %d %d \n", message.leftDirection, message.leftSpeed, message.rightDirection, message.rightSpeed);

  analogWrite(leftMotor.pwmPin, message.leftSpeed > 105 ? message.leftSpeed : 0);
  digitalWrite(leftMotor.directionPin, message.leftDirection);
  analogWrite(rightMotor.pwmPin, message.rightSpeed > 105 ? message.rightSpeed : 0);
  digitalWrite(rightMotor.directionPin, message.rightDirection);

  // use data[0..len-1]
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin();
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP now is broken");
    return;
  } else {
    Serial.println("ESP now is working!");
  }


  memcpy(peerInfo.peer_addr, sender, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_register_recv_cb(OnDataReceive);

  analogWrite(leftMotor.pwmPin, 0);
  analogWrite(rightMotor.pwmPin, 0);
  digitalWrite(leftMotor.brakePin, LOW);
  digitalWrite(rightMotor.brakePin, LOW);
  digitalWrite(leftMotor.directionPin, HIGH);
  digitalWrite(rightMotor.directionPin, HIGH);
}

void loop() {
}
