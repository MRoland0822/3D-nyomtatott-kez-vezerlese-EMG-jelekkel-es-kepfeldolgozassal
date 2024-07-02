#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <math.h>

const char *ssid = "Connect at your own risk"; 
const char *password = "password123";    

WiFiServer server(80);  

#define SERVO_DRIVER_ADDR 0x40

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(SERVO_DRIVER_ADDR);

const uint8_t servoChannels[] = {8, 9, 10, 11, 12}; 
const uint8_t servoCount = sizeof(servoChannels) / sizeof(servoChannels[0]); 
int servoMin = 150; 
int servoMax = 600; 

const int muscleSensorPin = 34;
const int numReadings = 5; 
int muscleReadings[numReadings];
int currentIndex = 0; 

const int switchPin = 26; 
bool useEMG = true; 
String request = "";

const int CS_PIN =  5;

#define I2C_SDA 21 
#define I2C_SCL 22 

int lastAdcValue = 0;  
const int spikeThreshold = 30;  

unsigned long lastMoveTime = 0;  
bool servosMoved = false;  

void setup() {
  Serial.begin(115200);
  pinMode(switchPin, INPUT_PULLUP); 

  Wire.begin(I2C_SDA, I2C_SCL); 
  pwm.begin();
  pwm.setPWMFreq(50); 

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.begin();

  for (int i = 0; i < servoCount; i++) {
    pwm.setPWM(servoChannels[i], 0, servoMin);
  }

  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); 
  pinMode(25, OUTPUT); 
  digitalWrite(25, HIGH);
}

void loop() {
  useEMG = digitalRead(switchPin) == LOW; 

  if (useEMG) {
    handleEMG();
  } else {
    Serial.println("Switch off");
    handleWiFi();
  }

  delay(20); 
}

void handleWiFi() {
  WiFiClient client = server.available();

  if (client) {
    Serial.println("New client connected");

    while (client.connected()) {
      useEMG = digitalRead(switchPin) == LOW;
      if (useEMG) {
        Serial.println("Switch to EMG mode detected, breaking WiFi loop");
        break;
      }
      if (client.available()) {
        request = client.readStringUntil('\r');
        Serial.print("Received data: ");
        Serial.println(request);

        if (request.length() > servoCount) {
          request = request.substring(request.length() - servoCount);
        }

        Serial.print("Received data: ");
        Serial.println(request);

        if (request.length() == servoCount) {
          for (int i = 0; i < servoCount; i++) {
            char servoState = request.charAt(i);
            if (servoState == '1') {
              moveServo(i, 0);
            } else if (servoState == '0') {
              moveServo(i, 120);
            } else {
              Serial.println("Invalid servo state");
            }
          }
        } else {
          Serial.println("Invalid data length");
        }
      }
    }

    client.stop();
    Serial.println("Client disconnected");
  }
}

void handleEMG() {
  int adcValue = readADC();
  Serial.println(adcValue);

  unsigned long currentTime = millis();

  if (abs(adcValue - lastAdcValue) > spikeThreshold) {
    if (!servosMoved) {
      for (uint8_t i = 0; i < servoCount; i++) {
        moveServo(i, 120);
        delay(20);
      }
      servosMoved = true;
    }
    lastMoveTime = currentTime; 
  } else {
    if (servosMoved && currentTime - lastMoveTime >= 1000) {
      for (uint8_t i = 0; i < servoCount; i++) {
        moveServo(i, 0);
        delay(20);
      }
      servosMoved = false;
    }
  }

  lastAdcValue = adcValue;  
}

uint16_t readADC() {
  uint16_t result = 0;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW); 

  result = SPI.transfer16(0x0000);

  digitalWrite(CS_PIN, HIGH); 
  SPI.endTransaction();

  return result;
}

void moveServo(int index, int position) {
  if (index >= 0 && index < servoCount) {
    int pulseWidth = map(position, 0, 180, servoMin, servoMax);
    pwm.setPWM(servoChannels[index], 0, pulseWidth);
  } else {
    Serial.println("Invalid servo index");
  }
}
