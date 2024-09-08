#include <Wire.h>
#include <math.h>
#include <ESP8266WiFi.h>

#define SOUND_SPEED 0.034  // cm/us
#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define INFRARED_PIN A0
#define TILT_SENSOR_ADDR 0x6b
#define TILT_CTRL_REG6_XL 0x20
#define TILT_CTRL_REG8 0x22
#define TILT_OUT_X_XL 0x28
#define LED_GREEN 10
#define LED_YELLOW 7
#define LED_RED 8

short xTilt, yTilt, zTilt;
unsigned long startTimeUS, endTimeUS;
bool echoFlag = false, triggerFlag = false;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Initializing...");

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), measureEcho, CHANGE);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  initTiltSensor();

  Serial.println("Setup Complete");
}

void initTiltSensor() {
  Wire1.begin();

  Wire1.beginTransmission(TILT_SENSOR_ADDR);
  Wire1.write(TILT_CTRL_REG8);
  Wire1.write(0x05);  // Reset
  Wire1.endTransmission();
  delay(10);

  // Set Sensitivity
  Wire1.beginTransmission(TILT_SENSOR_ADDR);
  Wire1.write(TILT_CTRL_REG6_XL);
  Wire1.write(0x70);  // Sensitivity Configuration
  Wire1.endTransmission();
}

double measureInfraredDistance() {
  int reading = 0;
  for (int i = 0; i < 1000; i++) {
    reading += analogRead(INFRARED_PIN);
  }
  reading /= 1000;

  double distance = 235.81 * exp(-0.006 * reading);
  Serial.print("Infrared Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

void triggerUltrasonic() {
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(TRIGGER_PIN, LOW);
}

void measureEcho() {
  if (!echoFlag) {
    startTimeUS = micros();
    echoFlag = true;
  } else {
    endTimeUS = micros();
    triggerFlag = true;
    echoFlag = false;
  }
}

double getUltrasonicDistance() {
  if (triggerFlag) {
    double distance = (endTimeUS - startTimeUS) * SOUND_SPEED / 2;
    triggerFlag = false;

    Serial.print("Ultrasonic Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(100);

    triggerUltrasonic();
    return distance;
  }
  triggerUltrasonic();
  return 0;
}

float readTilt() {
  byte buffer[6];
  Wire1.beginTransmission(TILT_SENSOR_ADDR);
  Wire1.write(0x80 | TILT_OUT_X_XL);
  Wire1.endTransmission(false);
  Wire1.requestFrom(TILT_SENSOR_ADDR, 6, true);

  for (int i = 0; i < 6 && Wire1.available(); i++) {
    buffer[i] = Wire1.read();
  }

  zTilt = (((int)buffer[5]) << 8) | buffer[4];
  float tiltZ = zTilt * 4.0 / 32768.0;
  return tiltZ;
}

void processTilt(float& tiltAngleDeg) {
  const float pi = 3.14159;
  float tiltZ = readTilt();
  float tiltRad = acos(tiltZ / 0.963);
  tiltAngleDeg = (tiltRad * 180) / pi;

  Serial.print("Tilt: ");
  Serial.print(tiltAngleDeg);
  Serial.println(" degrees");
}

void adjustDistanceForTilt(double& distance, float tiltAngleDeg) {
  distance = cos(tiltAngleDeg * M_PI / 180.0) * distance;
  Serial.print("Adjusted Distance: ");
  Serial.println(distance);
}

void setLEDsBasedOnDistance(double distance) {
  if (distance < 5) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);
  } else if (distance < 10) {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_RED, LOW);
  } else {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, HIGH);
  }
}

void loop() {
  double infraredDist = measureInfraredDistance();
  double ultrasonicDist = getUltrasonicDistance();
  double distanceFinal = 0;
  float tiltAngleDeg = 0;

  if (infraredDist < 2) {
    distanceFinal = infraredDist;
  } else if (abs(ultrasonicDist - infraredDist) < 3 && ultrasonicDist >= 2 && ultrasonicDist < 7) {
    distanceFinal = 0.6 * ultrasonicDist + 0.4 * infraredDist;
  } else if (abs(ultrasonicDist - infraredDist) < 3 && ultrasonicDist >= 7 && ultrasonicDist < 10) {
    distanceFinal = 0.8 * ultrasonicDist + 0.2 * infraredDist;
  } else {
    distanceFinal = ultrasonicDist;
  }

  processTilt(tiltAngleDeg);
  adjustDistanceForTilt(distanceFinal, tiltAngleDeg);
  setLEDsBasedOnDistance(distanceFinal);
}
