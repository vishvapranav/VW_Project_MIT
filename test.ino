#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(D7, D6);
SoftwareSerial sim800Serial(D5, D4);

const float ACC_THRESHOLD = 3.0;

double latitude = 0.0, longitude = 0.0;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  sim800Serial.begin(9600);

  if (!mpu.begin()) {
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  initializeSIM800L();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                            a.acceleration.y * a.acceleration.y +
                            a.acceleration.z * a.acceleration.z);

  if (accMagnitude > ACC_THRESHOLD) {
    getGPSLocation();
    sendSOSMessage();
    delay(60000);
  }

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    }
  }
}

void initializeSIM800L() {
  sim800Serial.println("AT");
  delay(1000);
  sim800Serial.println("AT+CMGF=1");
  delay(1000);
  sim800Serial.println("AT+CNMI=1,2,0,0,0");
  delay(1000);
}

void getGPSLocation() {
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }
}

void sendSOSMessage() {
  String message = "SOS! Crash detected. Location: https://maps.google.com/?q=" + 
                   String(latitude, 6) + "," + String(longitude, 6);
  sim800Serial.println("AT+CMGS=\"+916382654685\"");
  delay(1000);
  sim800Serial.print(message);
  sim800Serial.write(26);
  delay(5000);
}
