#include "Arduino.h"
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
#include <math.h>
#include "loop.h"
#include "imu.h"

IMU *imu;
double pos = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Beginning");
  imu = new IMU();
  imu->CalibrateGyro();
  Serial.println("Constructed");
}

void loop() {
  imu->Update();
  delay(10);
  double vel = imu->get_gyro_vel();
  pos += vel * 0.01;

  Serial.print(pos);
  Serial.print("\t");
  Serial.print(vel);
  Serial.print("\t");
  Serial.println(imu->get_compass_heading());
}
