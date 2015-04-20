#include <Encoder.h>
#include <Servo.h>
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>

#include <math.h>
#include "Arduino.h"

#include "loop.h"
#include "constants.h"
#include "turretpid.h"

TurretPID *turret;

unsigned long start;
bool ccw;

const int kMax = 500;
const int kMin = 300;

void setup() {
  Serial.begin(115200);
  // motor, pot, p, i, d
  turret = new TurretPID(turret_motor, turret_pot, 0.9, 0.01, 0.0);
  turret->set_deg(0);
  start = millis();
  ccw = false;
}

void loop() {
  if (start + 5000 < millis()) {
    start = millis();
    ccw = !ccw;
    turret->set_deg(ccw ? 180 : -90);
  }
  turret->Update();
}
