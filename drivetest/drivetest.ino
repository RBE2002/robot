#include "Arduino.h"
#include <Encoder.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
#include <math.h>

#include "loop.h"
#include "imu.h"
#include "drivetrain.h"
#include "constants.h"

Drivetrain *drive;

unsigned long start;

void setup() {
  Serial.begin(115200);
  //====================
  // Construct various objects.
  //====================
  drive = new Drivetrain(motor_ports, motor_inversions, encoder_ports);
  drive->imu_.CalibrateGyro();
  drive->set_wall_follow(true);

  drive->DriveDirection(Drivetrain::kUp, 0.5);
  start = millis();
}

bool updown = false;
void loop() {
  /*
  if (millis() > start + 8000 && updown) {
    drive->DriveDirection(Drivetrain::kLeft, 0.5);
    start = millis();
    updown = false;
  }
  else if (millis() > start + 6000 && !updown && millis() < start + 8000) {
    drive->DriveDirection(Drivetrain::kDown, 0.5);
    updown = true;
  }
  else if (millis() > start + 4000 && updown && millis() < start + 6000) {
    drive->DriveDirection(Drivetrain::kRight, 0.5);
    updown = false;
  }
  else if (millis() > start + 2000 && !updown && millis() < start + 4000) {
    drive->DriveDirection(Drivetrain::kUp, 0.5);
    updown = true;
  }
  */

  if (millis() > start + 10000) {
    drive->Stop();
    while (millis() < start + 10500) drive->Update();
    Serial.println(drive->get_path().size());
    Serial.println(drive->get_path()[drive->get_path().size() - 1].heading);
    Serial.println(drive->get_path()[drive->get_path().size() - 1].dist);
    delay(100);
    while (true) {
      drive->print(drive->RangeError(Drivetrain::kUp));
      delay(100);
    }
    exit(0);
  }

  //====================
  // Update all periodic objects.
  //====================
  drive->Update();
}
