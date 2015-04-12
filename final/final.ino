#include <Encoder.h>
#include <Servo.h>
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>

#include <math.h>
#include "Arduino.h"

#include "loop.h"
#include "imu.h"
#include "drivetrain.h"
#include "constants.h"

Drivetrain *drive;

void setup() {
  //====================
  // Construct various objects.
  //====================
  drive = new Drivetrain(motor_ports, motor_inversions, encoder_ports);
}

void loop() {

  //====================
  // Update all periodic objects.
  //====================
  drive->Update();
}
