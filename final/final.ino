#include <Encoder.h>
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>

#include <math.h>
#include "Arduino.h"

#include "Loop.h"
#include "IMU.h"
#include "Drivetrain.h"
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
