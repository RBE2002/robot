#include <Encoder.h>
#include <Servo.h>
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
#include <LiquidCrystal.h>

#include <math.h>
#include "Arduino.h"

#include "range.h"
#include "loop.h"
#include "imu.h"
#include "drivetrain.h"
#include "constants.h"
#include "navigator.h"

Navigator *nav;

void setup() {
  Serial.begin(115200);
  Serial.println("Program Started.");
  //====================
  // Construct various objects.
  //====================
  nav = new Navigator();

  // Wait for button press in order to begin.
  pinMode(start_button, INPUT_PULLUP);
  while (digitalRead(start_button)) nav->Update();
  Serial.println("Beginning!");
  nav->Start();
}

void loop() {

  //====================
  // Update all periodic objects.
  //====================
  nav->Update();
}
