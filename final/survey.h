#ifndef __SURVEY_H__
#define __SURVEY_H__
#include "Arduino.h"
#include "loop.h"

// Uses the flame sensor to take readings and determine the location of the
// flame.
// This will basically just move the turret back and forth all the time,
// continually updating its estimate of where the flame is relative to the robot
// (based on the assumption that the robot is not currently moving). Provides an
// accessor to let you know if it saw something recently and would like you to
// stop the robot.
class Survey : public Loop {
 public:
  Survey(char flame_port, char turret_motor, char turret_pot, float p = 0.0,
         float i = 0.0, float d = 0.0);

  void Update() {
    Loop::Update();
    turret_->Update();
  }
 private:
  const kCutoff = 800; // Cutoff reading for seeing the flame.
  ArmPID turret_;
};
#endif  // __SURVEY_H__
