#ifndef __NAVIGATOR_H__
#define __NAVIGATOR_H__

#include "Arduino.h"
#include <Servo.h>

#include "drivetrain.h"
#include "turretpid.h"
#include "loop.h"
#include "constants.h"

class Navigator : public Loop {
 public:
  Navigator();

  void Start();

  void Update() {
    Loop::Update();
    drive_.Update();
    // TODO: uncomment turret.
    //turret_.Update();
    red_.Update();
    black_.Update();
  }
  void Run();
  template <typename T>
  void print(T stuff) {
    drive_.print(stuff);
  }
 private:
  const float kTiltSlope = -1.6;
  const int kTiltOffset = 100;

  void UpdateTurret();
  // Tilts to certain degrees (0 = fan is straight up; + = fan towards sky).
  void Tilt(int deg) {
    int out = deg * kTiltSlope + kTiltOffset;
    fantilt_.write(out);
  }

  // Subsystems
  Drivetrain drive_;
  Servo fantilt_;
  TurretPID turret_;
  FlameRed red_;
  FlameBlack black_;

  // State variables
  bool walling_; // Whether we are currently navigating around the walls.
  bool saw_flame_;
};

#endif  // __NAVIGATOR_H__
