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
  // Used for conversion fropm degrees to tilt servo values. See Tilt().
  const float kTiltSlope = -1.6;
  const int kTiltOffset = 100;
  bool flame_out_;
  int num_legs_; // Number of legs in path.

  void UpdateTurret();
  // Tilts to certain degrees (0 = fan is straight up; + = fan towards sky).
  void Tilt(int deg) {
    int out = deg * kTiltSlope + kTiltOffset;
    fantilt_.write(out);
  }
  // Turns fan on/off.
  void Fan(bool on) {
    if (on) digitalWrite(fan_port, HIGH);
    else digitalWrite(fan_port, LOW);
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
