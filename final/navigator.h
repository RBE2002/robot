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
    // Only run when we are navigating TO flame.
    if (!saw_flame_) turret_.Update();
    red_.Update();
    black_.Update();
  }
  void Run();
  template <typename T>
  void print(T stuff, char * line2="") {
    drive_.print(stuff, line2);
  }
 private:
  // Used for conversion fropm degrees to tilt servo values. See Tilt().
  const float kTiltSlope = -1.6;
  const int kTiltOffset = 100;

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
  bool flame_out_;
  bool at_flame_;
  bool flame_z_;
  int num_legs_; // Number of legs in path.
  unsigned long flame_done_; // How long to wait after fan turns off to check flame.
  unsigned long fan_done_; // How long to wait after flame goes to turn of fan.
  int final_dir_;
  int lowest_z_servo_ /*Tilt angle with lowest flame return*/;
  int cur_z_servo_;
  int highest_flame_; // Most powerful detected flame value.
  unsigned long next_inc_z_;
  unsigned long stop_for_flame_; // Time at which to stop for flame.
};

#endif  // __NAVIGATOR_H__
