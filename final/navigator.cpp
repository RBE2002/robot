#include "navigator.h"
#include "constants.h"

Navigator::Navigator()
    : Loop(1e4 /*100Hz*/),
      drive_(motor_ports, motor_inversions, encoder_ports, range_ports),
      turret_(turret_motor, turret_pot),
      red_(red_port),
      black_(black_port),
      walling_(true) {
  fantilt_.attach(tilt_port);
  Tilt(-20);
  drive_.Stop();
  Serial.println("Calibrating Gyro...");
  drive_.imu_.CalibrateGyro();
  Serial.println("Done Calibrating Gyro.");
}

void Navigator::Start() {
  drive_.set_navigating(true);
  drive_.set_wall_follow(true);
  drive_.DriveDirection(Drivetrain::kUp, 0.8);
}

void Navigator::Run() {
  UpdateTurret();
  if (black_.flame()) drive_.Stop();
}

// Basically, we just want to keep the turret pointed at a right angle to our
// direction of travel while wall following.
void Navigator::UpdateTurret() {
  if (walling_) {
    int goal = (int)drive_.rightdir() * 90;
    turret_.set_deg(goal);
  }
}
