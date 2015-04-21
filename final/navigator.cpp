#include "navigator.h"
#include "constants.h"

Navigator::Navigator()
    : Loop(1e4 /*100Hz*/),
      drive_(motor_ports, motor_inversions, encoder_ports, range_ports),
      turret_(turret_motor, turret_pot, 0.9, 0.01, 0.0),  // TODO: Tune
      red_(red_port),
      black_(black_port),
      walling_(true),
      saw_flame_(false) {
  fantilt_.attach(tilt_port);
  Tilt(-20);
  drive_.Stop();
  Serial.println("Calibrating Gyro...");
  print("Calibrating...");
  drive_.imu_.CalibrateGyro();
  Serial.println("Done Calibrating Gyro.");
  print("Done Calibrating");
  pinMode(fan_port, OUTPUT);
}

void Navigator::Start() {
  drive_.set_navigating(true);
  drive_.set_wall_follow(true);
  drive_.DriveDirection(Drivetrain::kUp, 0.8);
}

void Navigator::Run() {
  UpdateTurret();
  // TODO: Refine this so it all is actually accurate, reports positions, etc.
  if (flame_out_) {
    if (drive_.get_path().size() >= num_legs_ * 2) {
      drive_.Stop();
    }
  } else if (red_.flame() && !saw_flame_) {
    saw_flame_ = true;
    drive_.DriveDist(0.5, drive_.leftdir(), 0.8, true);
    drive_.set_navigating(false);
    drive_.set_wall_follow(false);
  } else if (saw_flame_ && drive_.drive_dist_done()) {
    Fan(true);
    // Check until flame goes out.
    if (!red_.flame()) {
      Fan(false);
      drive_.set_wall_side(true /*Follow on left*/);
      flame_out_ = true;
      num_legs_ = drive_.get_path().size();
    }
  }
}

// Basically, we just want to keep the turret pointed at a right angle to our
// direction of travel while wall following.
void Navigator::UpdateTurret() {
  if (walling_) {
    int goal = (int)drive_.rightdir() * 90;
    turret_.set_deg(goal);
  }
}
