#include "navigator.h"
#include "constants.h"
#include "vector.h"

Navigator::Navigator()
    : Loop(1e4 /*100Hz*/),
      drive_(motor_ports, motor_inversions, encoder_ports, range_ports),
      turret_(turret_motor, turret_pot, 0.9, 0.03, 0.0),  // TODO: Tune
      red_(red_port),
      black_(black_port),
      walling_(true),
      saw_flame_(false),
      flame_out_(false),
      at_flame_(false),
      flame_z_(false),
      lowest_z_servo_(0),
      cur_z_servo_(20),
      next_inc_z_(0),
      highest_flame_(10000), /*inf*/
      flame_done_(0),
      fan_done_(0) {
  fantilt_.attach(tilt_port);
  Tilt(20);
  drive_.Stop();
  Serial.println("Calibrating Gyro...");
  print("Calibrating...");
  drive_.imu_.CalibrateGyro();
  Serial.println("Done Calibrating Gyro.");
  print("Done Calibrating");
  pinMode(fan_port, OUTPUT);
  turret_.set_deg(0);
}

void Navigator::Start() {
  Tilt(-1);
  drive_.set_navigating(true);
  drive_.set_wall_follow(true);
  drive_.DriveDirection(Drivetrain::kUp, 0.8);
}

void Navigator::Run() {
  UpdateTurret();
  // TODO: Refine this so it all is actually accurate, reports positions, etc.
  if (flame_out_) {
    if (drive_.get_path().size() >= ((num_legs_ - 1) * 2)) {
      drive_.set_navigating(false);
      drive_.set_wall_follow(true);
      // For last leg, go correct distance.
      vector::Vector<Drivetrain::Record> path = drive_.get_path();
      drive_.DriveDist(path[1].dist, drive_.dir(), 0.8);
    }
  } else if (red_.flame() && !saw_flame_ && !drive_.stopping()) {
    saw_flame_ = true;
    walling_ = false;
    final_dir_ = (int)drive_.tabledir();
    drive_.set_navigating(false);
    drive_.set_wall_follow(false);
    drive_.Stop(false);
    at_flame_ = false;
  } else if (flame_z_ && !at_flame_) {
    if (red_.raw() < 200) {
      turret_.Stop();
      drive_.Stop();
      at_flame_ = true;
    }
  } else if (saw_flame_ && !flame_z_) {
    if (millis() > next_inc_z_) {
      cur_z_servo_ -= 1;
      next_inc_z_ = millis() + 100;
    }
    Tilt(cur_z_servo_);
    if (red_.raw() < highest_flame_) {
      highest_flame_ = red_.raw();
      lowest_z_servo_ = cur_z_servo_;
    }
    if (cur_z_servo_ <= -20) {
      flame_z_ = true;
      double height = ((double)lowest_z_servo_ / 5.0 + 10.0);
      drive_.set_z(height);
      Tilt(lowest_z_servo_ + 5/*Necessary offset b/c flame sensor is at top*/);
      drive_.DriveDirection(drive_.tabledir(), 0.8);
    }
  } else if (at_flame_ && flame_z_) {
    Fan(true);
    drive_.Stop(false);
    // Check until flame goes out and then wait a couple seconds.
    if (!red_.flame() && fan_done_ == 0) {
      fan_done_ = millis() + 5000;
    } else if (!red_.flame() && fan_done_ < millis() && flame_done_ == 0) {
      Fan(false);
      flame_done_ = millis() + 5000;
    } else if (red_.flame()) {
      flame_done_ = 0;
      fan_done_ = 0;
      Fan(true);
    }
    else if (!red_.flame() && flame_done_ < millis() && fan_done_ < millis()) {
      Fan(false);
      drive_.set_navigating(true);
      drive_.set_wall_follow(true);
      drive_.set_wall_side(true /*Follow on left*/);
      drive_.DriveDirection(
          (Drivetrain::Direction)((final_dir_ + 2) % 4), 0.8);
      flame_out_ = true;
      num_legs_ = drive_.get_path().size();
    }
  }
}

// Basically, we just want to keep the turret pointed at a right angle to our
// direction of travel while wall following.
void Navigator::UpdateTurret() {
  if (walling_ && !saw_flame_) {
    int goal = (int)drive_.tabledir() * 90;
    turret_.set_deg(goal);
  }
}
