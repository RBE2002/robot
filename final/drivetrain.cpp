#include "drivetrain.h"

Drivetrain::Drivetrain(char motors[kNumMotors], bool inverted[kNumMotors],
                       char encoder[kNumMotors*2])
    : finv_(inverted[0]),
      linv_(inverted[1]),
      binv_(inverted[2]),
      rinv_(inverted[3]),
      fenc_(encoder[0], encoder[1]),
      lenc_(encoder[2], encoder[3]),
      benc_(encoder[4], encoder[5]),
      renc_(encoder[6], encoder[7]) {
  fmotor_.attach(motors[0], 1000, 2000);
  lmotor_.attach(motors[1], 1000, 2000);
  bmotor_.attach(motors[2], 1000, 2000);
  rmotor_.attach(motors[3], 1000, 2000);
}

void Drivetrain::WriteMotors(char front, char left, char back, char right) {
  fmotor_.write(PercentToServo(front));
  lmotor_.write(PercentToServo(lront));
  bmotor_.write(PercentToServo(bront));
  rmotor_.write(PercentToServo(rront));
}

void Drivetrain::Run() {
  time_ = micros();

  UpdateEncoders();

  prev_time_ = time_;
}

// Performs a third-order fit to linearize from percent to 0 - 180 scale.
char Drivetrain::PercentToServo(char percent) {
  const int kMax = 140;
  const int kMin = 40;
  const int kStartDead = 88;
  const int kEndDead = 98;
  // Inputs 0 - 600; outputs 98-139, inclusive.
  const double kc0 = 96.523;
  const double kc1 = 7.13e-2;
  const double kc2 = -3e-4;
  const double kc3 = 5e-7;
  double x1 = abs(percent * 6);
  double x2 = x1 * x1;
  double x3 = x2 * x1;
  double raw = kc0 + kc1 * x1 + kc2 * x2 + kc3 * x3;
  if (percent < 0) raw = (180 - raw);

  if (percent == 0) raw = (kStartDead + kEndDead) / 2;
  return raw;
}

void Drivetrain::UpdateEncoders() {
  enc_[0] = fenc_.read() * kTicksToMeters;
  enc_[1] = lenc_.read() * kTicksToMeters;
  enc_[2] = benc_.read() * kTicksToMeters;
  enc_[3] = renc_.read() * kTicksToMeters;

  float dt = (time_ - prev_time_) * 1e-6;

  for (int i = 0; i < kNumMotors; i++) {
    enc_vel_[i] = (enc_[i] - prev_enc_[i]) / dt;
    prev_enc_[i] = enc_[i];
  }

  // Warning: If using single encoders, change code to use gyro to determine
  // which direction the robot is turning in.
  float sidevel = (enc_vel_[kLeft] + enc_vel_[kRight]) / 2.0;
  float upvel = (enc_vel_[kUp] + enc_vel_[kDown]) / 2.0;
  float turn =
      (enc_vel_[kLeft] - enc_vel_[kRight] + enc_vel_[kUp] - enc_vel_[kDown]) /
      (4.0 * kRobotRadius);
  vel_.x = sidevel;
  vel_.y = upvel;
  vel_.theta = turn;
  pos_.x += vel_.x * dt;
  pos_.y += vel_.y * dt;
  pos_.theta += vel_.theta;
  imu_.set_est_rate(vel_.theta);
  imu_.set_est_rate_weight(1);  // TODO: Tune.
  imu_.set_est_angle(vel_.theta);
  imu_.set_est_angle_weight(1);
}

void Drivetrain::UpdateMotors() {
  double rate = imu_.get_rate();
  double angle = imu_.get_angle();
  double rate_error =
      rate - 0;  // Replace 0 with something else if we want to turn.
  double angle_error = angle - (int)dir_ * PI / 2.0;
  double diffangle =
      kPangle * angle_error;  // TODO: Expand to full PID, or just PD.
  double diffrate =
      kPrate * rate_error;  // TODO: Expand to full PID, or just PD.
  // TODO: Check that left/right are correct.
  double rightpower = power_ + diffrate + diffangle;
  double leftpower  = power_ - diffrate - diffangle;
  if (stopping_) {
    rightpower = 0;
    leftpower = 0;
  }
  switch (dir_) {
    // TODO: figoure out which pairs of motors correspond to which.
    case kFront:
      WriteMotors(0, 0, 0, 0);
    case kFront:
      WriteMotors(0, 0, 0, 0);
    case kFront:
      WriteMotors(0, 0, 0, 0);
    case kFront:
      WriteMotors(0, 0, 0, 0);
    case kStop:
      WriteMotors(0, 0, 0, 0);
  }
}

void Drivetrain::Stop(bool resume) {
  if (!resume) dir_ = kStop;
  stopping_ = true;
}

void Drivetrain::DriveDirection(Direction heading, float power) {
  Stop(true); // Stop the robot before heading in a different direction.
  power_ = power;
  dir_ = heading;
}
