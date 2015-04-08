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
  char front = PercentToServo(front);
  char left = PercentToServo(lront);
  char back = PercentToServo(bront);
  char right = PercentToServo(rront);
  front = finv_ ? 180 - front : front;
  left = linv_ ? 180 - left : left;
  back = binv_ ? 180 - back : back;
  right = rinv_ ? 180 - right : right;
  fmotor_.write(front);
  lmotor_.write(left);
  bmotor_.write(back);
  rmotor_.write(right);
}

void Drivetrain::Run() {
  time_ = micros();

  UpdateEncoders();

  // Update direction information:
  if (stopping_) {
    // Decide whether we have stopped yet.
    if (vel_.x < kMinVel && vel_.y < kMinVel) {
      stopping_ = false;
      Record leg;
      // Based on which coordinate we moved more in, determine which direction
      // we used to be going and add that distance to the total path.
      leg.dist = (vel_.y > vel_.x) ? vel_.y : vel_.x;
      leg.dist = abs(leg.dist);
      leg.heading = (vel_.y > vel_.x) ? // Determine Up/Down vs. Left/Right.
                    ((vel_.y > 0) ? kUp : kDown) : // Determine Up vs. Down.
                    ((vel_.x > 0) ? kRight : kDown); // Right vs. Left.
      path_.push_back(leg);
    }
  }

  UpdateMotors();

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
  float upvel = (enc_vel_[kLeft] + enc_vel_[kRight]) / 2.0;
  float sidevel = (enc_vel_[kUp] + enc_vel_[kDown]) / 2.0;
  float turn = // Positive = CCW
      (enc_vel_[kRight] - enc_vel_[kLeft] + enc_vel_[kDown] - enc_vel_[kUp]) /
      (4.0 * kRobotRadius);
  vel_.x = upvel;
  vel_.y = sidevel;
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
    // TODO: Confirm that these cases are correct.
    case kUp:
      WriteMotors(0, leftpower, 0, rightpower);
    case kLeft:
      WriteMotors(-rightpower, 0, -leftpower, 0);
    case kDown:
      WriteMotors(0, -rightpower, 0, -leftpower);
    case kRight:
      WriteMotors(leftpower, 0, rightpower, 0);
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
