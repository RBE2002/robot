#include "drivetrain.h"
//#define DEBUG

Drivetrain::Drivetrain(int motors[kNumMotors], bool inverted[kNumMotors],
                       int encoder[kNumMotors * 2], int range[kNumMotors])
    : Loop(1e4 /* microseconds=>100Hz */),
      finv_(inverted[0]),
      linv_(inverted[1]),
      binv_(inverted[2]),
      rinv_(inverted[3]),
      fenc_(encoder[0], encoder[1]),
      lenc_(encoder[2], encoder[3]),
      benc_(encoder[4], encoder[5]),
      renc_(encoder[6], encoder[7]),
      cliff_(cliff_ports),
      wall_follow_(false),
      navigating_(false),
      uturn_(false),
      wall_on_left_(false),
      stopping_(false),
      found_(false),
      stop_end_(0),
      dir_(kUp),
      drive_dist_(-1),
      drive_dist_done_(true),
      by_line_(false),
      z_pos_(10),
      lcd_(40, 41, 42, 43, 44, 45) {

  // Attach the motors.
  fmotor_.attach(motors[0], 1000, 2000);
  lmotor_.attach(motors[1], 1000, 2000);
  bmotor_.attach(motors[2], 1000, 2000);
  rmotor_.attach(motors[3], 1000, 2000);

  // Set more stuff to zero.
  pos_.x = 0;
  pos_.y = 0;
  pos_.theta = 0;
  abs_pos_.x = 0;
  abs_pos_.y = 0;
  abs_pos_.theta = 0;

  // Initialize LCD.
  lcd_.begin(16, 2);

  // Initialize members of various arrays.
  for (int i = 0; i < kNumMotors; i++) {
    range_[i].init(range[i]);
    enc_[i] = 0;
    prev_enc_[i] = 0;
    enc_vel_[i] = 0;
  }
  range_[2].init(range[2], Range::kMax); // Using a maxbotix sensor on the back.
}

void Drivetrain::WriteMotors(int front, int left, int back, int right) {
  // Convert from -100 to 100 values into raw servo values (0 - 180).
  int fraw = PercentToServo(front);
  int lraw = PercentToServo(left);
  int braw = PercentToServo(back);
  int rraw = PercentToServo(right);
  // Flip outputs for inverted motors.
  fraw = finv_ ? 184 - fraw : fraw;
  lraw = linv_ ? 184 - lraw : lraw;
  braw = binv_ ? 184 - braw : braw;
  rraw = rinv_ ? 184 - rraw : rraw;
  // Clip outputs.
  int kMaxRaw = 175;
  int kMinRaw = 5;
  fraw = (fraw > kMaxRaw) ? kMaxRaw : (fraw < kMinRaw) ? kMinRaw : fraw;
  lraw = (lraw > kMaxRaw) ? kMaxRaw : (lraw < kMinRaw) ? kMinRaw : lraw;
  braw = (braw > kMaxRaw) ? kMaxRaw : (braw < kMinRaw) ? kMinRaw : braw;
  rraw = (rraw > kMaxRaw) ? kMaxRaw : (rraw < kMinRaw) ? kMinRaw : rraw;
#ifdef DEBUG
  Serial.print("Motor out vals: ");
  Serial.print(fraw);
  Serial.print("\t");
  Serial.print(lraw);
  Serial.print("\t");
  Serial.print(braw);
  Serial.print("\t");
  Serial.println(rraw);
#endif
  // Perform actual write.
  fmotor_.write(fraw);
  lmotor_.write(lraw);
  bmotor_.write(braw);
  rmotor_.write(rraw);
}

void Drivetrain::Run() {
#ifdef DEBUG
  Serial.print(AvgRangeError(kUp));
  Serial.print("\t");
  Serial.print(AvgRangeError(kLeft));
  Serial.print("\t");
  Serial.print(AvgRangeError(kDown));
  Serial.print("\t");
  Serial.println(AvgRangeError(kRight));
#endif
  // Record time so that we have a semi-precise dt for any P(ID) loops.
  time_ = micros();

  UpdateEncoders();

#ifdef DEBUG
  Serial.print("stopping_: ");
  Serial.print(stopping_);
  Serial.print(" Dir: ");
  Serial.print(dir_);
  Serial.print("\t");
#endif

  // Update direction information:
  if (stopping_) {
    // Decide whether we have stopped yet.
    if (vel_.x < kMinVel && vel_.y < kMinVel && millis() > stop_end_) {
      Serial.println("Done Stopping!");
      stopping_ = false;
      Record leg;
      // Based on which coordinate we moved more in, determine which direction
      // we used to be going and add that distance to the total path.
      leg.dist = (abs(pos_.y) > abs(pos_.x)) ? pos_.y : pos_.x;
      leg.dist = abs(leg.dist);
      leg.heading = (pos_.y > pos_.x) ? // Determine Up/Down vs. Left/Right.
                    ((vel_.y > 0) ? kUp : kDown) : // Determine Up vs. Down.
                    ((vel_.x > 0) ? kRight : kDown); // Right vs. Left.
      // Reject any ultra-short paths as probably being the result of silliness
      if (leg.dist > 0.1) path_.push_back(leg);
      pos_.x = 0;
      pos_.y = 0;
      pos_.theta = 0;
    }
  }

  // Determine whether we have hit a wall/edge and need to change our motion.
  if (navigating_) {
    double cur_dist = ((int)dir_ % 2) ? pos_.x : pos_.y;
    // First, check for if we are about to hit a wall.
    if (AvgRangeError(dir_) >
        -0.06 /*Tune so that inertia doesn't make us hit the wall*/ ||
        cliff_.on_line((CliffDetector::RobotSide)dir_)) {
      Serial.print("\nOh no! Running into wall:\t");
      Serial.println(AvgRangeError(dir_));
      DriveDirection(tabledir(), power_);
      if (cliff_.on_line((CliffDetector::RobotSide)dir_)) by_line_ = true;
      else by_line_ = false;
    }
    // Check if we were following a cliff and have now reached a wall again.
    else if ((cliff_.last_on_line((CliffDetector::RobotSide)walldir()) > 300 ||
              (cliff_.last_on_line((CliffDetector::RobotSide)walldir()) > 25 &&
               AvgRangeError(walldir()) > -0.1)) &&
             by_line_) {
      by_line_ = false;
    }
    // Check if we have reached the end of a wall and should uturn.
    else if (AvgRangeError(walldir()) < -0.1 /*Tune*/ && !uturn_ && !by_line_) {
      Serial.print("\nPAST wall. Distance to wall:\t");
      Serial.println(AvgRangeError(walldir()));
      uturn_ = true;
      uturn_state_ = kForward;
      set_wall_follow(false);
      double dist = 0.2;
      if (cliff_.last_on_line((CliffDetector::RobotSide)walldir()) < 10)
        dist = 0.6;
      DriveDist(dist + cur_dist /*tune*/, dir_, power_, false);
    }
    // Keep going if we still see the wall.
    else if (AvgRangeError(walldir()) > -0.00 /*Tune*/ && uturn_) {
      double dist = 0.4;
      if (walldir() == kLeft) dist = 0.3;
      DriveDist(dist + cur_dist, dir_, power_, false);
    }
    // Check if we are in a uturn and need to change direction.
    else if (uturn_ && drive_dist_ < 0) {
      switch (uturn_state_) {
        case kBack:
          uturn_ = false;
          set_wall_follow(true);
          DriveDirection(dir_, power_);
          break;
        case kSide:
          uturn_state_ = kBack;
          DriveDist(0.5/*tune*/, walldir(), power_, false);
          break;
        case kForward:
          uturn_state_ = kSide;
          DriveDist(0.5/*tune*/, walldir(), power_, false);
          break;
      }
    }
  }

  // Handle drive_dist stuff.
  if (drive_dist_ > 0) {
    Serial.print(pos_.x);
    Serial.print("\t");
    Serial.println(pos_.y);
    // Check if we are going up/down or right/left.
    if ((int)dir_ % 2) {  // side-to-side
      if (pos_.x > drive_dist_) drive_dist_done_ = true;
      else drive_dist_done_ = false;
    }
    else {
      if (pos_.y > drive_dist_) drive_dist_done_ = true;
      else drive_dist_done_ = false;
    }
    if (drive_dist_done_) Stop(!stop_drive_dist_);
  }

  UpdateMotors();

  prev_time_ = time_;
}

// Performs a third-order fit to linearize from percent to 0 - 180 scale.
// Based on data collected by running motors at certain servo out values and
// recording the speed of the motors.
int Drivetrain::PercentToServo(int percent) {
  percent = (percent > 100) ? 100 : (percent < -100) ? -100 : percent;
  const int kMax = 140;
  const int kMin = 40;
  const int kStartDead = 78;
  const int kEndDead = 106;
  // Inputs 0 - ~600; outputs 0 - 80, inclusive.
  const double kc0 = (percent > 0) ? 82.06 : 106;
  const double kc1 = -0.1;
  const double kc2 = 2.68e-4;
  const double kc3 = -4.9e-7;
  double kc2_signed = (percent > 0) ? kc2 : -kc2;
  double x1 = percent * 6;
  double x2 = x1 * x1;
  double x3 = x2 * x1;
  double raw = kc0 + kc1 * x1 + kc2_signed * x2 + kc3 * x3;

  if (percent == 0) raw = (kStartDead + kEndDead) / 2;
  return raw;
}

void Drivetrain::UpdateEncoders() {
  // Read in most recent encoder values.
  // Note that we are not using quadrature encoders, so the values we read will
  // always be positive and strictly increasing.
  enc_[0] = fenc_.read() * kTicksToMeters;
  enc_[1] = lenc_.read() * kTicksToMeters;
  enc_[2] = benc_.read() * kTicksToMeters;
  enc_[3] = renc_.read() * kTicksToMeters;

  // dt, in seconds.
  float dt = (time_ - prev_time_) * 1e-6;

  // Update velocities.
  for (int i = 0; i < kNumMotors; i++) {
    enc_vel_[i] = (enc_[i] - prev_enc_[i]) / dt;
    prev_enc_[i] = enc_[i];
  }

  // We use the minimum of each pair of encoders facing in the same direction
  // because there is so much slippage in the wheels that whichever encoder is
  // outputing the lowest values is most likely write. This seems to work
  // reasonably well in practice.
  float upvel = min(enc_vel_[kLeft], enc_vel_[kRight]);
  float sidevel = min(enc_vel_[kUp], enc_vel_[kDown]);
  // Calculcate angular velocity; unused in practice.
  float turn = // Positive = CCW
      (enc_vel_[kRight] - enc_vel_[kLeft] + enc_vel_[kDown] - enc_vel_[kUp]) /
      (4.0 * kRobotRadius);
  vel_.x = sidevel;
  vel_.y = upvel;
  vel_.theta = turn;
  // pos_ is just the change in position since the last change in direction. It
  // is always positive.
  pos_.x += vel_.x * dt;
  pos_.y += vel_.y * dt;
  pos_.theta += vel_.theta;
  // abs_pos actulaly cares about the sign, so determine if the direction is up
  // or right (using the modulo 3 as a trick to get there).
  abs_pos_.x += vel_.x * dt * (((int)dir_ % 3) ? -1 : 1);
  abs_pos_.y += vel_.y * dt * (((int)dir_ % 3) ? -1 : 1);
  if (!wall_on_left_) {
    fin_pos_ = abs_pos_;
  }

  // Print various information out to the LCD.
  char outstr[120], absstr[120], line2[120], tempx[6], tempy[6], tempz[6];
  const float kMeterstoIn = 39.37;
  const float kCandleDist = 0.1; // Distance from candle
  double finx =
      fin_pos_.x + (candle_dir_ == kRight ? kCandleDist : candle_dir_ == kLeft
                    ? -kCandleDist : 0);
  double finy =
      fin_pos_.y + (candle_dir_ == kUp ? kCandleDist : candle_dir_ == kDown
                    ? -kCandleDist : 0);
  // Arduino doesn't support the %f modifier...
  dtostrf(fin_pos_.x * kMeterstoIn, 4, 1, tempx);
  dtostrf(fin_pos_.y * kMeterstoIn, 4, 1, tempy);
  dtostrf(z_pos_, 4, 1, tempz);
  sprintf(outstr, "X: %s Y: %s", tempx, tempy);
  // Print out actual absolute position for debugging.
  dtostrf(abs_pos_.x * kMeterstoIn, 4, 1, tempx);
  dtostrf(abs_pos_.y * kMeterstoIn, 4, 1, tempy);
  sprintf(absstr, "X: %s Y: %s", tempx, tempy);
  Serial.println(outstr);
  sprintf(line2, "%s Z %s", wall_on_left_ ? "Out  " : (found_ ? "Found" : "On!  "),
          tempz);
  print(outstr, line2);

  // Unused
  abs_pos_.theta += vel_.theta * dt;

  // Set imu filter weights.
  // Currently, don't perform any filtering and just take the current estimate
  // of angle from the gyro as the Word Of God. The drift that the gyro exhibits
  // over the course of a few minutes is negligible.
  imu_.set_est_rate(vel_.theta);
  imu_.set_est_rate_weight(0);  // TODO: Tune.
  imu_.set_est_angle(vel_.theta);
  imu_.set_est_angle_weight(0);
#ifdef DEBUG
  Serial.print(pos_.x);
  Serial.print("\t");
  Serial.print(pos_.y);
  Serial.print("\t");
  Serial.print(fenc_.read());
  Serial.print("\t");
  Serial.print(lenc_.read());
  Serial.print("\t");
  Serial.print(benc_.read());
  Serial.print("\t");
  Serial.println(renc_.read());
#endif  // DEBUG
}

void Drivetrain::UpdateMotors() {
  // All this first part does is run the drivetrain at the appropriate power and
  // performs a simple P feedback loop to keep the drivetrain pointed straight.
  double rate = imu_.get_rate();
  double angle = imu_.get_angle();
  double rate_error =
      rate - 0;  // Replace 0 with something else if we want to turn.
  double angle_error = -angle;
  if (angle_error > PI) angle_error -= 2 * PI;
  if (angle_error < -PI) angle_error += 2 * PI;
  double diffangle =
      constrain(kPangle * angle_error, -30, 30);  // TODO: Expand to full PID, or just PD.
  double diffrate =
      kPrate * rate_error;  // TODO: Expand to full PID, or just PD.
  // TODO: Check that left/right are correct.
  double rightpower = power_ * 100 + diffrate - diffangle;
  double leftpower  = power_ * 100 - diffrate + diffangle;
#ifdef DEBUG
  Serial.print(angle_error);
  Serial.print("\t");
  Serial.println(power_);
#endif  // DEBUG

  // Handle wall following.
  // Always use range sensor to right of current direction.
  // RangeError() returns negative if we are too far away.
  float rangepower =  - kPrange * RangeError(walldir()) * 100;
  if (wall_on_left_) rangepower *= -1;
  rangepower = (wall_follow_ && !by_line_) ? rangepower : 0;

  // If we recently hit a cliff, get away from it ASAP.
  if (cliff_.last_on_line((CliffDetector::RobotSide)walldir()) < 23)
    rangepower = wall_on_left_ ? 90 : -90;

  // If stopping, we don't want to keep moving, but we still want to account for
  // the wall following to avoid running into walls or cliffs.
  if (stopping_) {
    rightpower = 0;
    leftpower = 0;
  }

  // Reverse if we just hit a line.
  if (cliff_.on_line((CliffDetector::RobotSide)dir())) {
    rightpower = -20;
    rightpower = -20;
  }

  switch (dir_) {
    case kUp:
      WriteMotors(rangepower, leftpower, rangepower, rightpower);
      break;
    case kLeft:
      WriteMotors(-rightpower, rangepower, -leftpower, rangepower);
      break;
    case kDown:
      WriteMotors(-rangepower, -rightpower, -rangepower, -leftpower);
      break;
    case kRight:
      WriteMotors(leftpower, -rangepower, rightpower, -rangepower);
      break;
    case kStop:
      WriteMotors(0, 0, 0, 0);
      break;
  }
}

void Drivetrain::Stop(bool resume) {
  drive_dist_ = -1;
  if (!resume) dir_ = kStop;
  stopping_ = true;
  stop_end_ = millis() + 2000;
}

void Drivetrain::DriveDirection(Direction heading, float power) {
#ifdef DEBUG
  Serial.print("Driving Power: ");
  Serial.print(power);
  Serial.print("\tHeading: ");
  Serial.println(heading);
#endif  // DEBUG
  drive_dist_ = -1;
  power_ = power;
  if (heading == dir_) return;
  Stop(true); // Stop the robot before heading in a different direction.
  dir_ = heading;
}

void Drivetrain::DriveDist(float distance, Direction heading, float power, bool stop) {
  drive_dist_done_ = false;
  stop_drive_dist_ = stop;
  Serial.println("Driving Dist.");
  DriveDirection(heading, power);
  drive_dist_ = distance;
}
