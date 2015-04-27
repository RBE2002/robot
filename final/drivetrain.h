#ifndef __DRIVETRAIN_H__
#define __DRIVETRAIN_H__

#include "Arduino.h"
#include <Encoder.h>
#include <Servo.h>
#include <LiquidCrystal.h>

#include "imu.h"
#include "loop.h"
#include "vector.h"
#include "constants.h"
#include "range.h"
#include "flamered.h"
#include "flameblack.h"
#include "cliffdetector.h"

// The drivetrain constructs and initializes the drivetrain motors, the IMU, and
// the Encoders. You can access pointers for all of these through accessor
// functions and shouldn't really need to. The drivetrain will provide functions
// for getting the drivetrain's current position.
class Drivetrain : public Loop {
 public:
  enum Direction {
    kUp=0,
    kLeft=1,
    kDown=2,
    kRight=3,
    kStop,
  };

  struct Vector {
    Vector() : x(0), y(0), theta(0) {}
    double x; // Left/Right
    double y; // Up/Down
    double theta;
  };

  struct Record {
    Direction heading;
    double dist; // meters.
  };

  // Takes an array of front/left/back/right motors, their inversion states, and
  // an encoder associated with them. If only using one channel of the encoder,
  // enter the same number twice. The format of the array is [enc1a, enc1b,
  // enc2a, enc2b, enc3a, enc3b, enc4a, enc4b].
  // The first motor (the one this code refers to as "front") is the one at the
  // top if the robot, which points side-to-side. The next motors continue
  // counter-clockwise around the robot. Encoders correspond to their motors.
  // Motors, inversion, and encoders should be defined such that the positive
  // direction is forwards and rightwards, as appropriate.
  Drivetrain(int motors[kNumMotors], bool inverted[kNumMotors],
             int encoder[kNumMotors*2], int range[kNumMotors]);

  // Each is -100 to 100, where -100 is full CW and +100 is full CCW.
  void WriteMotors(int front, int left, int back, int right);
  // Stops the robot and then starts driving in the provided direction.
  void DriveDirection(Direction heading, float power /* 0.0 to 1.0 */);
  // Roughly drives the robot a certain distance---as of yet, no PID.
  void DriveDist(float distance, Direction heading, float power, bool stop=true);
  // pass true to resume in order to have the robot start travelling in the
  // current direction after it finishes stopping the robot. By default, assume
  // that we want to stop the robot completely.
  void Stop(bool resume=false); // Stops the robot; ends the current portion of the path.

  void Run(); // Overrides Run from Loop class.

  void set_wall_follow(bool wall_follow) { wall_follow_ = wall_follow; }
  void set_navigating(bool navigate) { navigating_ = navigate; }
  void set_wall_side(bool wall_on_left) { wall_on_left_ = wall_on_left; }

  float RangeError(Direction sensor_sel) {
    return kWallDist - range_[(int)sensor_sel].Dist();
  }

  float AvgRangeError(Direction sensor_sel) {
    float avg = range_[(int)sensor_sel].Avg();
    float err = kWallDist - avg;
    return err;
  }

  void Update() {
    Loop::Update();
    imu_.Update();
    cliff_.Update();
    for (int i = 0; i < kNumMotors; i++) range_[i].Update();
  }

  template <typename T>
  void print(T stuff, char * line2="") {
    lcd_.clear();
    lcd_.print(stuff);
    lcd_.setCursor(0, 1);
    lcd_.print(line2);
  }

  IMU imu_;

  vector::Vector<Record> get_path() { return path_; }
  Direction dir() { return dir_; }

  // direction to right of current one.
  Direction rightdir() { return (Direction)(((int)dir_ + 3) % 4); }
  // direction to left of current one.
  Direction leftdir() { return (Direction)(((int)dir_ + 1) % 4); }
  // direction in which we can find the wall we are following.
  Direction walldir() { return wall_on_left_ ? leftdir() : rightdir(); }
  // Direction opposite walldir
  Direction tabledir() { return wall_on_left_ ? rightdir() : leftdir(); }

  bool drive_dist_done() { return drive_dist_done_; }

 private:
  const float kTicksToMeters = 2.0 * PI / 360.0 /* ticks to radians */
                               * 0.035 /* radius of wheels, in m */;
  const float kRobotRadius = 0.1; // Radius of robot.
  const float kPangle = 200, kPrate = 0, kPrange = 10;
  const float kWallDist = 0.2;

  // Velocity threshold at which we consider things stopped.
  const float kMinVel = 0.01;

  // Converts a -100 to +100 value to run the motors at and converts it to the 0
  // to 180 value needed for the vex motor controllers.
  int PercentToServo(int percent);
  void UpdateEncoders();
  void UpdateMotors();

  Servo fmotor_, lmotor_, bmotor_, rmotor_;
  bool finv_, linv_, binv_, rinv_;
  Encoder fenc_, lenc_, benc_, renc_;
  // Range sensors; in order, they are the sensors pointing forwards, left,
  // back, and right.
  Range range_[kNumMotors];
  CliffDetector cliff_;

  // All in meters or meters/sec.
  float prev_enc_[kNumMotors];
  float enc_[kNumMotors];
  float enc_vel_[kNumMotors];
  // Unfiltered state. IMU will do the filtering.
  Vector vel_;
  // pos_ is rezeroed with every new direction order.
  Vector pos_;
  // abs_pos_ is not zeroed on every change.
  Vector abs_pos_;
  Vector fin_pos_;

  // Microseconds
  unsigned long prev_time_;
  unsigned long time_;

  Direction dir_;
  bool stopping_; // True if in process of stopping.
  double power_; // Power with which we are running the motors, 0.0 - 1.0;
  vector::Vector<Record> path_;
  bool wall_follow_; // Whether, at this instant, we are wall following.
  bool navigating_; // Whether we will continue wall following after this move.
  bool uturn_; // Whether we are currently in a u-turn.
  float drive_dist_; // If negative, no limit on distance to drive.
  bool stop_drive_dist_;
  bool drive_dist_done_;  // False if current running drive_dist and not done;
                          // true otherwise.
  bool wall_on_left_; // True if the wall is to our left. Default to false.
  unsigned long stop_end_;
  enum {
    kForward,
    kSide,
    kBack
  } uturn_state_; // Which leg of the uturn we are in.

  LiquidCrystal lcd_;
};
#endif  //  __DRIVETRAIN_H__
