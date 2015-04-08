#ifndef __DRIVETRAIN_H__
#define __DRIVETRAIN_H__

#include "Arduino.h"
#include <Encoder.h>

#include "IMU.h"
#include "Loop.h"

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
  Drivetrain(char motors[kNumMotors], bool inverted[kNumMotors],
             char encoder[kNumMotors*2]);

  // Each is -100 to 100, where -100 is full CW and +100 is full CCW.
  void WriteMotors(char front, char left, char back, char right);
  // Stops the robot and then starts driving in the provided direction.
  void DriveDirection(Direction heading, float power /* 0.0 to 1.0 */);
  // pass true to resume in order to have the robot start travelling in the
  // current direction after it finishes stopping the robot. By default, assume
  // that we want to stop the robot completely.
  void Stop(bool resume=false); // Stops the robot; ends the current portion of the path.

  IMU imu_;
 private:
  const float kTicksToMeters = 2.0 * PI / 90.0 /* ticks to radians */
                               * 0.02 /* radius of wheels, in m */;
  const float kRobotRadius = 0.1; // Radius of robot.
  const char kNumMotors = 4;
  const float kPangle = 1.0, kPrate = 1.0;

  // Velocity threshold at which we consider things stopped.
  const float kMinVel = 0.01;

  // Converts a -100 to +100 value to run the motors at and converts it to the 0
  // to 180 value needed for the vex motor controllers.
  char PercentToServo(char percent);
  void UpdateEncoders();
  void UpdateMotors();

  Servo fmotor_, lmotor_, bmotor_, rmotor_;
  bool finv_, linv_, binv_, rinv_;
  Encoder fenc_, lenc_, benc_, renc_;

  // All in meters or meters/sec.
  float prev_enc_[kNumMotors];
  float enc_[kNumMotors];
  float enc_vel_[kNumMotors];
  // Unfiltered state. IMU will do the filtering.
  Vector vel_;
  // pos_ is rezeroed with every new direction order.
  Vector pos_;

  // Microseconds
  unsigned long prev_time_;
  unsigned long time_;

  Direction dir_;
  bool stopping_; // True if in process of stopping.
  double power_; // Power with which we are running the motors, 0.0 - 1.0;
  vector::Vector<Record> path_;
};
#endif  //  __DRIVETRAIN_H__
