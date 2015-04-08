#ifndef __IMU_H__
#define __IMU_H__
#include "Arduino.h"
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>

#include <math.h>

#include "Loop.h"

// All values in radians, seconds, or radians/sec.
// This handles averaging the compass and gyro information.
// It will consider rejecting a compass reading if the compass and gyro deviate
// by too much or if the compass deviates too far from an estimate sent in by
// the user (eg, from the encoders or the such).
class IMU : Loop {
 public:
  enum CompassResolution {
    k2Gauss=0x00,
    k4Gauss=0x20,
    k8Gauss=0x40,
    k16Guass=0x60
  };

  IMU();

  // Returns true if the compass is returning too crazy of values to use.
  bool RejectCompass();

  double get_gyro_vel() { return (double)gyro_.g.z * kRawGyroToRad; }
  double get_compass_heading() { return compass_heading_; }
  double get_compass_rate() { return compass_rate_; }
  // Returns current best estimate of the rate.
  double get_rate() { return rate_; }
  double get_angle() { return angle_; }

  // Set an estimate of the current robot's rotational velocity.
  void set_est_rate(double rate/*rad/sec*/) { est_rate_ = rate; }
  void set_est_rate_weight(int weight) { est_rate_weight_ = weight; }

  // Set an estimate of the robot's current angle.
  void set_est_angle(double angle/*rad*/) { est_angle_ = angle; }
  void set_est_angle_weight(int weight) { est_angle_weight_ = weight; }
 private:
  // Should be called at start of every iteration; updates compass rate as
  // appropriate. Returns true iff the state has changed.
  bool UpdateCompass();

  // Combine compass, gyro, and, if provided, the est_rate_.
  void Filter();

  const double kRawGyroToRad = 245.0 * PI / 180.0 / 32768.0;

  const int kCompassAngleWeight = 1000; // Tune.
  const int kGyroRateWeight = 1000;
  // Weight to use for combining rate with new position estimate.
  const int kPreviousAngleWeight = 100;

  // Maximum value, in units as returned by the magnetometer, that we should
  // expect as the sum of all axes.
  const int kMaxRawCompass = 10000;

  // Objects for compass and gyro devices. The compass also has an accelerometer.
  LSM303 compass_; // See https://github.com/pololu/lsm303-arduino/
  L3G gyro_; // See https://github.com/pololu/l3g-arduino/

  double angle_;    // Current most accurate estimate of angle.
  double rate_;     // Current most accurate rate_ estimate that we have.
  double compass_rate_; // Current estimate of rate from compass.
  double compass_heading_; // Current direction from compass.
  double est_rate_; // Estimated rate from user.
  int est_rate_weight_;  // Weighting for the estimated rate. If zero, est_rate_
                         // is ignored by everything.
  double est_angle_;
  double est_angle_weight_;

  // Information for estimating derivatives.
  unsigned long last_time_ = 0; // The time, in us, of the last cycle run.
  unsigned long time_ = 0;
  double last_compass_heading_;
};
#endif  // __IMU_H__
