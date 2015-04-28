/**
 * The CliffDetector just samples the line sensors and returns whether any are
 * over black.
 */

#ifndef __CLIFFDETECTOR_H__
#define __CLIFFDETECTOR_H__
#include "Arduino.h"
#include "loop.h"
#include "constants.h"

class CliffDetector : public Loop {
 public:
  // Bits for bitmask used for indicating which sensors are activated.
  enum RobotSide {
    kFront=0,
    kLeft=1,
    kBack=2,
    kRight=3
  };
  // Takes an array of ports for the sensor being used, a cutoff (in units as
  // read from the ADC) for what is black/white for the sensor.
  // The ports are, in order, Front, Left, Back, and Right.
  CliffDetector(int ports[kNumMotors], int cutoff = 480)
      : kCutoff(cutoff), Loop(1e4 /* 100Hz */) {
        for (int i = 0; i < kNumMotors; i++) {
          ports_[i] = ports[i];
          last_on_[i] = 1000;
        }
  }

  // Returns which lines we are over.
  int on_line() { return linemask_; }
  // Returns if we are over the corresponding line.
  int on_line(RobotSide side) { return linemask_ & (0x01 << side); }
  int last_on_line(RobotSide side) { return last_on_[(int)side]; }

  // Run function; called very iteration of main arduino loop.
  void Run() {
    linemask_ = 0;
    for (int i = 0; i < kNumMotors; i++) {
      // Read sensor and determine whether or not we are over a line.
      int sensor = analogRead(ports_[i]);
      if (sensor > kCutoff /*over line*/) {
        linemask_ |= 0x01 << i;
        last_on_[i] = 0;
      }
      else {
        last_on_[i] += 1;
      }
    }
  }

 private:
  char linemask_;

  // Cutoff sensor value for determining whether or not a line is black/white.
  const int kCutoff;

  // Analog In ports of sensors.
  int ports_[kNumMotors];
  unsigned long last_on_[kNumMotors];// How many iterations ago we saw a line.
};
#endif  // __CLIFFDETECTOR_H__
