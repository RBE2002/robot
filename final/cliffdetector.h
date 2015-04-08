/**
 * The CliffDetector just samples the line sensors and returns whether any are
 * over black.
 */

#ifndef __CLIFFDETECTOR_H__
#define __CLIFFDETECTOR_H__
#include "Arduino.h"
#include "loop.h"

class CliffDetector : public Loop {
 public:
  // Bits for bitmask used for indicating which sensors are activated.
  enum RobotSide {
    kFront=0x01,
    kLeft=0x02,
    kBack=0x04,
    kRight=0x08
  };
  // Takes an array of ports for the sensor being used, a cutoff (in units as
  // read from the ADC) for what is black/white for the sensor.
  // The ports are, in order, Front, Left, Back, and Right.
  CliffDetector(uint8_t ports[kNumSensors], int cutoff = 500)
      : ports_(ports),
        kCutoff(cutoff),
        Loop(0UL /* Poll sensor as fast as possible */) {}

  // Returns which lines we are over.
  uint8_t on_line() { return linemask_; }

  // Run function; called very iteration of main arduino loop.
  void Run() {
    linemask_ = 0;
    for (int i = 0; i < kNumSensors; i++) {
      // Read sensor and determine whether or not we are over a line.
      int sensor = analogRead(ports_[i]);
      if (sensor > kCutoff /*over line*/) linemask_ |= 0x01 << i;
    }
    return;
  }

 private:
  const int kNumSensors = 4;
  char linemask_;

  // Cutoff sensor value for determining whether or not a line is black/white.
  const int kCutoff;

  // Analog In ports of sensors.
  uint8_t ports_;
};
#endif  // __CLIFFDETECTOR_H__
