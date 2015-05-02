#ifndef __RANGE_H__
#define __RANGE_H__
#include "Arduino.h"
#include "loop.h"

/**
 * The Range class handles observing the various types of range sensor and
 * converting the raw values into useful numbers (ie, distances in meters).
 */
class Range : public Loop {
 public:
  enum Type {
    kSharp,
    kMax,
  };

  Range(int port = 0, Type type = kSharp)
      : Loop(1e4 /*100Hz*/), type_(type), first_(true) {
    init(port);
    // Initialize things, assuming default distances of 0.15 meters. This is to
    // avoid making the wall following freak out that we are extra close to a
    // wall or something.
    for (int i = 0; i < kHistLen; i++) {
      hist_[i] = 0.15;
    }
    dist_ = 0.15;
    avg_ = 0.15;
  }

  void init(int port, Type type = kSharp) {
    port_ = port;
    type_ = type;
  }

  float Dist() {  // meters
    return dist_;
  }

  // Returns the average distance over the last kHistLen iterations.
  float Avg() { return avg_; }

  void Run() {
    // Updates most recent values as the running average.
    // This is run every approximately 100 Hz.
    float avg = 0.0;
    for (int i = kHistLen - 1; i > 0; i--) {
      float newval = hist_[i - 1];
      hist_[i] = newval;
      avg += newval;
    }
    dist_ = (type_ == kSharp) ? SharpDist() : MaxDist();
    hist_[0] = dist_;
    avg += dist_;
    avg_ = avg / (float)kHistLen;
  }

 private:
  float SharpDist() {
    // Uses an exponential curve fit to convert between the raw Sharp IR sensor
    // values and the distance in meters.
    int raw = constrain(analogRead(port_), 70, 700);
    const float kCoeff = 0.822;
    const float kBase = 0.416;
    float retval = kCoeff * pow(kBase, (float)raw * 5.0 / 1023.0);
    if (first_) {
      for (int i = 0; i < kHistLen; i++) {
        hist_[i] = retval;
      }
      first_ = false;
    }
    return retval;
  }

  // For maxbotix sonar sensor.
  float MaxDist() {
    // Just need to use a linear fit to convert to meters.
    int raw = analogRead(port_);
    const float kAnalogToDist = 0.01266 * 1.5;
    return kAnalogToDist * (float)raw;
  }

  int port_;    // analog in port of sensor.
  Type type_;   // Type of sensor; determines how to calculate distance.
  bool first_;  // Whether or not this is the first run and we need to do
                // anything special to the history values.
  static const int kHistLen = 20;  // Length of history to store.
  double hist_[kHistLen];  // Store a history of the last few distances. 0 =
                           // most recent.
  double dist_;            // Last retrieved value.
  double avg_;             // Average of last kHistLen values.
};

#endif  // __RANGE_H__
