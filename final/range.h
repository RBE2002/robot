#ifndef __RANGE_H__
#define __RANGE_H__
#include "Arduino.h"
#include "loop.h"

class Range : public Loop {
 public:
  enum Type {
    kSharp,
    kMax,
    kVex,
  };

  Range(int port = 0, Type type = kSharp)
      : Loop(1e4 /*100Hz*/), type_(type), first_(true) {
    init(port);
    for (int i = 0; i < kHistLen; i++) {
      hist_[i] = 0.15;
    }
    dist_ = 0.15;
    avg_ = 0.15;
  }

  void init(int port) {
    port_ = port;
  }

  float Dist() { // meters
    return dist_;
  }

  float Avg() {
    return avg_;
  }

  void Run() {
    float avg = 0.0;
    for (int i = kHistLen - 1; i > 0; i--) {
      float newval = hist_[i - 1];
      hist_[i] = newval;
      avg += newval;
    }
    dist_ = SharpDist();
    hist_[0] = dist_;
    avg += dist_;
    avg_ = avg / (float)kHistLen;
  }

 private:

  struct Point {
    int x;
    double dist;
  };

  float SharpDist() {
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

  int port_;
  Type type_;
  bool first_;
  static const int kHistLen = 20;
  double
      hist_[kHistLen];  // Store a history of the last few distances. 0 = most recent.
  double dist_;
  double avg_;
};

#endif  // __RANGE_H__
