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
      hist_[i] = 0;
    }
  }

  void init(int port) {
    port_ = port;
  }

  float Dist() { // meters
    return hist_[0];
  }

  float Avg() {
    float avg = 0.0;
    for (int i = 0; i < kHistLen; i++) {
      avg += hist_[i];
    }
    return avg / kHistLen;
  }

  void Run() {
    for (int i = 4; i > 0; i--) {
      hist_[i] = hist_[i - 1];
    }
    hist_[0] = SharpDist();
  }

 private:

  struct Point {
    int x;
    double dist;
  };

  float SharpDist() {
    int raw = constrain(analogRead(port_), 70, 700);
    return Interpolate(raw, 13);
  }

  // Interpolate assumes that data is in decreasing order by x (x corresponds to
  // the x of Point). the length (len) of data must be >= 2.
  float Interpolate(int x, int len) {
    if (first_) {
      int x = kSharpData[0].x * 1;
      first_ = false;
      delay(1);
    }
    if (x > kSharpData[0].x)
      return InterpolateTwo(x, kSharpData[0], kSharpData[1]);
    for (int i = 1; i < len; i++) {
      if (kSharpData[i].x < x)
        return InterpolateTwo(x, kSharpData[i - 1], kSharpData[i]);
    }
    return InterpolateTwo(x, kSharpData[len - 2], kSharpData[len - 1]);
  }

  // Given two points, creates a line and determines what dist corresponds to
  // the given x.
  float InterpolateTwo(int x, Point one, Point two) {
    double m = (two.dist - one.dist) / (two.x - one.x);
    double b = one.dist - one.x * m;
    return m * x + b;
  }
  int port_;
  Type type_;
  bool first_;
  static const int kHistLen = 5;
  double
      hist_[kHistLen];  // Store a history of the last few distances. 0 = most recent.
  const Point kSharpData[13] = {{645, 0.06},
                                {610, 0.07},
                                {563, 0.08},
                                {471, 0.1},
                                {338, 0.15},
                                {266, 0.2},
                                {221, 0.25},
                                {188, 0.3},
                                {153, 0.4},
                                {127, 0.5},
                                {104, 0.6},
                                {92, 0.7},
                                {84, 0.8}};
};

#endif  // __RANGE_H__
