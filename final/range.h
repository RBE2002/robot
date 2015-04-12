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

  Range(char port, Type type = kSharp)
      : Loop(1e4 /*100Hz*/), port_(port), type_(type) {}

  float Dist() { // meters
    switch (type_) {
      case kSharp:
        return SharpDist();
      default:
        return -1;
    }
  }

  void Run() { return; }

 private:

  struct Point {
    int x;
    double dist;
  };

  float SharpDist() {
    int raw = analogRead(port_);
    return Interpolate(raw, kSharpData, 13);
  }

  // Interpolate assumes that data is in decreasing order by x (x corresponds to
  // the x of Point). the length (len) of data must be >= 2.
  float Interpolate(int x, Point *data, int len) {
    if (x > data[0].x) return InterpolateTwo(x, data[0], data[1]);
    for (int i = 0; i < len; i++) {
      if (data[i].x < x) return InterpolateTwo(x, data[i - 1], data[i]);
    }
  }

  // Given two points, creates a line and determines what dist corresponds to
  // the given x.
  float InterpolateTwo(int x, Point one, Point two) {
    double m = (two.dist - one.dist) / (two.x - one.x);
    double b = one.dist - one.x * m;
    return m * x + b;
  }
  char port_;
  Type type_;
  Point kSharpData[13] = {{645, 0.06},
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
