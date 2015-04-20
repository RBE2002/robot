#ifndef __FLAMEBLACK_H__
#define __FLAMEBLACK_H__
#include "Arduino.h"
#include "loop.h"

class FlameBlack : public Loop {
 public:
  FlameBlack(int port = 0) : Loop(0 /*arbitrarily fast*/), flame_(false) {
    init(port);
  }

  void init(int port) { port_ = port; }

  bool flame() { return flame_; }

  // REturns 0.0 - 1.0; 0 = no flame, 1 = sensor on fire (not really).
  double strength() { return strength_; }

  void Run() {
    int raw = analogRead(port_);
    flame_ = raw < kCutoff;
    strength_ = (double)(raw - kMin) / (double)(kMax - kMin);
  }

 private:
  const int kCutoff = 400; // Cutoff--below=sees flame, above = doesn't.
  // Max = highest flaminess; Min = least flamy.
  const int kMax = 200, kMin = 900;
  bool flame_;
  double strength_;
  int port_;
};

#endif  // __FLAMEBLACK_H__
