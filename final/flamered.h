#ifndef __FLAMERED_H__
#define __FLAMERED_H__
#include "Arduino.h"
#include "loop.h"

class FlameRed : public Loop {
 public:
  FlameRed(char port = 0) : Loop(0 /*arbitrarily fast*/), flame_(false) {
    init(port);
  }

  void init(char port) { port_ = port; }

  bool flame() { return flame_; }

  void Run() { flame_ = analogRead(port_) < kCutoff; }

 private:
  const int kCutoff = 400; // Cutoff--below=sees flame, above = doesn't.
  bool flame_;
  char port_;
};

#endif  // __FLAMERED_H__
