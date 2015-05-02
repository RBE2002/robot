#ifndef __FLAMERED_H__
#define __FLAMERED_H__
#include "Arduino.h"
#include "loop.h"

/**
  *
  * This class contains code for the red flame sensor and its functions
  */
class FlameRed : public Loop {
 public:
  FlameRed(char port = 0) : Loop(0 /*arbitrarily fast*/), flame_(false) {
    init(port);
  }

  void init(char port) { port_ = port; }

  /**
    * Returns a boolean of the state of the flame
    * True = flame on False = flame off
    */
  bool flame() { return flame_; }

  /**
    * Returns the raw sensor values
    */
  int raw() { return raw_; }

  /**
    * Sets the variables and calculates them
    * To be run in Loop
    */
  void Run() {
    raw_ = analogRead(port_);
    flame_ = raw_ < kCutoff;
  }

 private:
  const int kCutoff = 400;  // Cutoff--below=sees flame, above = doesn't.
  bool flame_;  // whether or not a flam exists/ is seen
  char port_;  // the port of the red sensor
  int raw_;  // the raw sensor value to be later interpereted
};

#endif  // __FLAMERED_H__
