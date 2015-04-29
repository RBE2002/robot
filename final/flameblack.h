#ifndef __FLAMEBLACK_H__
#define __FLAMEBLACK_H__
#include "Arduino.h"
#include "loop.h"

/**
  *
  * This class contains code for the black flame sensor and its functions
  */
class FlameBlack : public Loop {
 public:
  FlameBlack(int port = 0)
      : Loop(1e4 /*100Hz*/), last_flame_(false), flame_(false) {
    init(port);
  }

/**
  * Initializes the flame sensor and sets the port
  */
  void init(int port) { port_ = port; }

/**
  * Return the booleans representing the flame state now and previously
  */
  bool flame() { return flame_ && last_flame_; }

  // REturns 0.0 - 1.0; 0 = no flame, 1 = sensor on fire (not really).
  double strength() { return strength_; }
  int raw() { return raw_; }

/**
  * Updates all the variables relating to the flame sensor
  * To be run in loop
  */
  void Run() {
    raw_ = analogRead(port_); //raw sensor values to be calculated
    last_flame_ = flame_; //stores the last raw value of the flame sensor
    flame_ = raw < kCutoff;
    strength_ = (double)(raw - kMin) / (double)(kMax - kMin); //calculates the intensity of the flame
  }

 private:
  const int kCutoff = 230; // Cutoff--below=sees flame, above = doesn't.
  // Max = highest flaminess; Min = least flamy.
  const int kMax = 100, kMin = 900;
  bool flame_; // Boolean state of the flame. True = on false = off
  bool last_flame_; // Whether we saw the flame on the last iteratoin as well.
  double strength_; //value which represents flame intensity and nearness to the flame
  int raw_;
  int port_; //the port the flame sensor is plugged into
};

#endif  // __FLAMEBLACK_H__
