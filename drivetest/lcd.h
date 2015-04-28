#ifndef __LCD_H__
#define __LCD_H__
#include "Arduino.h"
#include <LiquidCrystal.h>

class LCD {
 public:
 //sets the lcd ports
  LCD() : lcd_(40, 41, 42, 43, 44, 45) {
    begins the lcd with 2 lines of 16 characers
    lcd_.begin(16, 2);
  }
 private:
  static LiquidCrystal lcd_;
};
#endif  // __LCD_H__
