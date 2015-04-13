#ifndef __LCD_H__
#define __LCD_H__
#include "Arduino.h"
#include <LiquidCrystal.h>

class LCD {
 public:
  LCD() : lcd_(40, 41, 42, 43, 44, 45) {
    lcd_.begin(16, 2);
  }
 private:
  static LiquidCrystal lcd_;
};
#endif  // __LCD_H__
