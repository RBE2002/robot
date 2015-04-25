#include "Arduino.h"
#include "range.h"
#include <LiquidCrystal.h>

LiquidCrystal lcd(40, 41, 42, 43, 44, 45);

Range *sharp;

unsigned long cont;
void setup() {
  cont = 0;
  sharp = new Range[4];
  Serial.begin(115200);
  Serial.println("Restartin");
  lcd.begin(16, 2);
  sharp[0].init(1);
  sharp[1].init(2);
  sharp[2].init(3, Range::kMax);
  sharp[3].init(0);
}

void loop() {
  sharp[0].Update();
  sharp[1].Update();
  sharp[2].Update();
  sharp[3].Update();

  if (cont < millis()) {
    lcd.clear();
    Serial.print(sharp[0].Avg());
    lcd.print(sharp[0].Avg());
    Serial.print("\t");
    Serial.print(sharp[1].Avg());
    lcd.setCursor(8, 0);
    lcd.print(sharp[1].Avg());
    Serial.print("\t");
    Serial.print(sharp[2].Avg());
    lcd.setCursor(0, 1);
    lcd.print(sharp[2].Avg());
    Serial.print("\t");
    Serial.println(sharp[3].Avg());
    lcd.setCursor(8, 1);
    lcd.print(sharp[3].Avg());
    cont = millis() + 100;
  }
}
