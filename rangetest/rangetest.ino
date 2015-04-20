#include "Arduino.h"
#include "range.h"

Range *sharp;

unsigned long cont;
void setup() {
  cont = 0;
  sharp = new Range[4];
  Serial.begin(115200);
  sharp[0].init(0);
  sharp[1].init(1);
  sharp[2].init(2);
  sharp[3].init(3);
}

void loop() {
  sharp[0].Update();
  sharp[1].Update();
  sharp[2].Update();
  sharp[3].Update();

  if (cont < millis()) {
    Serial.print(sharp[0].Avg());
    Serial.print("\t");
    Serial.print(sharp[1].Avg());
    Serial.print("\t");
    Serial.print(sharp[2].Avg());
    Serial.print("\t");
    Serial.println(sharp[3].Avg());
    cont = millis() + 100;
  }
}
