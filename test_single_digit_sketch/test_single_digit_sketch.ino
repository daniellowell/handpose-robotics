#include <Servo.h>
Servo s;
const int pins[] = {3,5,6,9,10,11}; // common mapping; we'll discover the real one

void setup() {
  Serial.begin(9600);
  Serial.println("One-Channel Tester");
  Serial.println("Commands:");
  Serial.println("  p <pin>   (e.g., 'p 9')   -> attach on that UNO pin");
  Serial.println("  a <deg>   (70..110)       -> move to angle");
  Serial.println("  d                         -> detach (quiet)");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n'); line.trim();
    if (line.length() == 0) return;

    if (line[0] == 'p') {
      int pin;
      if (sscanf(line.c_str(), "p %d", &pin)==1) {
        s.detach();
        delay(100);
        s.attach(pin);
        s.write(90);
        Serial.print("Attached on D"); Serial.println(pin);
      } else {
        Serial.println("Usage: p <pin>");
      }
    } else if (line[0] == 'a') {
      int ang;
      if (sscanf(line.c_str(), "a %d", &ang)==1) {
        ang = constrain(ang, 70, 110);
        s.write(ang);
        Serial.print("Angle "); Serial.println(ang);
      } else {
        Serial.println("Usage: a <deg 70..110>");
      }
    } else if (line[0] == 'd') {
      s.detach();
      Serial.println("Detached (quiet).");
    } else {
      Serial.println("Unknown. Use: p <pin> | a <deg> | d");
    }
  }
}
