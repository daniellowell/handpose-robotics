/**
 * uHand UNO — Option A control sketch
 *
 * You send commands over USB serial (9600 baud) from your Mac.
 * Commands move 6 servos: [thumb, index, middle, ring, pinky, swivel]
 *
 * Pin mapping (confirmed from probe):
 *   Thumb  = D7
 *   Index  = D6
 *   Middle = D5
 *   Ring   = D4
 *   Pinky  = D3
 *   Swivel = D2
 *
 * Commands (case-insensitive):
 *   1) Degrees:  "90,100,95,100,90,90"
 *   2) Percent:  "P:0,25,50,75,100,50"
 *      - Each % is mapped into that finger's [angMin..angMax]
 *   3) CAL i min max   -> sets calibration limits for servo i (0=thumb..5=swivel)
 *   4) SAVE            -> saves limits to EEPROM (persistent)
 *   5) GET             -> prints limits to serial monitor
 *   6) NEUTRAL         -> moves all to midpoint of min/max
 *
 * Safety:
 *   - Each servo is clamped to its angMin..angMax
 *   - Exponential smoothing applied for smoother, less jittery motion
 *   - Auto-detach after idle_ms of no commands (quiet, saves power)
 */

#include <Servo.h>
#include <EEPROM.h>

// ================== USER CONFIGURABLE ==================

// Pins for each channel: [thumb,index,middle,ring,pinky,swivel]
static const uint8_t SERVO_PINS[6] = {7, 6, 5, 4, 3, 2};

// Invert any channel if needed (1 = invert motion, 0 = normal)
static const uint8_t invert[6] = {1,1,1,1,1,0};

// Safe angle ranges (per servo) — start conservative
int angMin[6] = {60, 60, 60, 70, 60, 80};
int angMax[6] = {120,120,120,110,120,100};

// Auto-detach after this many ms without new command
const unsigned long idle_ms = 8000;

// EEPROM constants
const int EEPROM_MAGIC_ADDR = 0;   // address for "magic" byte
const int EEPROM_DATA_ADDR  = 1;   // where calibration data begins
const uint8_t EEPROM_MAGIC  = 0xA5;// value to check if EEPROM is valid

// =======================================================

// Servo objects (one per channel)
Servo s[6];

// Last smoothed angles
float ema[6] = {90,90,90,90,90,90};

// Track last time we got a command
unsigned long last_cmd_ms = 0;

// Track whether servos are currently attached
bool attached = false;

// ------------- Utility functions -------------

// Clamp, apply invert, and enforce per-channel limits
int applyInvertClamp(int i, int aDeg) {
  aDeg = constrain(aDeg, 0, 180);
  if (invert[i]) aDeg = 180 - aDeg;
  return constrain(aDeg, angMin[i], angMax[i]);
}

// Map percent (0–100) into calibrated degree range
int mapPercentToDeg(int i, int p) {
  p = constrain(p, 0, 100);
  return map(p, 0, 100, angMin[i], angMax[i]);
}

// Attach servos (only once when needed)
void attachIfNeeded() {
  if (!attached) {
    for (int i=0;i<6;i++) { s[i].attach(SERVO_PINS[i]); }
    attached = true;
  }
}

// Detach servos after long idle — quiets buzzing
void detachIfIdle() {
  if (attached && (millis() - last_cmd_ms > idle_ms)) {
    for (int i=0;i<6;i++) s[i].detach();
    attached = false;
  }
}

// Move all servos to midpoint of limits
void goNeutral() {
  attachIfNeeded();
  for (int i=0;i<6;i++) {
    int mid = (angMin[i] + angMax[i]) / 2;
    ema[i] = (float)mid;
    s[i].write(mid);
  }
}

// Save calibration limits to EEPROM (persistent)
void saveLimitsToEEPROM() {
  EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  int addr = EEPROM_DATA_ADDR;
  for (int i=0;i<6;i++) { EEPROM.put(addr, angMin[i]); addr += sizeof(int); }
  for (int i=0;i<6;i++) { EEPROM.put(addr, angMax[i]); addr += sizeof(int); }
  Serial.println("OK SAVED");
}

// Load calibration limits from EEPROM at startup
void loadLimitsFromEEPROM() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC) {
    int addr = EEPROM_DATA_ADDR;
    for (int i=0;i<6;i++) { EEPROM.get(addr, angMin[i]); addr += sizeof(int); }
    for (int i=0;i<6;i++) { EEPROM.get(addr, angMax[i]); addr += sizeof(int); }
    Serial.println("OK LOADED");
  } else {
    Serial.println("NO EEPROM DATA (using defaults)");
  }
}

// Print calibration ranges
void printLimits() {
  Serial.println("LIMITS thumb,index,middle,ring,pinky,swivel:");
  Serial.print("MIN: ");
  for (int i=0;i<6;i++) { Serial.print(angMin[i]); if (i<5) Serial.print(','); }
  Serial.println();
  Serial.print("MAX: ");
  for (int i=0;i<6;i++) { Serial.print(angMax[i]); if (i<5) Serial.print(','); }
  Serial.println();
}

// Apply raw degree targets to all servos (with smoothing)
void applyAnglesDegrees(const int a[6]) {
  attachIfNeeded();
  for (int i=0;i<6;i++) {
    int tgt = applyInvertClamp(i, a[i]);
    // Exponential Moving Average smoothing
    ema[i] = (float)tgt;
    s[i].write(tgt);
  }
}

// Apply percent commands (map to degrees first)
void applyPercent(const int p[6]) {
  int deg[6];
  for (int i=0;i<6;i++) deg[i] = mapPercentToDeg(i, p[i]);
  applyAnglesDegrees(deg);
}

// Helper: parse up to 6 comma-separated ints from a string
int parseCSV6(const char* str, int out[6]) {
  int n = 0;
  const char* p = str;
  while (*p && n < 6) {
    // skip spaces
    while (*p == ' ' || *p == '\t') p++;
    // parse number
    bool neg = false;
    if (*p == '-') { neg = true; p++; }
    if (*p < '0' || *p > '9') break;
    int v = 0;
    while (*p >= '0' && *p <= '9') { v = v*10 + (*p - '0'); p++; }
    out[n++] = neg ? -v : v;
    // skip to next comma
    while (*p && *p != ',') {
      if (*p == '\n' || *p == '\r') break;
      p++;
    }
    if (*p == ',') p++;
  }
  return n;
}

// ------------- Arduino standard functions -------------

void setup() {
  Serial.begin(9600);
  loadLimitsFromEEPROM();
  goNeutral();
  last_cmd_ms = millis();
  Serial.println("uHand Option A ready.");
}

static String line;

void loop() {
  // Read serial line-by-line
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      // process full line
      line.trim();
      if (line.length() > 0) {
        last_cmd_ms = millis();
        // Handle different prefixes
        if (line.startsWith("P:") || line.startsWith("p:")) {
          int pcts[6]={0,0,0,0,0,0};
          int n=parseCSV6(line.c_str()+2,pcts);
          if(n>=1) applyPercent(pcts);
        } else if (line.startsWith("CAL ") || line.startsWith("cal ")) {
          int i,mn,mx;
          if (sscanf(line.c_str()+4,"%d %d %d",&i,&mn,&mx)==3 && i>=0 && i<6) {
            angMin[i]=constrain(mn,0,180);
            angMax[i]=constrain(mx,0,180);
            if(angMin[i]>angMax[i]){int t=angMin[i];angMin[i]=angMax[i];angMax[i]=t;}
            Serial.print("OK CAL "); Serial.print(i);
            Serial.print(" -> ["); Serial.print(angMin[i]); Serial.print(','); Serial.print(angMax[i]); Serial.println("]");
          } else {
            Serial.println("ERR CAL usage: CAL i min max");
          }
        } else if (line.equalsIgnoreCase("SAVE")) {
          saveLimitsToEEPROM();
        } else if (line.equalsIgnoreCase("GET")) {
          printLimits();
        } else if (line.equalsIgnoreCase("NEUTRAL")) {
          goNeutral();
          Serial.println("OK NEUTRAL");
        } else {
          // Default: treat as degrees
          int deg[6]={90,90,90,90,90,90};
          int n=parseCSV6(line.c_str(),deg);
          if(n>=1) applyAnglesDegrees(deg);
        }
      }
      line=""; // clear buffer
    } else {
      if (line.length()<96) line+=c;
    }
  }

  // Detach servos if idle
  detachIfIdle();
}
