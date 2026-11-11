/*
  TG-AV-003  VEHICLE DOCTOR   (vehicle Nano only, run before any powered test)
  ============================================================================
  Checks every part on the vehicle board and says PASS / CHECK / FAIL for each, with what to do.
  No radio. The servo only gets a signal for one short test, and it cannot move without the
  arming plug (no servo power), so this is safe with the battery connected.

  Upload to the VEHICLE Nano, Serial Monitor 115200, line ending "New Line".
  It prints a full report, then live readings every second.
  Type  r  + Enter to run the full report again (after you change wiring).

  Needs the library "VL53L0X" by Pololu for the laser
  (Sketch > Include Library > Manage Libraries, search VL53L0X, pick Pololu).

  Sections
    1. Wiring shorts    drives each signal pin low and checks no other signal pin follows
    2. Laser VL53L0X    I2C A4/A5: present, chip ID, distance
    3. IMU LSM6DS3      I2C D2/D3: present, chip ID, acceleration (should total ~1 g at rest), gyro
    4. Temp DS18B20     1-Wire D4: present, power wiring, temperature
    5. Pressure PT1     A0: loop current through the 100 ohm shunt (open air = 4.0 mA = 0 psi)
    6. Servo supply     A1: UBEC 6 V seen through the divider (arming plug OUT = about 0 V)
    7. Servo signal     D5: runs the servo signal for 1 s and checks it does not disturb the IMU
*/

#include <Wire.h>
// Needs the "VL53L0X" library by Pololu (Library Manager). A plain #include is required:
// the Arduino IDE only adds a library to the build when it sees the #include directly.
#include <VL53L0X.h>
#define HAVE_TOF_LIB 1
VL53L0X tof;

// ---------------- pins (rev E) ----------------
// Servo and analog: pass the chip pin if the core does not translate the Arduino name for that call
#if defined(ledcAttachPin) || defined(ledcAttach)
const uint8_t PIN_SERVO = D5;
#else
const uint8_t PIN_SERVO = 8;            // GPIO8 = D5
#endif
#if defined(analogReadMilliVolts)
const uint8_t PIN_PRESSURE = A0, PIN_SERVO_V = A1;
#else
const uint8_t PIN_PRESSURE = 1, PIN_SERVO_V = 2;   // GPIO1 = A0, GPIO2 = A1
#endif
const uint8_t PIN_OW = D4;              // DS18B20 data, R7 4.7k pull-up to 3V3
const uint8_t LASER_ADDR = 0x29;

// All types are declared here, before the first function: the Arduino IDE adds function
// declarations above the first function, so any type a function uses must already exist.
enum Result { R_PASS, R_CHECK, R_FAIL, R_SKIP };
enum Bus { BUS_NONE, BUS_LASER, BUS_IMU };
Bus curBus = BUS_NONE;
struct Sig { uint8_t pin; const char *name; };
Sig sigs[] = { {D2, "D2 (IMU SDA)"}, {D3, "D3 (IMU SCL)"}, {D4, "D4 (temp DQ)"},
               {D5, "D5 (servo signal)"}, {A4, "A4 (laser SDA)"}, {A5, "A5 (laser SCL)"} };
const int NUM_SIGS = sizeof(sigs) / sizeof(sigs[0]);
const char *rname(Result r) { return r == R_PASS ? "PASS " : r == R_CHECK ? "CHECK" : r == R_FAIL ? "FAIL " : "-    "; }
Result rShort, rLaser, rImu, rTemp, rPress, rServoV, rServoSig;
uint8_t imuAddr = 0;

// =============================================================================
// I2C helpers: one controller (Wire), switched between the laser pins and the IMU pins.
// This is the arrangement that worked reliably on this board.
// =============================================================================

void busClear(uint8_t sda, uint8_t scl) {      // frees a sensor stuck half-way through a message
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 9; i++) { digitalWrite(scl, LOW); delayMicroseconds(5); digitalWrite(scl, HIGH); delayMicroseconds(5); }
  pinMode(sda, OUTPUT_OPEN_DRAIN);
  digitalWrite(sda, LOW); delayMicroseconds(5); digitalWrite(scl, HIGH); delayMicroseconds(5); digitalWrite(sda, HIGH);
  pinMode(sda, INPUT); pinMode(scl, INPUT);
}
void useBus(Bus b) {
  if (b == curBus) return;
  if (curBus != BUS_NONE) Wire.end();
  uint8_t sda = b == BUS_LASER ? A4 : D2, scl = b == BUS_LASER ? A5 : D3;
  busClear(sda, scl);
  Wire.begin(sda, scl, 100000);
  Wire.setTimeOut(20);
  for (uint8_t a = 0x08; a < 0x10; a++) { Wire.beginTransmission(a); Wire.endTransmission(); }   // warm-up
  curBus = b;
}
bool probe(uint8_t addr) {
  for (int i = 0; i < 3; i++) { Wire.beginTransmission(addr); if (Wire.endTransmission() == 0) return true; delay(1); }
  return false;
}
bool regWrite(uint8_t addr, uint8_t reg, uint8_t v) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}
bool regRead(uint8_t addr, uint8_t reg, uint8_t *buf, int n) {
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, n) != n) return false;
  for (int i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
bool idleHigh(uint8_t sda, uint8_t scl) {      // with the chip's weak pull-down: HIGH only if an external pull-up is there
  pinMode(sda, INPUT_PULLDOWN); pinMode(scl, INPUT_PULLDOWN); delay(3);
  bool ok = digitalRead(sda) && digitalRead(scl);
  pinMode(sda, INPUT); pinMode(scl, INPUT);
  return ok;
}

// =============================================================================
// 1. Wiring shorts
// =============================================================================

void testShorts() {
  Serial.println("\n1. WIRING SHORTS (each signal pin pulled low, the others watched)");
  if (curBus != BUS_NONE) { Wire.end(); curBus = BUS_NONE; }
  int shorts = 0, stuck = 0;
  bool idleLow[NUM_SIGS];
  for (int i = 0; i < NUM_SIGS; i++) pinMode(sigs[i].pin, INPUT_PULLUP);
  delay(5);
  for (int i = 0; i < NUM_SIGS; i++) {
    idleLow[i] = digitalRead(sigs[i].pin) == LOW;
    if (idleLow[i]) {
      bool servoPin = sigs[i].pin == D5;
      Serial.printf("   %-18s reads LOW even with a pull-up: %s\n", sigs[i].name,
                    servoPin ? "OK if the servo lead is plugged in with the servo unpowered" : "held low or shorted to ground");
      if (!servoPin) stuck++;
    }
  }
  for (int i = 0; i < NUM_SIGS; i++) {
    pinMode(sigs[i].pin, OUTPUT); digitalWrite(sigs[i].pin, LOW); delay(2);
    for (int j = 0; j < NUM_SIGS; j++) {
      if (j == i || idleLow[i] || idleLow[j]) continue;       // a pin already low at rest can't be tested this way
      if (digitalRead(sigs[j].pin) == LOW && j > i) {
        Serial.printf("   SHORT: %s is connected to %s\n", sigs[i].name, sigs[j].name);
        shorts++;
      }
    }
    pinMode(sigs[i].pin, INPUT_PULLUP); delay(2);
  }
  for (int i = 0; i < NUM_SIGS; i++) pinMode(sigs[i].pin, INPUT);
  if (!shorts && !stuck) Serial.println("   no shorts between D2, D3, D4, D5, A4, A5");
  rShort = shorts ? R_FAIL : stuck ? R_CHECK : R_PASS;
}

// =============================================================================
// 2. Laser
// =============================================================================
bool laserLibOk = false;
void testLaser() {
  Serial.println("\n2. LASER VL53L0X  (I2C on A4 = SDA, A5 = SCL, row 24-29 board)");
  bool idle = idleHigh(A4, A5);
  Serial.printf("   lines idle: %s\n", idle ? "HIGH, pull-ups and power present" : "LOW: laser unpowered (VCC 24, GND 25) or SDA/SCL wire missing");
  useBus(BUS_LASER);
  if (!probe(LASER_ADDR)) {
    Serial.println("   not answering at 0x29. Check VCC (row 24 -> 3V3), GND (row 25), SDA row 27 <- A4, SCL row 26 <- A5");
    rLaser = R_FAIL; return;
  }
  uint8_t id = 0; regRead(LASER_ADDR, 0xC0, &id, 1);
  Serial.printf("   found at 0x29, model ID 0x%02X %s\n", id, id == 0xEE ? "(VL53L0X)" : "(expected 0xEE)");
  rLaser = id == 0xEE ? R_PASS : R_CHECK;
#if HAVE_TOF_LIB
  tof.setBus(&Wire);
  tof.setTimeout(200);
  laserLibOk = tof.init();
  if (!laserLibOk) { Serial.println("   library init FAILED (power cycle the board and retry)"); rLaser = R_CHECK; return; }
  uint16_t mm = tof.readRangeSingleMillimeters();
  if (tof.timeoutOccurred()) { Serial.println("   distance read timed out"); rLaser = R_CHECK; }
  else if (mm >= 8190)        Serial.println("   distance: out of range (nothing within ~1.2 m). Point it at a surface 10-100 cm away");
  else                        Serial.printf("   distance: %u mm  (check against a tape measure)\n", mm);
#else
  Serial.println("   distance: install the Pololu VL53L0X library to read distance");
#endif
}

// =============================================================================
// 3. IMU (raw registers, no library)
// =============================================================================
bool imuReadMotion(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t b[12];
  if (!regRead(imuAddr, 0x22, b, 12)) return false;            // OUTX_L_G .. OUTZ_H_XL
  auto s16 = [&](int i) { return (int16_t)(b[i] | (b[i + 1] << 8)); };
  gx = s16(0) * 0.00875f; gy = s16(2) * 0.00875f; gz = s16(4) * 0.00875f;   // 250 dps range
  ax = s16(6) * 0.000061f; ay = s16(8) * 0.000061f; az = s16(10) * 0.000061f; // 2 g range
  return true;
}
void testImu() {
  Serial.println("\n3. IMU LSM6DS3  (I2C on D2 = SDA, D3 = SCL)");
  bool idle = idleHigh(D2, D3);
  Serial.printf("   lines idle: %s\n", idle ? "HIGH, pull-ups and power present" : "LOW: IMU unpowered (VIN -> 3V3, GND) or SDA/SCL wire missing");
  useBus(BUS_IMU);
  imuAddr = probe(0x6A) ? 0x6A : probe(0x6B) ? 0x6B : 0;
  if (!imuAddr) {
    Serial.println("   not answering at 0x6A / 0x6B. Check CS -> 3V3, SA0 -> GND, SDA <- D2 (13J->38J), SCL <- D3 (12J->37J), header soldered");
    rImu = R_FAIL; return;
  }
  uint8_t who = 0; regRead(imuAddr, 0x0F, &who, 1);
  const char *chip = who == 0x69 ? "LSM6DS3" : who == 0x6A ? "LSM6DS3TR-C" : who == 0x6C ? "LSM6DSO" : "unexpected";
  Serial.printf("   found at 0x%02X (%s), WHO_AM_I 0x%02X = %s\n", imuAddr, imuAddr == 0x6A ? "SA0 to GND, as designed" : "SA0 high or floating", who, chip);
  regWrite(imuAddr, 0x10, 0x40);       // CTRL1_XL: 104 Hz, +-2 g
  regWrite(imuAddr, 0x11, 0x40);       // CTRL2_G: 104 Hz, 250 dps
  delay(60);
  float ax, ay, az, gx, gy, gz;
  if (!imuReadMotion(ax, ay, az, gx, gy, gz)) { Serial.println("   motion read FAILED"); rImu = R_CHECK; return; }
  float g = sqrtf(ax * ax + ay * ay + az * az);
  Serial.printf("   accel %+.2f %+.2f %+.2f g, total %.2f g   gyro %+.1f %+.1f %+.1f dps\n", ax, ay, az, g, gx, gy, gz);
  bool still = g > 0.9f && g < 1.1f && fabsf(gx) < 10 && fabsf(gy) < 10 && fabsf(gz) < 10;
  Serial.printf("   %s\n", still ? "at rest: total ~1 g and gyro near 0, good" : "keep the board still: total should be ~1.00 g and gyro near 0");
  rImu = (who == 0x69 || who == 0x6A || who == 0x6C) ? (still ? R_PASS : R_CHECK) : R_CHECK;
}

// =============================================================================
// 4. DS18B20 temperature probe (1-Wire on D4, done in software, no library)
// =============================================================================
portMUX_TYPE owMux = portMUX_INITIALIZER_UNLOCKED;
inline void owLow()     { digitalWrite(PIN_OW, LOW); }
inline void owRelease() { digitalWrite(PIN_OW, HIGH); }   // open drain: HIGH = let R7 pull it up

bool owReset() {
  owRelease(); delayMicroseconds(10);
  owLow(); delayMicroseconds(480);
  portENTER_CRITICAL(&owMux);
  owRelease(); delayMicroseconds(70);
  bool present = digitalRead(PIN_OW) == LOW;
  portEXIT_CRITICAL(&owMux);
  delayMicroseconds(410);
  return present;
}
void owWriteBit(bool b) {
  portENTER_CRITICAL(&owMux);
  owLow(); delayMicroseconds(b ? 6 : 60);
  owRelease(); delayMicroseconds(b ? 64 : 10);
  portEXIT_CRITICAL(&owMux);
}
bool owReadBit() {
  portENTER_CRITICAL(&owMux);
  owLow(); delayMicroseconds(3);
  owRelease(); delayMicroseconds(10);
  bool b = digitalRead(PIN_OW);
  portEXIT_CRITICAL(&owMux);
  delayMicroseconds(53);
  return b;
}
void owWrite(uint8_t v) { for (int i = 0; i < 8; i++) owWriteBit((v >> i) & 1); }
uint8_t owRead() { uint8_t v = 0; for (int i = 0; i < 8; i++) if (owReadBit()) v |= 1 << i; return v; }
uint8_t crc8(const uint8_t *d, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; i++) { uint8_t x = d[i]; for (int b = 0; b < 8; b++) { uint8_t mix = (c ^ x) & 1; c >>= 1; if (mix) c ^= 0x8C; x >>= 1; } }
  return c;
}
// returns temperature in C, or a code below -200 for errors
float dsReadTemp(bool &parasite) {
  if (!owReset()) return -300;
  owWrite(0xCC); owWrite(0xB4);                 // read power supply: 0 = parasite (VDD not on 3V3)
  parasite = !owReadBit();
  if (!owReset()) return -300;
  owWrite(0xCC); owWrite(0x44);                 // start conversion
  uint32_t t0 = millis();
  if (parasite) { pinMode(PIN_OW, OUTPUT); digitalWrite(PIN_OW, HIGH); delay(760); pinMode(PIN_OW, OUTPUT_OPEN_DRAIN); owRelease(); }
  else while (!owReadBit() && millis() - t0 < 900) delay(5);
  if (!owReset()) return -300;
  owWrite(0xCC); owWrite(0xBE);
  uint8_t sp[9]; for (int i = 0; i < 9; i++) sp[i] = owRead();
  if (crc8(sp, 8) != sp[8]) return -301;
  return (int16_t)(sp[0] | (sp[1] << 8)) / 16.0f;
}
void testTemp() {
  Serial.println("\n4. TEMP PROBE DS18B20  (1-Wire on D4, TB5: VDD row 43, DQ row 45, GND row 47, R7 pull-up)");
  pinMode(PIN_OW, INPUT); delay(2);
  if (digitalRead(PIN_OW) == LOW) {
    Serial.println("   DQ line is LOW at rest: the probe's GND wire is probably in the DQ screw, or R7 is missing");
    rTemp = R_FAIL; return;
  }
  pinMode(PIN_OW, OUTPUT_OPEN_DRAIN); owRelease();
  if (!owReset()) {
    Serial.println("   no probe answered. The probe's data wire is not reaching D4:");
    Serial.println("   swap the two wires that are NOT ground (DQ and VDD screws) and run again (type r)");
    rTemp = R_FAIL; return;
  }
  owWrite(0x33);                                // read ROM (one probe on the line)
  uint8_t rom[8]; for (int i = 0; i < 8; i++) rom[i] = owRead();
  bool romOk = crc8(rom, 7) == rom[7];
  Serial.printf("   probe answered. ROM %02X-%02X%02X%02X%02X%02X%02X %s\n", rom[0], rom[6], rom[5], rom[4], rom[3], rom[2], rom[1],
                !romOk ? "(CRC error: loose wire?)" : rom[0] == 0x28 ? "(DS18B20)" : "(not a DS18B20 family code)");
  bool parasite = false;
  float t = dsReadTemp(parasite);
  if (parasite) Serial.println("   power: PARASITE. The probe's VDD wire is not on 3V3 (probably in the GND screw). Works, but move it to VDD");
  else          Serial.println("   power: VDD on 3V3, as designed");
  if (t < -200)            { Serial.println("   temperature read FAILED (CRC). Check the screws are tight"); rTemp = R_FAIL; return; }
  if (t == 85.0f)          { Serial.println("   reads 85.0 C: the power-on value, conversion did not run. Check VDD"); rTemp = R_CHECK; return; }
  Serial.printf("   temperature %.2f C  %s\n", t, (t > 5 && t < 40) ? "(room temperature, good. Hold the tip and it should rise)" : "(unusual for a room)");
  rTemp = (romOk && !parasite && t > 5 && t < 40) ? R_PASS : R_CHECK;
}

// =============================================================================
// 5. Pressure loop and 6. servo supply (analog)
// =============================================================================
void readAnalog(uint8_t pin, uint32_t &avg, uint32_t &lo, uint32_t &hi) {
  uint32_t sum = 0; lo = 99999; hi = 0;
  for (int i = 0; i < 64; i++) { uint32_t v = analogReadMilliVolts(pin); sum += v; if (v < lo) lo = v; if (v > hi) hi = v; delayMicroseconds(300); }
  avg = sum / 64;
}
void testPressure() {
  Serial.println("\n5. PRESSURE TRANSMITTER PT1  (A0, 100 ohm shunt R3, R4 10k, loop powered from the 12 V rail via TB4)");
  uint32_t mv, lo, hi; readAnalog(PIN_PRESSURE, mv, lo, hi);
  float mA = mv / 100.0f, psi = (mA - 4.0f) / 16.0f * 1000.0f;
  Serial.printf("   A0 %u mV (min %u, max %u) = %.2f mA", mv, lo, hi, mA);
  if (hi - lo > 300) {
    Serial.println("\n   reading jumps around: A0 is FLOATING. The A0 wire (6A -> 56A), R4 (56E-60E) or R3 (60A -> left - rail) is open,");
    Serial.println("   or the left - rail is not joined to the Nano GND (16A -> left - rail)");
    rPress = R_FAIL;
  } else if (mA < 3.5f) {
    Serial.println("\n   no loop current. Battery on? Fuse good? 12 V reaching TB4 + (58A -> left + rail)? PT1 wires tight in TB4?");
    Serial.println("   (PT1 needs 12 V: it shows nothing on USB power alone)");
    rPress = R_FAIL;
  } else if (mA <= 4.4f) {
    Serial.printf(" = %.0f psi. Open air should read ~4.0 mA / 0 psi: good\n", psi < 0 ? 0 : psi);
    rPress = R_PASS;
  } else if (mA <= 20.5f) {
    Serial.printf(" = %.0f psi. In open air this should be ~0 psi: check the 100 ohm shunt value and grounding\n", psi);
    rPress = R_CHECK;
  } else {
    Serial.println("\n   above 20 mA: shunt missing or wrong, or a short. Unplug PT1 until fixed");
    rPress = R_FAIL;
  }
}
void testServoSupply() {
  Serial.println("\n6. SERVO SUPPLY  (A1 sees the UBEC 6 V through R5 100k / R6 47k)");
  uint32_t mv, lo, hi; readAnalog(PIN_SERVO_V, mv, lo, hi);
  float v = mv * (147.0f / 47.0f) / 1000.0f;
  Serial.printf("   A1 %u mV (min %u, max %u) -> servo rail %.2f V\n", mv, lo, hi, v);
  if (hi - lo > 300)      { Serial.println("   reading jumps around: A1 FLOATING. Check 7A -> 40A, R6 (40D-45D), 45A -> left - rail"); rServoV = R_FAIL; }
  else if (v < 0.5f)      { Serial.println("   0 V: correct with the arming plug OUT (UBEC unpowered)"); rServoV = R_PASS; }
  else if (v < 5.5f)      { Serial.println("   partly powered: plug out but voltage present? check C1 discharge / wiring"); rServoV = R_CHECK; }
  else if (v <= 6.6f)     { Serial.println("   ~6 V: UBEC is ON (arming plug in). Fine if intended"); rServoV = R_PASS; }
  else                    { Serial.println("   ABOVE 6.6 V: UBEC not set to 6.0 V. Unplug the servo, fix the UBEC jumper"); rServoV = R_FAIL; }
}

// =============================================================================
// 7. Servo signal does not disturb the IMU
// =============================================================================
void servoOn() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_SERVO, 50, 14); ledcWrite(PIN_SERVO, 500UL * 16384UL / 20000UL);
#else
  ledcSetup(0, 50, 14); ledcAttachPin(PIN_SERVO, 0); ledcWrite(0, 500UL * 16384UL / 20000UL);
#endif
}
void servoOff() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(PIN_SERVO);
#else
  ledcDetachPin(PIN_SERVO);
#endif
  pinMode(D5, INPUT);
}
void testServoSignal() {
  Serial.println("\n7. SERVO SIGNAL  (D5 -> R1 -> servo orange lead, 500 us = valve shut; servo has no power without the arming plug)");
  if (rImu == R_FAIL) { Serial.println("   skipped: IMU not found, so this test can't tell anything"); rServoSig = R_SKIP; return; }
  servoOn(); delay(300);
  useBus(BUS_IMU);
  bool ok1 = imuAddr && probe(imuAddr);
  delay(700);
  bool ok2 = imuAddr && probe(imuAddr);
  servoOff();
  Serial.printf("   IMU with the servo signal running: %s\n", (ok1 && ok2) ? "still answering, good" : "LOST: the servo signal reaches the IMU wires");
  if (!(ok1 && ok2)) Serial.println("   check for anything joining row 10 (D5) or row 51 (R1 / servo signal) to rows 12, 13, 37, 38");
  rServoSig = (ok1 && ok2) ? R_PASS : R_FAIL;
}

// =============================================================================
void fullReport() {
  Serial.println("\n================ TG-AV-003 VEHICLE DOCTOR ================");
  Serial.printf("Pins: servo -> %s, analog -> %s, laser library: %s\n",
                PIN_SERVO == 8 ? "GPIO8 direct" : "D5 via core", PIN_PRESSURE == 1 ? "GPIO1/2 direct" : "A0/A1 via core",
                HAVE_TOF_LIB ? "installed" : "NOT installed (presence only)");
  testShorts();
  testLaser();
  testImu();
  testTemp();
  testPressure();
  testServoSupply();
  testServoSignal();
  Serial.println("\n---------------- SUMMARY ----------------");
  Serial.printf("  %s  Wiring shorts\n", rname(rShort));
  Serial.printf("  %s  Laser VL53L0X\n", rname(rLaser));
  Serial.printf("  %s  IMU LSM6DS3\n", rname(rImu));
  Serial.printf("  %s  Temp probe DS18B20\n", rname(rTemp));
  Serial.printf("  %s  Pressure transmitter loop\n", rname(rPress));
  Serial.printf("  %s  Servo supply (A1)\n", rname(rServoV));
  Serial.printf("  %s  Servo signal vs IMU\n", rname(rServoSig));
  Serial.println("Live readings every second below. Type r + Enter to re-run this report.\n");
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) {}
  delay(300);
  fullReport();
}

uint32_t lastLive = 0;
void loop() {
  while (Serial.available()) { char c = Serial.read(); if (c == 'r' || c == 'R') fullReport(); }
  if (millis() - lastLive < 1000) return;
  lastLive = millis();

  char laser[24] = "laser --";
#if HAVE_TOF_LIB
  if (rLaser != R_FAIL && laserLibOk) {
    useBus(BUS_LASER);
    uint16_t mm = tof.readRangeSingleMillimeters();
    if (tof.timeoutOccurred()) snprintf(laser, sizeof laser, "laser timeout");
    else if (mm >= 8190)       snprintf(laser, sizeof laser, "laser >1.2 m");
    else                       snprintf(laser, sizeof laser, "laser %4u mm", mm);
  }
#endif
  char imu[48] = "IMU --";
  if (imuAddr) {
    useBus(BUS_IMU);
    float ax, ay, az, gx, gy, gz;
    if (imuReadMotion(ax, ay, az, gx, gy, gz))
      snprintf(imu, sizeof imu, "accel %+.2f %+.2f %+.2f g", ax, ay, az);
  }
  char temp[20] = "temp --";
  if (rTemp != R_FAIL) {
    pinMode(PIN_OW, OUTPUT_OPEN_DRAIN); owRelease();
    bool par; float t = dsReadTemp(par);
    if (t > -200) snprintf(temp, sizeof temp, "temp %.1f C", t);
  }
  uint32_t mv, lo, hi;
  readAnalog(PIN_PRESSURE, mv, lo, hi);
  float mA = mv / 100.0f, psi = (mA - 4.0f) / 16.0f * 1000.0f;
  uint32_t sv, slo, shi;
  readAnalog(PIN_SERVO_V, sv, slo, shi);
  Serial.printf("%s | %s | %s | pressure %.2f mA = %.0f psi | servo rail %.2f V\n",
                laser, imu, temp, mA, mA < 3.5f ? 0.0f : psi, sv * (147.0f / 47.0f) / 1000.0f);
}
