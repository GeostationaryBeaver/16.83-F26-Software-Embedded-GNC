/* ===========================================================================
 * vl53l0x_bringup.ino -- confirm the VL53L0X is wired correctly and working.
 *
 * TARGET: Arduino Nano ESP32, Arduino IDE (this is C++, not the ESP-IDF C).
 *
 * WIRING (4 wires):
 *     breakout VIN  -> 3V3     <-- NOT 5V
 *     breakout GND  -> GND
 *     breakout SDA  -> A4
 *     breakout SCL  -> A5
 *     XSHUT, GPIO1  -> leave unconnected
 *
 * LIBRARY: Sketch > Include Library > Manage Libraries, search "VL53L0X",
 *          install "VL53L0X" by Pololu.
 *
 * WHAT IT DOES
 *   Phase 1: scans the I2C bus and reports every address that answers.
 *   Phase 2: if 0x29 answered, initialises the sensor and streams distance.
 *
 * The two phases separate the two things that can be wrong. A failure in
 * phase 1 is electrical -- power, wiring, a dead board. A failure in phase 2
 * with phase 1 passing means the wiring is fine and the problem is the chip
 * or its configuration. Knowing which saves a lot of guessing.
 * ======================================================================== */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

/* Did the scan find the sensor? Gates phase 2. */
bool sensorPresent = false;

/* Did init() succeed? Gates the ranging loop. */
bool sensorReady = false;

static const uint8_t VL53L0X_ADDR = 0x29;


/* --- Phase 1: bus scan -------------------------------------------------
 * Walks every legal 7-bit address and checks for an acknowledgement.
 * Wire.endTransmission() returning 0 means a device ACKed -- something is
 * physically there and responding.
 *
 * Addresses 0x00-0x07 and 0x78-0x7F are reserved by the I2C spec, so we
 * scan 0x08..0x77.
 * -------------------------------------------------------------------- */
void scanBus()
{
  Serial.println(F("--- I2C bus scan ---"));

  int found = 0;

  for (uint8_t addr = 0x08; addr < 0x78; addr++) {

    Wire.beginTransmission(addr);
    uint8_t result = Wire.endTransmission();

    if (result == 0) {
      Serial.print(F("  device at 0x"));
      if (addr < 0x10) Serial.print('0');
      Serial.print(addr, HEX);

      if (addr == VL53L0X_ADDR) {
        Serial.print(F("  <-- VL53L0X"));
        sensorPresent = true;
      }
      Serial.println();
      found++;
    }
  }

  if (found == 0) {
    Serial.println(F("  nothing responded."));
    Serial.println();
    Serial.println(F("  Check, in this order:"));
    Serial.println(F("   1. VIN on 3V3 (NOT 5V, and not VBUS)"));
    Serial.println(F("   2. GND connected"));
    Serial.println(F("   3. SDA on A4, SCL on A5 -- and not swapped"));
    Serial.println(F("   4. jumper wires actually seated / not broken"));
  } else {
    Serial.print(F("  "));
    Serial.print(found);
    Serial.println(F(" device(s) found."));
  }
  Serial.println();
}


void setup()
{
  Serial.begin(115200);

  /* The Nano ESP32 uses the ESP32-S3's native USB, so the serial port only
   * exists once the host has opened it. Without this wait the first several
   * lines vanish before the monitor attaches.
   *
   * The 3-second cap matters: `while (!Serial);` alone would hang forever
   * when running on battery with no host attached. Always bound this. */
  unsigned long start = millis();
  while (!Serial && (millis() - start) < 3000) {
    delay(10);
  }

  delay(500);          /* let the monitor settle before the first output */

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F(" VL53L0X bring-up test"));
  Serial.println(F(" Arduino Nano ESP32"));
  Serial.println(F("========================================"));
  Serial.println();

  /* Pass the pins explicitly, using the Arduino LABELS A4/A5. The core
   * remaps those to raw GPIO 11 and 12 for us.
   *
   * Do NOT write Wire.begin(21, 22) here -- that is the classic ESP32
   * pairing, and the ESP32-S3 has no GPIO 22 at all. */
  Wire.begin(A4, A5);

  /* Start the scan at the default 100 kHz. Slower is more forgiving of long
   * jumper wires and marginal pull-ups, so if something is going to answer
   * at all, it will answer here. We step up to 400 kHz after. */
  Wire.setClock(100000);

  scanBus();

  if (!sensorPresent) {
    Serial.println(F("VL53L0X not found at 0x29 -- stopping."));
    Serial.println(F("Fix the wiring, then reset the board."));
    return;             /* loop() will idle */
  }

  /* --- Phase 2: initialise ------------------------------------------- */
  Serial.println(F("--- sensor init ---"));

  Wire.setClock(400000);        /* VL53L0X maximum */

  sensor.setTimeout(500);

  /* init() reads the model ID, loads the tuning registers, manages the
   * reference SPADs, and runs VHV/phase calibration. It takes ~50 ms. */
  if (!sensor.init()) {
    Serial.println(F("  init() FAILED."));
    Serial.println();
    Serial.println(F("  The chip answered the scan but would not"));
    Serial.println(F("  initialise. Usually one of:"));
    Serial.println(F("   - marginal power (try a different 3V3 wire)"));
    Serial.println(F("   - 400 kHz too fast for your wiring"));
    Serial.println(F("     (try Wire.setClock(100000) instead)"));
    Serial.println(F("   - a counterfeit or damaged module"));
    return;
  }

  Serial.println(F("  init() ok"));

  /* 33 ms budget: the balanced default. ~1.2 m indoors. */
  sensor.setMeasurementTimingBudget(33000);

  /* Continuous back-to-back ranging, paced by the timing budget. */
  sensor.startContinuous();

  sensorReady = true;

  Serial.println(F("  continuous mode started"));
  Serial.println();
  Serial.println(F("--- ranging ---"));
  Serial.println(F("Wave your hand over the sensor. Expect roughly"));
  Serial.println(F("50-1200 mm. 8190 means no target / out of range."));
  Serial.println();
}


void loop()
{
  /* Nothing to do if we never got a working sensor. Idle visibly rather
   * than silently, so the console shows why. */
  if (!sensorReady) {
    Serial.println(F("halted -- fix wiring and reset"));
    delay(5000);
    return;
  }

  uint16_t mm = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    Serial.println(F("TIMEOUT -- sensor stopped responding"));
  }
  else if (mm >= 8190) {
    /* Not an error: the chip reports this when nothing is in range. */
    Serial.println(F("---- mm   (no target in range)"));
  }
  else {
    Serial.print(mm);
    Serial.print(F(" mm"));

    /* A crude bar, so you can see the value track your hand without
     * reading numbers. One block per 50 mm, capped at 20 blocks. */
    Serial.print(F("   "));
    uint8_t bars = mm / 50;
    if (bars > 20) bars = 20;
    for (uint8_t i = 0; i < bars; i++) Serial.print('#');

    Serial.println();
  }

  delay(100);          /* 10 Hz: readable in the serial monitor */
}
