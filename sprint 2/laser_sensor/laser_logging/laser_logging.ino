/* ===========================================================================
 * laser_logging.ino -- stream VL53L0X range data as CSV for LaserLogging.py.
 *
 * TARGET: Arduino Nano ESP32 (ESP32-S3), Arduino IDE. C++, not ESP-IDF C.
 *
 * This is the laser equivalent of loadcell3.ino: it streams comma-separated
 * rows continuously and accepts single-letter commands at any time, so the
 * logger never has to catch the one-shot banner that setup() prints.
 *
 * WIRING (4 wires)
 *     breakout VIN  -> 3V3     <-- NOT 5V
 *     breakout GND  -> GND
 *     breakout SDA  -> A4
 *     breakout SCL  -> A5
 *     XSHUT, GPIO1  -> leave unconnected
 *
 * SETUP
 *   Board:   Tools > Board > Arduino ESP32 Boards > Arduino Nano ESP32
 *            (install "Arduino ESP32 Boards" by Arduino in Boards Manager
 *             first -- without it the IDE compiles for AVR and Wire.begin()
 *             will not accept pin arguments)
 *   Library: "VL53L0X" by Pololu
 *
 * OUTPUT FORMAT
 *   Data rows:    millis,raw_mm,range_mm,status
 *   Status lines: begin with '#' and are never data
 *
 *   raw_mm    straight from the sensor, before offset correction
 *   range_mm  raw + offset; 'nan' when the sample is not trustworthy
 *   status    0 = valid, else the range status code (see statusName())
 *
 *   range_mm is deliberately 'nan' rather than a number whenever the sensor
 *   flags the reading. That mirrors how loadcell3.ino writes nan for force_N
 *   before calibration: the column either holds a trustworthy value or
 *   nothing, so no downstream code has to re-derive validity.
 *
 * COMMANDS (type into Serial Monitor or send from the logger)
 *   z <mm>   zero/calibrate: tell the board the target is truly <mm> away
 *            and let it compute the offset  (e.g. "z 100")
 *   r        reset the offset to zero
 *   p <0-3>  range profile: 0 default, 1 high speed, 2 long range,
 *            3 high accuracy
 *   ?        print current settings
 * ======================================================================== */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

/* --- Tunables ----------------------------------------------------------- */

/* Sensor sanity window, in mm. Readings outside this are reported with a
 * status code and a nan range, not silently passed through. 8190 and friends
 * are the chip's own out-of-range sentinels. */
static const uint16_t SANE_MIN_MM = 30;      /* below this, crosstalk        */
static const uint16_t SANE_MAX_MM = 1600;    /* honest indoor ceiling        */
static const uint16_t OUT_OF_RANGE = 8190;   /* chip sentinel                */

/* Samples averaged when computing an offset with 'z'. */
static const uint8_t CAL_SAMPLES = 50;

/* --- State -------------------------------------------------------------- */

int16_t offsetMm = 0;         /* added to every raw reading                 */
bool    sensorReady = false;

char command[32];
uint8_t commandIndex = 0;


/* --- Status decoding ----------------------------------------------------
 * The Pololu library does not expose the range status register, so we read
 * it directly. RESULT_RANGE_STATUS is 0x14; the status code lives in bits
 * 3:7, hence the shift and mask.
 *
 * Code 11 is the chip's "valid" value, which we map to 0 so that the CSV
 * column reads 0-for-good like an errno. ---------------------------------- */

uint8_t readRangeStatus()
{
  uint8_t raw = sensor.readReg(0x14);
  uint8_t code = (raw & 0x78) >> 3;
  return (code == 11) ? 0 : code;
}

const char *statusName(uint8_t code)
{
  switch (code) {
    case 0:  return "valid";
    case 1:
    case 2:
    case 3:  return "hardware fail";
    case 4:  return "signal too low";
    case 5:  return "below min range";
    case 6:
    case 9:  return "phase/wraparound fail";
    case 8:
    case 10: return "sigma too high";
    default: return "unknown";
  }
}


/* --- Commands ----------------------------------------------------------- */

/* Average CAL_SAMPLES readings and derive the offset that would centre them
 * on trueMm. Blocks for roughly CAL_SAMPLES * timing budget. */
void calibrateOffset(float trueMm)
{
  if (trueMm <= 0) {
    Serial.println(F("# ERROR: give a positive distance, e.g. 'z 100'"));
    return;
  }

  Serial.print(F("# Calibrating against "));
  Serial.print(trueMm, 1);
  Serial.println(F(" mm -- keep the target still"));

  /* Measure with the offset disabled, or we would calibrate against an
   * already-corrected value. */
  int16_t saved = offsetMm;
  offsetMm = 0;

  /* uint32_t accumulator: 50 samples x up to 8190 mm overflows 16 bits after
   * about 8 samples. */
  uint32_t sum = 0;
  uint8_t  good = 0;

  for (uint8_t i = 0; i < CAL_SAMPLES; i++) {
    uint16_t mm = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred()) continue;
    if (readRangeStatus() != 0) continue;
    if (mm < SANE_MIN_MM || mm >= OUT_OF_RANGE) continue;
    sum += mm;
    good++;
  }

  offsetMm = saved;

  /* Demand a solid majority. If a third of the samples failed, the setup is
   * wrong -- bad target, bad lighting, wrong distance -- and an offset from
   * the remainder would bake that error in permanently. */
  if (good < (CAL_SAMPLES * 2 / 3)) {
    Serial.print(F("# ERROR: only "));
    Serial.print(good);
    Serial.print('/');
    Serial.print(CAL_SAMPLES);
    Serial.println(F(" samples valid -- check target and lighting"));
    return;
  }

  uint16_t mean = (uint16_t)((sum + good / 2) / good);   /* rounded mean */
  offsetMm = (int16_t)trueMm - (int16_t)mean;

  Serial.print(F("# Calibration complete: "));
  Serial.print(offsetMm);
  Serial.print(F(" mm offset (mean "));
  Serial.print(mean);
  Serial.print(F(" mm over "));
  Serial.print(good);
  Serial.println(F(" samples)"));
}

void setProfile(int profile)
{
  /* Order matters: signal limit, then VCSEL periods, then timing budget --
   * changing a period perturbs the budget, so the budget goes last. */
  switch (profile) {
    case 1:   /* high speed: 20 ms, 50 Hz, least lag */
      sensor.setSignalRateLimit(0.25);
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
      sensor.setMeasurementTimingBudget(20000);
      Serial.println(F("# Profile: high speed (20 ms)"));
      break;
    case 2:   /* long range: ~2 m, more ambient-sensitive */
      sensor.setSignalRateLimit(0.10);
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
      sensor.setMeasurementTimingBudget(33000);
      Serial.println(F("# Profile: long range (33 ms)"));
      break;
    case 3:   /* high accuracy: 200 ms, 5 Hz */
      sensor.setSignalRateLimit(0.25);
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
      sensor.setMeasurementTimingBudget(200000);
      Serial.println(F("# Profile: high accuracy (200 ms)"));
      break;
    default:  /* balanced */
      sensor.setSignalRateLimit(0.25);
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
      sensor.setMeasurementTimingBudget(33000);
      Serial.println(F("# Profile: default (33 ms)"));
      break;
  }
}

void printSettings()
{
  Serial.print(F("# offset_mm="));
  Serial.print(offsetMm);
  Serial.print(F(" sane_min_mm="));
  Serial.print(SANE_MIN_MM);
  Serial.print(F(" sane_max_mm="));
  Serial.println(SANE_MAX_MM);
}

void processCommand(char *cmd)
{
  if (cmd[0] == 'z' || cmd[0] == 'Z') {
    calibrateOffset(atof(cmd + 1));
  }
  else if (cmd[0] == 'r' || cmd[0] == 'R') {
    offsetMm = 0;
    Serial.println(F("# Offset reset to 0"));
  }
  else if (cmd[0] == 'p' || cmd[0] == 'P') {
    setProfile(atoi(cmd + 1));
  }
  else if (cmd[0] == '?') {
    printSettings();
  }
  else {
    Serial.println(F("# Unknown command"));
    Serial.println(F("# z 100 = calibrate, target truly 100 mm away"));
    Serial.println(F("# r     = reset offset"));
    Serial.println(F("# p 0-3 = profile (0 def, 1 fast, 2 long, 3 accurate)"));
    Serial.println(F("# ?     = show settings"));
  }
}

/* Read commands without blocking the sampling loop. Accumulates characters
 * until a newline, exactly like loadcell3.ino's checkSerial(). */
void checkSerial()
{
  while (Serial.available() > 0) {
    char ch = Serial.read();

    if (ch == '\n' || ch == '\r') {
      if (commandIndex > 0) {
        command[commandIndex] = '\0';
        processCommand(command);
        commandIndex = 0;
      }
    }
    else if (commandIndex < 31) {
      command[commandIndex++] = ch;
    }
  }
}


/* --- Setup -------------------------------------------------------------- */

void setup()
{
  Serial.begin(115200);

  /* The Nano ESP32 uses the ESP32-S3's native USB, so the port only exists
   * once the host opens it. Bound the wait: an unbounded `while (!Serial);`
   * hangs forever on battery with no host attached. */
  unsigned long start = millis();
  while (!Serial && (millis() - start) < 3000) {
    delay(10);
  }
  delay(500);

  /* Arduino LABELS A4/A5 -- the core remaps them to raw GPIO 11 and 12.
   * Do not write Wire.begin(21, 22): the ESP32-S3 has no GPIO 22. */
  Wire.begin(A4, A5);
  Wire.setClock(400000);

  sensor.setTimeout(500);

  if (!sensor.init()) {
    Serial.println(F("# ERROR: VL53L0X not responding at 0x29"));
    Serial.println(F("# Check VIN on 3V3, GND, SDA on A4, SCL on A5"));
    while (true) {
      delay(1000);
    }
  }

  setProfile(0);
  sensor.startContinuous();
  sensorReady = true;

  Serial.println(F("# VL53L0X ready"));
  Serial.println(F("# Commands:"));
  Serial.println(F("# z 100 = calibrate, target truly 100 mm away"));
  Serial.println(F("# r     = reset offset"));
  Serial.println(F("# p 0-3 = profile (0 def, 1 fast, 2 long, 3 accurate)"));
  Serial.println(F("# ?     = show settings"));
  printSettings();
  Serial.println(F("# millis,raw_mm,range_mm,status"));

  Serial.println(F("millis,raw_mm,range_mm,status"));
}


/* --- Main loop ---------------------------------------------------------- */

void loop()
{
  checkSerial();

  uint16_t raw = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    Serial.println(F("# ERROR: VL53L0X timeout"));
    return;
  }

  uint8_t status = readRangeStatus();

  /* Apply the offset in signed arithmetic, then clamp at zero.
   *
   * The cast matters: raw is unsigned, so `raw + offsetMm` with a negative
   * offset would wrap to a huge positive number instead of going negative. */
  int32_t corrected = (int32_t)raw + offsetMm;
  if (corrected < 0) corrected = 0;

  /* Two independent gates, and both matter: the chip's own status code (did
   * IT trust the reading?) and our sanity window (is it plausible for us?).
   * The chip will hand you a confident-looking number with a sigma-fail flag
   * on it. */
  bool valid = (status == 0) &&
               (raw < OUT_OF_RANGE) &&
               (corrected >= SANE_MIN_MM) &&
               (corrected <= SANE_MAX_MM);

  Serial.print(millis());
  Serial.print(',');
  Serial.print(raw);
  Serial.print(',');

  if (valid) {
    Serial.print(corrected);
  } else {
    Serial.print(F("nan"));
  }

  Serial.print(',');
  Serial.println(status);
}
