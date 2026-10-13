
#include <HX711.h>
#include <stdlib.h>

// HX711 connected to Arduino Mega
const int DT_PIN  = A3;
const int CLK_PIN = A2;

HX711 scale;

// Calibration variables
long offset = 0;
float countsPerNewton = 0.0;

// Settings
const int TARE_SAMPLES = 10;
const int CAL_SAMPLES = 10;
const unsigned long TIMEOUT_MS = 1500;

// Serial command buffer
char command[32];
int commandIndex = 0;


// ----------------------------------------
// Average multiple raw HX711 readings
// ----------------------------------------
bool averageRaw(int samples, long &result) {
  int64_t sum = 0;

  for (int i = 0; i < samples; i++) {
    if (!scale.wait_ready_timeout(TIMEOUT_MS)) {
      return false;
    }

    sum += scale.read();
  }

  result = (long)(sum / samples);
  return true;
}


// ----------------------------------------
// Tare: set the current force to zero
// ----------------------------------------
void tareScale() {
  long newOffset;

  if (averageRaw(TARE_SAMPLES, newOffset)) {
    offset = newOffset;
    Serial.println("# Tare complete");
  } else {
    Serial.println("# ERROR: Tare failed");
  }
}


// ----------------------------------------
// Calibrate using a known mass in grams
// ----------------------------------------
void calibrateScale(float massGrams) {
  if (massGrams <= 0) {
    Serial.println("# ERROR: Enter a positive mass in grams");
    return;
  }

  Serial.println("# Calibrating - keep weight still");

  long average;

  if (!averageRaw(CAL_SAMPLES, average)) {
    Serial.println("# ERROR: Calibration failed");
    return;
  }

  // Reverse sign so applied force is positive
  float netCounts = (float)(offset - average);

  // Convert known mass (grams) to force (N)
  float knownForce = (massGrams / 1000.0) * 9.80665;

  // Calculate conversion factor
  if (netCounts <= 0) {
    Serial.println("# ERROR: Expected positive counts");
    Serial.println("# Check tare, weight, and sensor direction");
    return;
  }

  countsPerNewton = netCounts / knownForce;

  Serial.print("# Calibration complete: ");
  Serial.print(countsPerNewton, 4);
  Serial.println(" counts/N");
}


// ----------------------------------------
// Process commands from Serial Monitor
// ----------------------------------------
void processCommand(char *cmd) {

  if (cmd[0] == 't' || cmd[0] == 'T') {
    tareScale();
  }

  else if (cmd[0] == 'c' || cmd[0] == 'C') {
    // Example: c 500 means calibrate with 500 g
    float mass = atof(cmd + 1);
    calibrateScale(mass);
  }

  else {
    Serial.println("# Unknown command");
    Serial.println("# t = tare");
    Serial.println("# c 500 = calibrate with 500 grams");
  }
}


// ----------------------------------------
// Read commands without blocking sampling
// ----------------------------------------
void checkSerial() {
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


// ----------------------------------------
// Setup
// ----------------------------------------
void setup() {
  Serial.begin(115200);

  // Channel A, gain 128
  scale.begin(DT_PIN, CLK_PIN, 128);

  if (!scale.wait_ready_timeout(2000)) {
    Serial.println("# ERROR: HX711 not responding");
    while (true) {
      delay(1000);
    }
  }

  delay(500);

  Serial.println("# Keep platform unloaded");
  tareScale();

  Serial.println("# Commands:");
  Serial.println("# t = tare");
  Serial.println("# c 500 = calibrate with 500 g");
  Serial.println("# millis,raw,counts,force_N");

  Serial.println("millis,raw,counts,force_N");
}


// ----------------------------------------
// Main loop
// ----------------------------------------
void loop() {

  checkSerial();

  if (!scale.wait_ready_timeout(TIMEOUT_MS)) {
    Serial.println("# ERROR: HX711 timeout");
    return;
  }

  long raw = scale.read_average();

  // Reverse sign and subtract zero offset
  long netCounts = offset - raw;

  // Convert counts to newtons
  float forceN = 0;

  if (countsPerNewton > 0) {
    forceN = (float)netCounts / countsPerNewton;
  }

  // Output measurements
  Serial.print(millis());
  Serial.print(",");
  Serial.print(raw);
  Serial.print(",");
  Serial.print(netCounts);
  Serial.print(",");

  if (countsPerNewton > 0) {
    Serial.println(forceN, 3);
  } else {
    Serial.println("nan");
  }
}
