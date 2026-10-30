// MAC Series 35 solenoid valve
// Arduino Mega 2560

const int MAC_VALVE_PIN = 7;

// --------------------------------------------------
// Valve timing constants, all in milliseconds
// --------------------------------------------------

// MAC Series 35 datasheet response times
const float T_EN_MS = 6.0;      // Energization response time
const float T_DE_MS = 2.0;      // De-energization response time

// Fixed amount of time valve stays closed each cycle
const float T_CLOSED_MS = 50;

// Maximum allowed commanded open time.
// Needed because your equation goes to infinity as power -> 1.
const float MAX_OPEN_TIME_MS = 5000.0;


// --------------------------------------------------
// Current operating variables
// --------------------------------------------------

// Current commanded power: 0.0 to 1.0
float powerLevel = 0.0;

// Calculated valve-open duration
unsigned long openTimeMs = 0;

// Valve state
bool valveOpen = false;

// Time at which current state began
unsigned long stateStartTime = 0;


// --------------------------------------------------
// Convert power level into valve-open dwell time
//
// Your equation:
//
//             P(t_en + t_de + t_closed)
//                 - (t_en + t_de)/2
// t_open = --------------------------------
//                       1 - P
//
// --------------------------------------------------
unsigned long powerToOpenTimeMs(float power)
{
  // Power of zero means valve remains closed
  if (power <= 0.0) {
    return 0;
  }

  // At power = 1, denominator becomes zero.
  // Cap dwell time at our chosen maximum.
  if (power >= 1.0) {
    return (unsigned long)MAX_OPEN_TIME_MS;
  }

  // Calculate numerator
  float numerator =
      power * (T_EN_MS + T_DE_MS + T_CLOSED_MS)
      - (T_EN_MS + T_DE_MS) / 2.0;

  // Calculate denominator
  float denominator = 1.0 - power;

  // Calculate open dwell time
  float tOpenMs = numerator / denominator;

  // Your equation produces a negative dwell time
  // for sufficiently small power levels.
  // Physically, we treat that as zero open time.
  if (tOpenMs <= 0.0) {
    return 0;
  }

  // Prevent excessively long open times
  if (tOpenMs > MAX_OPEN_TIME_MS) {
    tOpenMs = MAX_OPEN_TIME_MS;
  }

  return (unsigned long)tOpenMs;
}


// --------------------------------------------------
// Set a new power level
// --------------------------------------------------
void setPower(float newPower)
{
  // Constrain input to range 0 to 1
  newPower = constrain(newPower, 0.0, 1.0);

  powerLevel = newPower;

  // Calculate corresponding open dwell time
  openTimeMs = powerToOpenTimeMs(powerLevel);

  Serial.print("Power set to: ");
  Serial.println(powerLevel, 3);

  Serial.print("Open dwell time: ");
  Serial.print(openTimeMs);
  Serial.println(" ms");

  // If calculated open time is zero, immediately close valve
  if (openTimeMs == 0) {
    digitalWrite(MAC_VALVE_PIN, LOW);
    valveOpen = false;
    stateStartTime = millis();
  }
}


// --------------------------------------------------
// Read power commands from Serial
// --------------------------------------------------
void checkSerial()
{
  if (Serial.available() > 0) {

    // Read a floating-point number
    float newPower = Serial.parseFloat();

    // Remove newline / carriage-return characters
    while (Serial.available() > 0) {
      Serial.read();
    }

    setPower(newPower);
  }
}


// --------------------------------------------------
// Update valve state
// --------------------------------------------------
void updateValve()
{
  unsigned long currentTime = millis();

  // Zero open time means keep valve closed
  if (openTimeMs == 0) {
    digitalWrite(MAC_VALVE_PIN, LOW);
    valveOpen = false;
    return;
  }

  // -----------------------------
  // Valve is currently OPEN
  // -----------------------------
  if (valveOpen) {

    if (currentTime - stateStartTime >= openTimeMs) {

      // Close valve
      digitalWrite(MAC_VALVE_PIN, LOW);

      valveOpen = false;
      stateStartTime = currentTime;
    }
  }

  // -----------------------------
  // Valve is currently CLOSED
  // -----------------------------
  else {

    if (currentTime - stateStartTime >= (unsigned long)T_CLOSED_MS) {

      // Open valve
      digitalWrite(MAC_VALVE_PIN, HIGH);
      Serial.println("opening valve");

      valveOpen = true;
      stateStartTime = currentTime;
    }
  }
}


// --------------------------------------------------
// Arduino setup
// --------------------------------------------------
void setup()
{
  Serial.begin(115200);

  pinMode(MAC_VALVE_PIN, OUTPUT);

  // Valve starts closed
  digitalWrite(MAC_VALVE_PIN, LOW);

  valveOpen = false;
  stateStartTime = millis();

  Serial.println("MAC valve controller ready.");
  Serial.println("Enter power level from 0.0 to 1.0:");
}


// --------------------------------------------------
// Main loop
// --------------------------------------------------
void loop()
{
  checkSerial();
  updateValve();
}