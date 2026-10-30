#include <Servo.h>

//Open Serial monitor, set baud rate to 9600, and set line ending to Newline
// type in a number between 0 and 180 to move the servo position. This is in degrees. If it has an issue parsing, it will set servo position to 0 degrees.
Servo myServo;
const int SERVO_PIN = 9;

void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
  Serial.println("Enter an angle (0-180):");
}

void loop() {
  if (Serial.available() > 0) {
    float angle = Serial.readStringUntil('\n').toFloat();
    angle = constrain(angle, 0, 180);

    // Convert angle to pulse width (544-2400 µs) so decimals work
    int pulse = 544 + angle * (2400 - 544) / 180.0;
    myServo.writeMicroseconds(pulse);

    Serial.print("Moved to: ");
    Serial.println(angle);
  }
}