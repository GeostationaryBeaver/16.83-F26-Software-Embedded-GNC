#include <HX711.h>

const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;


HX711 scale;

int count = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("HX711 Demo");
  Serial.println("Initializing the scale");
  // scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  // scale.set_scale(); //Must get this number ourselves to calibrate
  // scale.tare();
  Serial.println("Scale initialization complete.");
  Serial.println("Starting Load Cell Readings:");
  //set the output data rate somehow
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.print(millis()); Serial.print(","); Serial.println(count);
  count++;
  delay(1000);
  // serial.println(scale.read(), DEC);
}
