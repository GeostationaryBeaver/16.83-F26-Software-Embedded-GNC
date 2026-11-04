// Pressure transmitter PT1 (4-20 mA = 0-1000 psi) through a 100 ohm shunt into A0.
// Conversion to psi uses the calibration (see calibration.h).
#pragma once

// Loop current in mA, averaging 4 back-to-back ADC samples (well under 1 ms)
float pressure_read_ma();
