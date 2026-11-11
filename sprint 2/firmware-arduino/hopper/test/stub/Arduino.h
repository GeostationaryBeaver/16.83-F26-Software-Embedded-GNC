#pragma once
#include <stdio.h>
#include <stdint.h>
#include <math.h>
// Nano ESP32 pin names, for drivers/pins.h (values don't matter on the host)
#ifndef A4
enum { A0 = 1, A1 = 2, A4 = 11, A5 = 12, D2 = 5, D3 = 6, D4 = 7, D5 = 8 };
#endif
