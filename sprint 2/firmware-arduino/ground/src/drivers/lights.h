// Ground station lights: green on D6 (link), red on D5 (vehicle armed). Active high.
#pragma once

// Sets up the pins with both lights on, as a lamp test.
void lights_init();

void lights_set(bool link, bool armed);
