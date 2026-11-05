// E-stop switch on D2, normally closed to GND with the internal pull-up. Pressing it, or a
// cut wire, opens the switch and reads as STOP.
#pragma once

void estop_init();

// True while the switch is open (stop)
bool estop_open();
