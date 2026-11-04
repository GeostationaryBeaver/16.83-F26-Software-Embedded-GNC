How to use loadcell3.ino

1. Plug into arduino Mega and upload sketch
    - If you check serial monitor, you should see a set of four numbers coming in, those are: 
    time [ms], raw counts from load cell, zero-calibrated counts from load cell, calibrated load in newtons
    - If the last number says nan, that's probably chill, it's just not calibrated

2. Zeroing:
    - In the serial monitor, if tou send the command t, it will tare the load cell.
    - What it does is takes an average over 10 readings and uses that as the offset to bring the output back to 0

3. Calibrating:
    - To claibrate, get a known weight in grams
    - Make sure to tare the load cell before applying weight
    - Put the weight on the load cell, then in the serial monitor send, c [mass of weight in grams]
    - Now it should have fourth number and that is the force in newtons that the load cells detects
    * Check the serial plotter while applying weight. If for some reason applying the weight results in a decrease in counts, you may need to calibrate in reverse.
    This should be the case anymore now that the load cells in mounted, but who knows. 
    If weight results in a count decrease, the calibration will fuck up because its seeing a negative weight. The way around this is to put the weight on, then tare it so that when you take it off, the counts increase again. Then you can use the same calibration command and it should be good, just witha sign flip
