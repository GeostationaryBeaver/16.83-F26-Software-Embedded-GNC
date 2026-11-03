This is a quick guide how to run the load cell plotting python file. If you need help running the loadcell3.ino file, check the directory with that name

1. Running TS:

    - Uploade loadcell3.ino to the arduino mega
    - You cna check the serial monitor to see that you're getting values, but after that close it before running the plotter, they can be open at the same time

    in terminal either run:
    python3 Load_cell_plotting.py [csv filename].csv
    or
    pyhton.exe Load_cell_plotting.py [csv filename].csv

    depending on whether you are using WSL or powershell 
    idrk what to tell you if you're on MAC, but I think python3 should work

    only dependencies you might need are matplotlib most likely

    run python3 Load_cell_plotting.py -h 
    for help with the rest of the arguments


