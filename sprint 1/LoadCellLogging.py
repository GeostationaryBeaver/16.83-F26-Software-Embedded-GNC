import serial
#First start the python script, then press reset to run the arduino code
ser = serial.Serial('COM4', 115200) #Change Baud rate and COM port as needed
trial_time = 10000 #time in milliseconds for the trial
file_name = 'load_cell_data1' #omit .csv

finished = False
started = False
start_line = "Starting Load Cell Readings:"
prev_line = []
start_time = 0
with open(file_name + '.csv', 'w') as f: #remember to change the file name for different trials
    while not finished:
        print('e')
        line = ser.readline().decode('utf-8').rstrip()
        if started:
            if prev_line != line:
                print(line)
                time = tuple(line.split(','))[0]
                if not start_time:
                    start_time = int(time) # the first value in the line is the timestamp
                if int(time) - start_time > int(trial_time):
                    finished = True
                    break

            #checking if the line is different from the previous line to avoid duplicates
                prev_line = line
                f.write(line + '\n')

        else:
            print(line)
            if line == start_line:
                started = True
                print("Starting Load Cell Readings")

    f.close()
