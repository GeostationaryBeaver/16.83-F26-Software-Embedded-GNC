import matplotlib.pyplot as plt

filenames = ['load_cell_data1.csv', 'load_cell_data.csv'] # Add your filenames here
for file in filenames:
    with open(file, 'r') as f:
        lines = f.readlines()
        time = [line.split(',')[0] for line in lines]
        load_cell_values = [line.split(',')[1] for line in lines]
        plt.plot(time, load_cell_values, label=file)


plt.xlabel('Time')
plt.ylabel('Load Cell Value')
plt.title('Load Cell Output (grams)')
plt.legend()
plt.show()