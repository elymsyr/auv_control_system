import csv
import numpy as np
import h5py

# Initialize lists to hold data
x_current_data = []
x_desired_data = []
u_opt_data = []
x_opt_data = []

# Read and parse the CSV file
with open('mpc_data.csv', 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar="'")
    header = next(reader)  # Skip the header row
    
    for row in reader:
        if len(row) != 4:
            print(f"Skipping invalid row: {row}")
            continue
        
        # Parse each field into a list of floats
        try:
            x_current = list(map(float, row[0].split(',')))
            x_desired = list(map(float, row[1].split(',')))
            u_opt = list(map(float, row[2].split(',')))
            x_opt = list(map(float, row[3].split(',')))
        except ValueError as e:
            print(f"Error processing row: {e}")
            continue
        
        # Append to respective lists
        x_current_data.append(x_current)
        x_desired_data.append(x_desired)
        u_opt_data.append(u_opt)
        x_opt_data.append(x_opt)

# Convert lists to NumPy arrays
x_current_arr = np.array(x_current_data)
x_desired_arr = np.array(x_desired_data)
u_opt_arr = np.array(u_opt_data)
x_opt_arr = np.array(x_opt_data)

# Write to HDF5 file
with h5py.File('mpc_data.h5', 'w') as hf:
    hf.create_dataset('x_current', data=x_current_arr)
    hf.create_dataset('x_desired', data=x_desired_arr)
    hf.create_dataset('u_opt', data=u_opt_arr)
    hf.create_dataset('x_opt', data=x_opt_arr)

print("HDF5 file created successfully.")