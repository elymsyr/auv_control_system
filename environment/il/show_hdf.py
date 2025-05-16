import h5py
import numpy as np

# 1. Load the HDF5 file
with h5py.File('mpc_data.h5', 'r') as hf:
    print("Datasets in the HDF5 file:", list(hf.keys()))
    x_current = hf['x_current'][:]
    x_desired = hf['x_desired'][:]
    u_opt = hf['u_opt'][:]
    x_opt = hf['x_opt'][:]
    num_rows_x_current = hf['x_current'].shape[0]
    num_rows_x_desired = hf['x_desired'].shape[0]
    num_rows_u_opt = hf['u_opt'].shape[0]
    num_rows_x_opt = hf['x_opt'].shape[0]

# 4. Print the data (showing first 2 rows for brevity)
print("\n=== x_current (First 2 rows) ===")
print(x_current[:2])

print("\n=== x_desired (First 2 rows) ===")
print(x_desired[:2])

print("\n=== u_opt (First 2 rows) ===")
print(u_opt[:2])

print("\n=== x_opt (First 2 rows) ===")
print(x_opt[:2])

print(f"Number of rows in 'x_current': {num_rows_x_current}")
print(f"Number of rows in 'x_desired': {num_rows_x_desired}")
print(f"Number of rows in 'u_opt': {num_rows_u_opt}")
print(f"Number of rows in 'x_opt': {num_rows_x_opt}")
