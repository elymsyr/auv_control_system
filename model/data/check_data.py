import h5py
import numpy as np

with h5py.File('data.h5', 'r') as hf:
    # Load datasets
    x_current = hf['x_current'][:]  # Shape: (num_samples, 12)
    x_ref = hf['x_ref'][:]          # Shape: (num_samples, 12*(N+1))
    u_opt = hf['u_opt'][:]          # Shape: (num_samples, 8)
    
    # Reshape x_ref into 3D array: (samples, horizon+1, state_dim)
    x_path = x_ref.reshape(x_ref.shape[0], 12, 41).transpose(0, 2, 1)

    for i, row in enumerate(x_current):
        pos_array = np.array([row[0], row[1], row[2], 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float32)
        x_path[i] -= pos_array
    
    x_current = x_current[:, 3:]

    X = np.hstack((x_current, x_ref)).astype(np.float32)
    y = u_opt.astype(np.float32)

print("X shape: ", X.shape)
print("y shape: ", y.shape)