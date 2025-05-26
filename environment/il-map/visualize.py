import numpy as np
import matplotlib.pyplot as plt

# Configuration
W, H = 129, 129

# Load data as uint8 (1 byte per element)
raw_data0 = np.fromfile('grid_data.bin', dtype=np.uint8).reshape(H, W)
raw_data1 = np.fromfile('grid_data1.bin', dtype=np.uint8).reshape(H, W)

# Plot side by side
fig, axes = plt.subplots(1, 2, figsize=(12, 6))
im0 = axes[0].imshow(raw_data0, cmap='viridis')
axes[0].set_title('Grid Visualization 0 (uint8)')
plt.colorbar(im0, ax=axes[0])

im1 = axes[1].imshow(raw_data1, cmap='viridis')
axes[1].set_title('Grid Visualization 1 (uint8)')
plt.colorbar(im1, ax=axes[1])

plt.tight_layout()
plt.show()
