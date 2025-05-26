import numpy as np
import matplotlib.pyplot as plt

def visualize_grid(filename, size=(129,129)):
    grid = np.fromfile(filename, dtype=np.uint8).reshape(size)
    plt.imshow(grid, cmap='viridis', vmin=0, vmax=255)
    plt.colorbar()
    plt.title(filename)
    plt.show()

# Compare two grid states
def compare_grids(file_before, file_after):
    grid1 = np.fromfile(file_before, dtype=np.uint8)
    grid2 = np.fromfile(file_after, dtype=np.uint8)
    
    print(f"Max difference: {np.max(np.abs(grid1 - grid2))}")
    print(f"Changed pixels: {np.sum(grid1 != grid2)}")
    
    plt.subplot(121)
    plt.imshow(grid1.reshape(129,129), cmap='viridis')
    plt.title('Before')
    
    plt.subplot(122)
    plt.imshow(grid2.reshape(129,129), cmap='viridis')
    plt.title('After')
    
    plt.show()

# Usage
visualize_grid("test_initial.bin")
compare_grids("test_initial.bin", "test_shifted.bin")