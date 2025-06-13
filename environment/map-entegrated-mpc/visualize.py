import os
import numpy as np
import matplotlib.pyplot as plt
from glob import glob

def visualize_all():
    base_names = set()
    bin_files = glob('*.bin')

    # Extract base names
    for f in bin_files:
        if f.startswith('grid_') or f.startswith('node_'):
            base = f.split('_', 1)[1].rsplit('.', 1)[0]
            if f.startswith('node_'):
                base = base.split('_')[0]
            base_names.add(base)

    for base in sorted(base_names):
        fig, axes = plt.subplots(1, 2, figsize=(12, 6))
        fig.suptitle(f'Base: {base}', fontsize=16)

        # --- Grid File ---
        grid_file = f'grid_{base}.bin'
        if os.path.exists(grid_file):
            grid_data = np.fromfile(grid_file, dtype=np.uint8)
            size = int(np.sqrt(len(grid_data)))
            if size * size == len(grid_data):
                im0 = axes[0].imshow(grid_data.reshape((size, size)), cmap='viridis', vmin=0, vmax=255)
                axes[0].set_title('Grid Data (uint8)')
                fig.colorbar(im0, ax=axes[0])
            else:
                axes[0].set_title('Grid size mismatch')
        else:
            axes[0].set_title('Grid file not found')

        # --- Node File ---
        node_file_f = f'node_{base}_f.bin'
        if os.path.exists(node_file_f):
            node_data = np.fromfile(node_file_f, dtype=np.float32)
            size = int(np.sqrt(len(node_data)))
            if size * size == len(node_data):
                im1 = axes[1].imshow(node_data.reshape((size, size)), cmap='plasma')
                axes[1].set_title('Node Data (float32)')
                fig.colorbar(im1, ax=axes[1])
            else:
                axes[1].set_title('Node size mismatch')
        else:
            axes[1].set_title('Node file not found')

        plt.tight_layout(rect=[0, 0, 1, 0.95])
        plt.show()

# Run the visualization
visualize_all()
