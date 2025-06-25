import os
import numpy as np
import matplotlib.pyplot as plt
from glob import glob

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
        fig, axes = plt.subplots(1, 4, figsize=(16, 5))
        fig.suptitle(f'Base: {base}', fontsize=16)

        # --- Grid File ---
        grid_file = f'grid_{base}.bin'
        if os.path.exists(grid_file):
            grid_data = np.fromfile(grid_file, dtype=np.uint8)
            size = int(np.sqrt(len(grid_data)))
            if size * size == len(grid_data):
                im0 = axes[0].imshow(grid_data.reshape((size, size)), cmap='viridis', vmin=0, vmax=255)
                axes[0].set_title('Grid (uint8)')
                fig.colorbar(im0, ax=axes[0])
            else:
                axes[0].set_title('Grid size mismatch')
        else:
            axes[0].set_title('Grid file not found')

        # --- Node Files ---
        for ax, suffix, title in zip(axes[1:], ['f', 'g', 'h'], ['Node F', 'Node G', 'Node H']):
            node_file = f'node_{base}_{suffix}.bin'
            if os.path.exists(node_file):
                node_data = np.fromfile(node_file, dtype=np.float32)
                size = int(np.sqrt(len(node_data)))
                if size * size == len(node_data):
                    im = ax.imshow(node_data.reshape((size, size)), cmap='plasma')
                    ax.set_title(f'{title} (float32)')
                    fig.colorbar(im, ax=ax)
                else:
                    ax.set_title(f'{title} size mismatch')
            else:
                ax.set_title(f'{title} not found')

        plt.tight_layout(rect=[0, 0, 1, 0.93])
        plt.show()

# Run the visualization
visualize_all()
