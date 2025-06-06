import os
import numpy as np
import matplotlib.pyplot as plt
from glob import glob

def visualize_grids():
    # Get all bin files
    bin_files = glob('*.bin')
    # Group files by title
    file_groups = {}
    for f in bin_files:
        # Extract title and subtitle
        parts = os.path.splitext(f)[0].split('_')
        if len(parts) < 2:
            continue

        title = parts[0]
        subtitle = '_'.join(parts[1:]) if len(parts) > 1 else 'no_subtitle'

        if title not in file_groups:
            file_groups[title] = []
        file_groups[title].append((subtitle, f))
    
    # Visualize each group
    for title, files in file_groups.items():
        files.sort(key=lambda x: x[0])  # Sort by subtitle
        num_files = len(files)
        
        plt.figure(figsize=(15, 5))
        plt.suptitle(f'Title: {title}', fontsize=16)
        
        for i, (subtitle, filename) in enumerate(files):
            try:
                # Read and reshape grid
                grid_data = np.fromfile(filename, dtype=np.uint8)
                size = int(np.sqrt(len(grid_data)))
                grid = grid_data.reshape((size, size))
                
                # Create subplot
                plt.subplot(1, num_files, i+1)
                plt.imshow(grid, cmap='viridis', vmin=0, vmax=255)
                plt.colorbar()
                plt.title(f'Subtitle: {subtitle}')
            except Exception as e:
                print(f"Error processing {filename}: {str(e)}")
        
        plt.tight_layout()
        plt.show()

# Run the visualization
visualize_grids()