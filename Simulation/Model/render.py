import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R
import h5py

class Render():
    def __init__(self, size: tuple[float,float,float]=(1, 0.6, 0.4)):
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Vehicle properties
        self.vehicle_vertices = self.get_vehicle_vertices(size=size)
        self.faces_indices = [
            [0, 1, 2, 3], [4, 5, 6, 7],  # Top and bottom
            [0, 1, 5, 4], [2, 3, 7, 6],  # Front and back
            [0, 3, 7, 4], [1, 2, 6, 5]   # Left and right
        ]
        
        # State tracking
        self.state_history = []
        
        # Visualization elements
        self.trajectory, = self.ax.plot([], [], [], 'b-', linewidth=1.5, label="Vehicle Path")
        self.path_trajectory, = self.ax.plot([], [], [], 'r--', linewidth=0.7, alpha=0.7, label="Reference Path")
        self.ref_point = self.ax.scatter([], [], [], c='g', marker='o', s=50, label='Target')
        self.vehicle_box = None
        self.velocity_arrow = None
        self.target_direction_arrow = None
        self.current_yaw_arrow = None
        self.desired_yaw_arrow = None
        
        # View settings
        self.ax.set_xlim([-30, 30])
        self.ax.set_ylim([-30, 30])
        self.ax.set_zlim([-30, 30])
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.set_zlabel("Z Position")
        self.ax.set_title("3D Vehicle Visualization")
        self.ax.legend(loc='upper right')
        self.ax.grid(True)
        self.ax.view_init(elev=30, azim=45)

    def render(self, eta, ref, current_ref_path, v_world=None, desired_yaw=None):
        """Update visualization with current state and reference path"""
        # Extract state
        x, y, z = eta[0], eta[1], eta[2]
        roll, pitch, yaw = eta[3], eta[4], eta[5]
        
        # Update view limits
        self.ax.set_xlim([x-20, x+20])
        self.ax.set_ylim([y-20, y+20])
        self.ax.set_zlim([z-20, z+20])
        
        # Update trajectory
        self.state_history.append([x, y, z])
        state_history_np = np.array(self.state_history)
        self.trajectory.set_data(state_history_np[:, 0], state_history_np[:, 1])
        self.trajectory.set_3d_properties(state_history_np[:, 2])
        
        # Update reference path for this step
        if current_ref_path is not None and len(current_ref_path) > 0:
            self.path_trajectory.set_data(current_ref_path[:, 0], current_ref_path[:, 1])
            self.path_trajectory.set_3d_properties(current_ref_path[:, 2])
        
        # Update reference point
        self.ref_point._offsets3d = ([ref[0]], [ref[1]], [ref[2]])
        
        # Update vehicle position and orientation
        transformed_vertices = self.transform_vehicle(self.vehicle_vertices, [x, y, z], [roll, pitch, yaw])
        faces = [[transformed_vertices[:, i] for i in face] for face in self.faces_indices]
        
        # Create or update vehicle box
        if self.vehicle_box is None:
            self.vehicle_box = Poly3DCollection(faces, color='cyan', edgecolor='k', alpha=0.8)
            self.ax.add_collection3d(self.vehicle_box)
        else:
            self.vehicle_box.set_verts(faces)
        
        # Create or update arrows
        self.update_arrows(x, y, z, ref, yaw, v_world, desired_yaw)
        
        # Redraw the plot
        plt.draw()
        plt.pause(0.001)

    def update_arrows(self, x, y, z, ref, yaw, v_world, desired_yaw):
        # Update or create velocity arrow
        if v_world is not None:
            if self.velocity_arrow is None:
                self.velocity_arrow = self.ax.quiver(x, y, z, v_world[0], v_world[1], v_world[2], 
                                                    color='blue', length=5, normalize=True, label='Velocity')
            else:
                self.velocity_arrow.remove()
                self.velocity_arrow = self.ax.quiver(x, y, z, v_world[0], v_world[1], v_world[2], 
                                                    color='blue', length=5, normalize=True)
        
        # Update or create target direction arrow
        target_direction = ref - np.array([x, y, z])
        if self.target_direction_arrow is None:
            self.target_direction_arrow = self.ax.quiver(x, y, z, target_direction[0], target_direction[1], target_direction[2], 
                                                        color='orange', length=5, normalize=True, label='Target Direction')
        else:
            self.target_direction_arrow.remove()
            self.target_direction_arrow = self.ax.quiver(x, y, z, target_direction[0], target_direction[1], target_direction[2], 
                                                        color='orange', length=5, normalize=True)
        
        # Update or create current yaw arrow
        current_yaw_vector = np.array([np.cos(yaw), np.sin(yaw), 0])
        if self.current_yaw_arrow is None:
            self.current_yaw_arrow = self.ax.quiver(x, y, z, current_yaw_vector[0], current_yaw_vector[1], current_yaw_vector[2], 
                                                   color='purple', length=5, normalize=True, label='Current Yaw')
        else:
            self.current_yaw_arrow.remove()
            self.current_yaw_arrow = self.ax.quiver(x, y, z, current_yaw_vector[0], current_yaw_vector[1], current_yaw_vector[2], 
                                                   color='purple', length=5, normalize=True)
        
        # Update or create desired yaw arrow
        if desired_yaw is not None:
            desired_yaw_vector = np.array([np.cos(desired_yaw), np.sin(desired_yaw), 0])
            if self.desired_yaw_arrow is None:
                self.desired_yaw_arrow = self.ax.quiver(x, y, z, desired_yaw_vector[0], desired_yaw_vector[1], desired_yaw_vector[2], 
                                                       color='red', length=5, normalize=True, label='Desired Yaw')
            else:
                self.desired_yaw_arrow.remove()
                self.desired_yaw_arrow = self.ax.quiver(x, y, z, desired_yaw_vector[0], desired_yaw_vector[1], desired_yaw_vector[2], 
                                                       color='red', length=5, normalize=True)

    def reset(self):
        """Reset for a new trial"""
        self.state_history = []
        self.trajectory.set_data([], [])
        self.trajectory.set_3d_properties([])
        self.path_trajectory.set_data([], [])
        self.path_trajectory.set_3d_properties([])
        self.ref_point._offsets3d = ([], [], [])
        
        # Clear arrows
        for arrow in [self.velocity_arrow, self.target_direction_arrow, 
                      self.current_yaw_arrow, self.desired_yaw_arrow]:
            if arrow is not None:
                arrow.remove()
        
        self.velocity_arrow = None
        self.target_direction_arrow = None
        self.current_yaw_arrow = None
        self.desired_yaw_arrow = None
        
        # Clear vehicle
        if self.vehicle_box is not None:
            self.vehicle_box.remove()
            self.vehicle_box = None

    def get_vehicle_vertices(self, size):
        l, w, h = size
        vertices = np.array([
            [-l, -w, -h], [l, -w, -h], [l, w, -h], [-l, w, -h],
            [-l, -w, h], [l, -w, h], [l, w, h], [-l, w, h]
        ])
        return vertices.T

    def transform_vehicle(self, vertices, position, angles):
        r = R.from_euler('xyz', angles, degrees=False).as_matrix()
        rotated_vertices = r @ vertices
        translated_vertices = rotated_vertices + np.array(position).reshape(3,1)
        return translated_vertices

    def close(self):
        plt.close(self.fig)

def load_trial_data(hdf5_path='astar_mpc_data.h5', trial_index=0, steps_per_trial=200):
    """Load data for a specific trial"""
    with h5py.File(hdf5_path, 'r') as f:
        # Calculate start and end indices for this trial
        start_idx = trial_index * steps_per_trial
        end_idx = start_idx + steps_per_trial
        
        # Load state data for this trial
        state_history = f['x_current'][start_idx:end_idx, :]
        
        # Load reference data for this trial
        x_ref = f['x_ref'][start_idx:end_idx, :]
        
        # Extract reference points for this trial
        ref_history = []
        ref_yaw_history = []
        current_ref_paths = []  # List of arrays, each containing reference points for a step
        
        for i in range(x_ref.shape[0]):
            # Extract all reference points for this step
            step_ref_points = []
            for j in range(0, x_ref.shape[1], 12):
                if j + 12 <= x_ref.shape[1]:
                    ref_point = x_ref[i, j:j+12]
                    step_ref_points.append(ref_point[:3])  # x,y,z
            
            # Store reference path for this step
            current_ref_paths.append(np.array(step_ref_points))
            
            # Extract first reference point for current target
            first_ref = x_ref[i, :12]
            ref_history.append(first_ref[:3])  # x,y,z
            ref_yaw_history.append(first_ref[5])  # yaw
    
    return state_history, np.array(ref_history), np.array(ref_yaw_history), current_ref_paths

def get_total_trials(hdf5_path='astar_mpc_data.h5', steps_per_trial=200):
    """Get the total number of trials in the HDF5 file"""
    with h5py.File(hdf5_path, 'r') as f:
        total_steps = f['x_current'].shape[0]
        return total_steps // steps_per_trial

if __name__ == "__main__":
    # Create a single renderer instance
    render = Render()
    
    # Get total number of trials
    total_trials = get_total_trials()
    
    # Visualize each trial separately
    for trial_index in range(total_trials):
        print(f"Visualizing Trial {trial_index+1}/{total_trials}")
        
        # Load data for this trial
        state_history, ref_history, ref_yaw_history, current_ref_paths = load_trial_data(trial_index=trial_index)
        
        # Reset renderer for new trial
        render.reset()
        
        # Process each step in the trial
        for i in range(len(state_history)):
            eta = state_history[i, :6]
            u, v, w = state_history[i, 6:9]
            ref = ref_history[i]
            ref_yaw = ref_yaw_history[i]
            current_ref_path = current_ref_paths[i]
            
            # Convert to world-frame velocity
            roll, pitch, yaw = eta[3:6]
            rotation = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
            v_world = rotation @ np.array([u, v, w])
            
            # Render with current step's reference path
            render.render(eta, ref, current_ref_path, v_world=v_world, desired_yaw=ref_yaw)
    
    render.close()