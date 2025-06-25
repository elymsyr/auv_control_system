import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R

class Render():
    def __init__(self, size: tuple[float,float,float]=(1, 0.6, 0.4)):
        self.fig = plt.figure(figsize=(7, 5))
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.vehicle_vertices = self.get_vehicle_vertices(size=size)
        self.faces_indices = [
            [0, 1, 2, 3], [4, 5, 6, 7],  # Top and bottom
            [0, 1, 5, 4], [2, 3, 7, 6],  # Front and back
            [0, 3, 7, 4], [1, 2, 6, 5]   # Left and right
        ]
        self.state_history = []
        self.path_trajectory_np = None

        self.trajectory, = self.ax.plot([], [], [], 'b', label="Vehicle Path")
        self.path_trajectory, = self.ax.plot([], [], [], 'r', label="Reference Path")
        self.ax.set_xlim([-30, 30])
        self.ax.set_ylim([-30, 30])
        self.ax.set_zlim([-30, 30])
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.set_zlabel("Z Position")
        self.ax.set_title("3D Vehicle Visualization")

    def render(self, eta, ref, v_world = None, desired_yaw = None):
        """
        Live 3D rendering of the vehicle's state, including reference point, velocity, and trajectory.
        """
        self.clear()

        # Extract position and orientation from state
        x, y, z = eta[0], eta[1], eta[2]
        roll, pitch, yaw = eta[3], eta[4], eta[5]

        self.ax.set_xlim([x-20, x+20])
        self.ax.set_ylim([y-20, y+20])
        self.ax.set_zlim([z-20, z+20])

        # Update trajectory with current state
        self.state_history.append([x, y, z])
        state_history_np = np.array(self.state_history)
        trajectory_x, trajectory_y, trajectory_z = state_history_np[:, 0], state_history_np[:, 1], state_history_np[:, 2]
        
        if self.path_trajectory_np is not None:
            path_trajectory_x, path_trajectory_y, path_trajectory_z = self.path_trajectory_np[:, 0], self.path_trajectory_np[:, 1], self.path_trajectory_np[:, 2]

            self.path_trajectory.set_data(path_trajectory_x, path_trajectory_y)
            self.path_trajectory.set_3d_properties(path_trajectory_z)
        
        self.trajectory.set_data(trajectory_x, trajectory_y)
        self.trajectory.set_3d_properties(trajectory_z)

        # Update vehicle position and orientation
        transformed_vertices = self.transform_vehicle(self.vehicle_vertices, [x, y, z], [roll, pitch, yaw])
        faces = [[transformed_vertices[:, i] for i in face] for face in self.faces_indices]
        vehicle_box = Poly3DCollection(faces, color=['cyan', 'cyan', 'cyan', 'cyan', 'cyan', 'red'], edgecolor='k', alpha=0.6)

        # Draw the vehicle
        self.ax.add_collection3d(vehicle_box)

        # Add reference point
        self.ref_scatter = self.ax.scatter3D([ref[0]], [ref[1]], [ref[2]], color='g', marker='o', s=50, label='Reference Point')

        if v_world is not None:
            # Add velocity arrow (quiver)
            self.ax.quiver(x, y, z, v_world[0], v_world[1], v_world[2], color='blue', length=10, normalize=True, label='Velocity')

        # Add direction-to-target arrow (quiver)
        target_direction = ref - np.array([x, y, z])  # Direction to
        self.ax.quiver(x, y, z, target_direction[0], target_direction[1], target_direction[2], color='orange', length=5, normalize=True, label='Target Direction')

        # Add current yaw arrow (quiver)
        current_yaw_vector = np.array([np.cos(yaw), np.sin(yaw), 0])  # Direction based on current yaw
        self.ax.quiver(x, y, z, current_yaw_vector[0], current_yaw_vector[1], current_yaw_vector[2], color='purple', length=5, normalize=True, label='Current Yaw')

        if desired_yaw is not None:
            # Add desired yaw arrow (quiver)
            desired_yaw_vector = np.array([np.cos(desired_yaw), np.sin(desired_yaw), 0])  # Direction based on desired yaw
            self.ax.quiver(x, y, z, desired_yaw_vector[0], desired_yaw_vector[1], desired_yaw_vector[2], color='red', length=5, normalize=True, label='Desired Yaw')

        # Add legend to explain the elements
        self.ax.legend(loc='upper right')

        # Redraw the plot
        plt.draw()
        plt.pause(0.00000001)

    def reset(self, path_trajectory_np):
        self.clear()
        self.state_history = []
        self.path_trajectory_np = path_trajectory_np

    def clear(self):
        """
        Clears all objects (vehicle, trajectory, etc.) from the plot.
        """
        for coll in self.ax.collections:
            coll.remove()  # Remove all previous 3D collections (vehicle, trajectory, etc.)
        self.ax.cla()  # Clear the axis
        self.ax.set_xlim([-30, 30])
        self.ax.set_ylim([-30, 30])
        self.ax.set_zlim([-30, 30])
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.set_zlabel("Z Position")
        self.ax.set_title("3D Vehicle Visualization")
        self.trajectory, = self.ax.plot([], [], [], 'b', label="Vehicle Path")  # Reset trajectory plot
        self.path_trajectory, = self.ax.plot([], [], [], 'r', label="Reference Path")

    def get_vehicle_vertices(self, size):
        """
        Returns the 8 corner vertices of a rectangular prism centered at (0,0,0).
        """
        l, w, h = size  # Length, width, height
        vertices = np.array([
            [-l, -w, -h], [l, -w, -h], [l, w, -h], [-l, w, -h],  # Bottom face
            [-l, -w, h], [l, -w, h], [l, w, h], [-l, w, h]       # Top face
        ])
        return vertices.T  # Transpose for easy manipulation

    def transform_vehicle(self, vertices, position, angles):
        """
        Applies rotation (roll, pitch, yaw) and translation to the vehicle vertices.
        """
        r = R.from_euler('xyz', angles, degrees=False).as_matrix()  # Rotation matrix
        rotated_vertices = r @ vertices  # Apply rotation
        translated_vertices = rotated_vertices + np.array(position).reshape(3,1)  # Apply translation
        return translated_vertices

    def close(self):
        plt.close(self.fig)
    def transform_vehicle(self, vertices, position, angles):
        """
        Applies rotation (roll, pitch, yaw) and translation to the vehicle vertices.
        """
        r = R.from_euler('xyz', angles, degrees=False).as_matrix()  # Rotation matrix
        rotated_vertices = r @ vertices  # Apply rotation
        translated_vertices = rotated_vertices + np.array(position).reshape(3,1)  # Apply translation
        return translated_vertices

    def close(self):
        plt.close(self.fig)

def load_data():
    # Load state history
    state_data = np.loadtxt('state_history.txt')
    state_history = []
    for row in state_data:
        # x, y, z, roll, pitch, yaw
        state_history.append([row[0], row[1], row[2], row[3], row[4], row[5]])
    
    # Load reference history
    ref_data = np.loadtxt('reference_history.txt')
    ref_history = []
    ref_yaw_history = []
    for row in ref_data:
        # ref_x, ref_y, ref_z
        ref_history.append([row[0], row[1], row[2]])
        ref_yaw_history.append(row[5])

    return np.array(state_history), np.array(ref_history), np.array(ref_yaw_history)

render = Render()
state_history, ref_history, ref_yaw_history = load_data()
for (eta, ref, ref_yaw) in zip(state_history, ref_history, ref_yaw_history):
    render.render(eta, ref, desired_yaw=ref_yaw)
render.close()
