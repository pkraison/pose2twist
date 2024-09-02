import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def parse_pose(line):
    values = list(map(float, line.strip().split()))
    position = np.array(values[:3])
    quaternion = np.array(values[3:])
    return position, quaternion

def compute_twist(pos1, pos2, quat1, quat2, dt):
    # Linear velocity
    print(pos1, pos2)
    linear_velocity = (pos2 - pos1) / dt
    
    # Angular velocity
    q1 = R.from_quat(quat1)
    q2 = R.from_quat(quat2)
    dq = q2 * q1.inv()
    
    angle = dq.magnitude()
    if angle > 1e-6:  # to avoid division by zero
        axis = dq.as_rotvec() / angle
        angular_velocity = (angle / dt) * axis
    else:
        angular_velocity = np.array([0.0, 0.0, 0.0])
    
    return linear_velocity, angular_velocity

def process_poses_and_times(pose_file_path, time_file_path):
    with open(pose_file_path, 'r') as pose_file:
        poses = pose_file.readlines()
        
    with open(time_file_path, 'r') as time_file:
        timestamps = list(map(float, time_file.readlines()))
    
    positions = []
    quaternions = []
    linear_velocities = []
    angular_velocities = []
    print(len(timestamps))
    #print(len(poses))
    dt = timestamps[1] - timestamps[0]
    for i in range(len(poses)-1):
        pos, quat = parse_pose(poses[i])
        positions.append(pos)
        quaternions.append(quat)
        
        if i > 0:  # and i < :
        #    
            #print(i, i-1)
            linear_velocity, angular_velocity = compute_twist(positions[i-1], positions[i], quaternions[i-1], quaternions[i], dt)
            linear_velocities.append(linear_velocity)
            angular_velocities.append(angular_velocity)
    
    return np.array(positions), np.array(linear_velocities), np.array(angular_velocities), timestamps

def plot_poses(positions):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], marker='o')
    ax.set_title('3D Plot of Poses')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

def plot_velocities(linear_velocities, angular_velocities, timestamps):
    time = np.array(timestamps[1:]) - timestamps[0]  # Normalized time for plotting
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Linear velocities
    ax1.plot(time, linear_velocities[:, 0], label='v_x')
    ax1.plot(time, linear_velocities[:, 1], label='v_y')
    ax1.plot(time, linear_velocities[:, 2], label='v_z')
    ax1.set_title('Linear Velocities')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Velocity (m/s)')
    ax1.legend()
    
    # Angular velocities
    ax2.plot(time, angular_velocities[:, 0], label='ω_x')
    ax2.plot(time, angular_velocities[:, 1], label='ω_y')
    ax2.plot(time, angular_velocities[:, 2], label='ω_z')
    ax2.set_title('Angular Velocities')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.legend()
    
    plt.tight_layout()
    plt.show()

# Example usage:
pose_file_path = "data/groundtruth_p.txt"
time_file_path = "data/times.txt"

positions, linear_velocities, angular_velocities, timestamps = process_poses_and_times(pose_file_path, time_file_path)

# Plot the results
plot_poses(positions)
plot_velocities(linear_velocities, angular_velocities, timestamps)
