# pose2twist
calculating linear and angular velocities from given pose and timestamps in kitti format.
req: Python 3.8+(install required packages)

`python pose2twist.py`


# Calculating Twists from Poses and Timestamps

Calculating twists (linear and angular velocities) from poses and timestamps is a common task in robotics, often used in odometry and motion estimation. Twists are typically derived from the change in pose over time, using finite differences.

## Definitions:
- **Pose**: Given as \((x, y, z, q_x, q_y, q_z, q_w)\), where \((x, y, z)\) represents the position and \((q_x, q_y, q_z, q_w)\) represents the orientation as a quaternion.
- **Twist**: Represented as \((v_x, v_y, v_z, \omega_x, \omega_y, \omega_z)\), where \((v_x, v_y, v_z)\) are linear velocities and \((\omega_x, \omega_y, \omega_z)\) are angular velocities.

## Steps to Calculate Twists:

### 1. Time Difference:
Calculate the time difference between consecutive poses.

\[
\Delta t = t_{i+1} - t_i
\]

### 2. Linear Velocity:
Compute the difference in position between consecutive poses.

\[
\Delta x = x_{i+1} - x_i
\]
\[
\Delta y = y_{i+1} - y_i
\]
\[
\Delta z = z_{i+1} - z_i
\]

The linear velocity components are then:

\[
v_x = \frac{\Delta x}{\Delta t}, \quad v_y = \frac{\Delta y}{\Delta t}, \quad v_z = \frac{\Delta z}{\Delta t}
\]

### 3. Angular Velocity:
Compute the relative orientation between consecutive poses using quaternion multiplication.

The relative rotation \(\Delta q\) is calculated by multiplying the inverse of the previous quaternion by the current quaternion.

\[
\Delta q = q_{i}^{-1} \cdot q_{i+1}
\]

Convert the resulting quaternion \(\Delta q = (q_x', q_y', q_z', q_w')\) to a rotation vector (axis-angle representation).

The angle \(\theta\) of rotation is obtained from:

\[
\theta = 2 \cdot \arccos(q_w')
\]

The axis of rotation \((\omega_x, \omega_y, \omega_z)\) is given by the normalized vector part of the quaternion:

\[
(\omega_x', \omega_y', \omega_z') = \frac{(q_x', q_y', q_z')}{\sqrt{1 - q_w'^2}}
\]

The angular velocity components are then:

\[
\omega_x = \frac{\theta \cdot \omega_x'}{\Delta t}, \quad \omega_y = \frac{\theta \cdot \omega_y'}{\Delta t}, \quad \omega_z = \frac{\theta \cdot \omega_z'}{\Delta t}
\]
