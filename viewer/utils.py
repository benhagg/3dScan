import numpy as np

def calc_euler_planar(gravity):
    """2D planar projection trigonometry."""
    gx = gravity.get("x", 0.0)
    gy = gravity.get("y", 0.0)
    gz = gravity.get("z", 0.0)
    roll = np.arctan2(gx, -gz)
    pitch = np.arctan2(-gy, (gx**2 + gz**2)**0.5)
    yaw = 0.0
    return roll, pitch, yaw

def calc_euler_quat(gravity):
    """Shortest-arc 3D quaternion rotation from down vector [0, 0, -1] to gravity."""
    g = np.array([gravity.get("x", 0.0), gravity.get("y", 0.0), gravity.get("z", 0.0)], dtype=float)
    norm = np.linalg.norm(g)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0
    g /= norm

    w = 1.0 - g[2]
    x = -g[1]
    y =  g[0]
    z = 0.0
    q_norm = np.sqrt(w*w + x*x + y*y)
    w, x, y = w / q_norm, x / q_norm, y / q_norm

    roll = np.arctan2(2.0 * (w * y + x * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * x - y * z), -1.0, 1.0))
    return roll, pitch


class GyroPredictor:
    """Dead-reckoning attitude predictor by integrating angular velocity over time."""

    def __init__(self, pitch=0.0, roll=0.0, yaw=0.0):
        self.pitch = float(pitch)
        self.roll = float(roll)
        self.yaw = float(yaw)
        self.last_time = None
        self.initialized = False

    def initialize(self, pitch, roll, yaw):
        self.pitch = float(pitch)
        self.roll = float(roll)
        self.yaw = float(yaw)
        self.initialized = True

    def predict(self, rotation_rate, current_time):
        """Integrate angular velocity: pitch += wx*dt, roll += wy*dt, yaw += wz*dt."""
        if self.last_time is None:
            self.last_time = current_time
            return self.pitch, self.roll, self.yaw

        dt = current_time - self.last_time
        self.last_time = current_time

        if 0.0 < dt < 0.5:
            # Handles both DeviceMotion {alpha, beta, gamma} and Gyroscope {x, y, z}
            w_pitch = float(rotation_rate.get("beta", rotation_rate.get("x", 0.0)))
            w_roll = float(rotation_rate.get("gamma", rotation_rate.get("y", 0.0)))
            w_yaw = float(rotation_rate.get("alpha", rotation_rate.get("z", 0.0)))

            self.pitch += w_pitch * dt
            self.roll += w_roll * dt
            self.yaw += w_yaw * dt

        return self.pitch, self.roll, self.yaw

## used for my own learning and visualization. Apple already implements a filter like this on device which is more accurate
class KalmanFilter:
    """3-Axis attitude Kalman filter estimating (pitch, roll, yaw) and gyro biases."""

    def __init__(self):
        # Best state estimates
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

        # Gyro drift biases
        self.bias_pitch = 0.0
        self.bias_roll = 0.0
        self.bias_yaw = 0.0

        # Error covariance matrices (2x2) for each axis: [angle_error, bias_error]
        self.P_pitch = np.eye(2)
        self.P_roll = np.eye(2)
        self.P_yaw = np.eye(2)

        # Process noise covariance: [angle_noise_var, bias_drift_var]
        self.Q = np.diag([0.001, 0.003])

        # Measurement noise variances
        self.R_gravity = 0.03  # Accelerometer tilt noise
        self.R_mag = 0.10      # Magnetometer heading noise

    def predict(self, gyro, dt):
        """High-speed prediction step using gyro rates (rad/s) and time delta dt."""
        # 1. Deduct biases from gyro rates (iPhone coords: x=pitch, y=roll, z=yaw)
        rate_pitch = float(gyro.get("x", 0.0) or 0.0) - self.bias_pitch
        rate_roll  = float(gyro.get("y", 0.0) or 0.0) - self.bias_roll
        rate_yaw   = float(gyro.get("z", 0.0) or 0.0) - self.bias_yaw

        # 2. Dead-reckoning angles forward
        self.pitch += rate_pitch * dt
        self.roll  += rate_roll * dt
        self.yaw   += rate_yaw * dt

        # 3. Propagate covariance P forward for each axis: P = F @ P @ F.T + Q * dt
        F = np.array([[1.0, -dt],
                      [0.0,  1.0]])
        Q_dt = self.Q * dt
        self.P_pitch = F @ self.P_pitch @ F.T + Q_dt
        self.P_roll  = F @ self.P_roll  @ F.T + Q_dt
        self.P_yaw   = F @ self.P_yaw   @ F.T + Q_dt

    def measure_gravity(self, gravity):
        """Corrects pitch and roll using the 1G gravity vector."""
        grav_roll, grav_pitch = calc_euler_quat(gravity)

        # --- Pitch Update ---
        err_p = grav_pitch - self.pitch
        S_p = self.P_pitch[0, 0] + self.R_gravity
        K_p = np.array([self.P_pitch[0, 0] / S_p, self.P_pitch[1, 0] / S_p])
        self.pitch += K_p[0] * err_p
        self.bias_pitch += K_p[1] * err_p
        self.P_pitch = (np.eye(2) - np.array([[K_p[0], 0.0], [K_p[1], 0.0]])) @ self.P_pitch

        # --- Roll Update ---
        err_r = grav_roll - self.roll
        S_r = self.P_roll[0, 0] + self.R_gravity
        K_r = np.array([self.P_roll[0, 0] / S_r, self.P_roll[1, 0] / S_r])
        self.roll += K_r[0] * err_r
        self.bias_roll += K_r[1] * err_r
        self.P_roll = (np.eye(2) - np.array([[K_r[0], 0.0], [K_r[1], 0.0]])) @ self.P_roll

    def measure(self, gravity):
        """Convenience alias for measure_gravity."""
        self.measure_gravity(gravity)

    def measure_mag(self, mag):
        """Corrects yaw using horizontal magnetometer components."""
        mx = float(mag.get("x", 0.0) or 0.0)
        my = float(mag.get("y", 0.0) or 0.0)
        if abs(mx) < 1e-4 and abs(my) < 1e-4:
            return

        # Measured heading in radians (counter-clockwise from north)
        mag_yaw = -np.arctan2(my, mx)

        # Innovation with angle wrap [-pi, pi]
        err_y = (mag_yaw - self.yaw + np.pi) % (2 * np.pi) - np.pi
        S_y = self.P_yaw[0, 0] + self.R_mag
        K_y = np.array([self.P_yaw[0, 0] / S_y, self.P_yaw[1, 0] / S_y])

        self.yaw += K_y[0] * err_y
        self.bias_yaw += K_y[1] * err_y
        self.P_yaw = (np.eye(2) - np.array([[K_y[0], 0.0], [K_y[1], 0.0]])) @ self.P_yaw
        


