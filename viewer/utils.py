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
    yaw = 0.0
    return roll, pitch, yaw


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


def kalman_filter(gyro, gravity, accel):
    pass

