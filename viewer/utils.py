import numpy as np

def calc_euler_planar(gravity):
    """2D planar projection trigonometry (independent planes)."""
    gx = gravity.get("x", 0.0)
    gy = gravity.get("y", 0.0)
    gz = gravity.get("z", 0.0)
    roll = np.arctan2(gx, (gy**2 + gz**2)**0.5)
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

def kalman_filter(gyro, gravity, accel):
    return calc_euler_quat(gravity)