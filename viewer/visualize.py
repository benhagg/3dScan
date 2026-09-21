import os
import sys
import json
import cv2
import rerun as rr
import rerun.blueprint as rrb
from utils import calc_euler_quat, GyroPredictor


def visualize_session(session_path):
    if not os.path.exists(session_path):
        print(f"Error: Session folder '{session_path}' not found.")
        return

    session_name = os.path.basename(os.path.normpath(session_path))
    print(f"Loading session: {session_name} into Rerun Viewer...")

    # Initialize Rerun and spawn desktop viewer
    rr.init(f"3dScan_{session_name}", spawn=True)

    # Set up layout blueprint
    blueprint = rrb.Blueprint(
        rrb.Grid(
            rrb.Spatial2DView(origin="camera/video", name="Video Stream"),
            rrb.TimeSeriesView(origin="imu/attitude", name="Attitude (Pitch/Roll/Yaw)"),
            rrb.TimeSeriesView(origin="imu/drift_noise", name="Drift + Noise"),
            rrb.TimeSeriesView(origin="imu/accel", name="Accelerometer"),
            rrb.TimeSeriesView(origin="imu/gyro", name="Gyroscope"),
        )
    )
    rr.send_blueprint(blueprint)

    # Pre-register series line colors and names so pitch, roll, and yaw are grouped and color-coded
    rr.log("imu/attitude/pitch", rr.SeriesLines(colors=[255, 75, 75], names="Pitch (Beta)"), static=True)
    rr.log("imu/attitude/roll", rr.SeriesLines(colors=[75, 255, 75], names="Roll (Gamma)"), static=True)
    rr.log("imu/attitude/yaw", rr.SeriesLines(colors=[75, 150, 255], names="Yaw (Alpha)"), static=True)

    # Drift + Noise comparison (Quat tilt from gravity vs Gyro integration)
    rr.log("imu/drift_noise/quat_pitch", rr.SeriesLines(colors=[255, 100, 100], names="Quat Pitch (Gravity)"), static=True)
    rr.log("imu/drift_noise/quat_roll", rr.SeriesLines(colors=[100, 255, 100], names="Quat Roll (Gravity)"), static=True)
    rr.log("imu/drift_noise/gyro_pitch", rr.SeriesLines(colors=[255, 180, 50], names="Gyro Pitch (Drift)"), static=True)
    rr.log("imu/drift_noise/gyro_roll", rr.SeriesLines(colors=[50, 220, 255], names="Gyro Roll (Drift)"), static=True)
    rr.log("imu/drift_noise/gyro_yaw", rr.SeriesLines(colors=[220, 100, 255], names="Gyro Yaw (Drift)"), static=True)

    rr.log("imu/accel/x", rr.SeriesLines(colors=[255, 75, 75], names="Accel X"), static=True)
    rr.log("imu/accel/y", rr.SeriesLines(colors=[75, 255, 75], names="Accel Y"), static=True)
    rr.log("imu/accel/z", rr.SeriesLines(colors=[75, 150, 255], names="Accel Z"), static=True)

    rr.log("imu/gyro/x", rr.SeriesLines(colors=[255, 75, 75], names="Gyro X"), static=True)
    rr.log("imu/gyro/y", rr.SeriesLines(colors=[75, 255, 75], names="Gyro Y"), static=True)
    rr.log("imu/gyro/z", rr.SeriesLines(colors=[75, 150, 255], names="Gyro Z"), static=True)

    # 1. Load & Log Accelerometer
    accel_file = os.path.join(session_path, "accelerometer.json")
    if os.path.exists(accel_file):
        with open(accel_file, "r") as f:
            for s in json.load(f):
                t_sec = (s.get("t") or 0) / 1000.0
                rr.set_time("timeline", duration=t_sec)
                rr.log("imu/accel/x", rr.Scalars(s.get("x", 0)))
                rr.log("imu/accel/y", rr.Scalars(s.get("y", 0)))
                rr.log("imu/accel/z", rr.Scalars(s.get("z", 0)))

    # 2. Load & Log Gyroscope
    gyro_file = os.path.join(session_path, "gyroscope.json")
    if os.path.exists(gyro_file):
        with open(gyro_file, "r") as f:
            for s in json.load(f):
                t_sec = (s.get("t") or 0) / 1000.0
                rr.set_time("timeline", duration=t_sec)
                rr.log("imu/gyro/x", rr.Scalars(s.get("x", 0)))
                rr.log("imu/gyro/y", rr.Scalars(s.get("y", 0)))
                rr.log("imu/gyro/z", rr.Scalars(s.get("z", 0)))

    # 3. Load & Log Device Motion (Gravity & Attitude & Gyro Integration)
    motion_file = os.path.join(session_path, "device_motion.json")
    if os.path.exists(motion_file):
        gyro_pred = GyroPredictor()

        with open(motion_file, "r") as f:
            for s in json.load(f):
                t_sec = (s.get("t") or 0) / 1000.0
                rr.set_time("timeline", duration=t_sec)

                # Reference attitude from Apple DeviceMotion
                if "attitude" in s and s["attitude"]:
                    pitch = float(s["attitude"].get("beta", 0))
                    roll = float(s["attitude"].get("gamma", 0))
                    yaw = float(s["attitude"].get("alpha", 0))

                    rr.log("imu/attitude/pitch", rr.Scalars(pitch))
                    rr.log("imu/attitude/roll", rr.Scalars(roll))
                    rr.log("imu/attitude/yaw", rr.Scalars(yaw))

                    if not gyro_pred.initialized:
                        gyro_pred.initialize(pitch, roll, yaw)

                # Calculated quaternion pitch/roll from gravity vector
                if "gravity" in s and s["gravity"]:
                    q_roll, q_pitch, _ = calc_euler_quat(s["gravity"])
                    rr.log("imu/drift_noise/quat_pitch", rr.Scalars(q_pitch))
                    rr.log("imu/drift_noise/quat_roll", rr.Scalars(q_roll))

                # Dead-reckoning gyro prediction over time
                if "rotationRate" in s and s["rotationRate"]:
                    rot = s["rotationRate"]
                    g_pitch, g_roll, g_yaw = gyro_pred.predict(rot, t_sec)

                    rr.log("imu/drift_noise/gyro_pitch", rr.Scalars(g_pitch))
                    rr.log("imu/drift_noise/gyro_roll", rr.Scalars(g_roll))
                    rr.log("imu/drift_noise/gyro_yaw", rr.Scalars(g_yaw))

    # 4. Load & Log Video Stream
    video_file = os.path.join(session_path, "video.mov")
    if os.path.exists(video_file):
        cap = cv2.VideoCapture(video_file)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        frame_idx = 0

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            t_sec = frame_idx / fps
            rr.set_time("timeline", duration=t_sec)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rr.log("camera/video", rr.Image(rgb_frame))
            frame_idx += 1

        cap.release()
        print(f"Logged {frame_idx} video frames.")

    print(f"Done! Rerun Viewer is displaying {session_name}.")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        target_dir = sys.argv[1]
    else:
        # Pick the most recent session in captures/
        captures_root = os.path.join(os.path.dirname(__file__), "..", "captures")
        if os.path.exists(captures_root):
            sessions = sorted(
                [os.path.join(captures_root, d) for d in os.listdir(captures_root) if d.startswith("session_")],
                key=os.path.getmtime,
                reverse=True
            )
            target_dir = sessions[0] if sessions else None
        else:
            target_dir = None

    if target_dir:
        visualize_session(target_dir)
    else:
        print("Usage: python viewer/visualize.py captures/session_<timestamp>")
