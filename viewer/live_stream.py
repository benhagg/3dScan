import os
import sys
import json
import time
import asyncio
import websockets
import base64
import cv2
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from utils import calc_euler_quat, GyroPredictor, KalmanFilter, device_attitude_to_cam_quat, QuaternionSmoother

SERVER_URL = "ws://127.0.0.1:5050"


def setup_rerun(recording_name, has_video=False):
    """Initializes Rerun blueprint and registers line styles in one unified place."""
    rr.init(recording_name, spawn=True)

    grid_views = [
        rrb.Spatial3DView(origin="world", name="3D Camera Frustum"),
    ]
    if has_video:
        grid_views.append(rrb.Spatial2DView(origin="camera/video", name="Video Stream"))

    grid_views.extend([
        rrb.TimeSeriesView(origin="imu/attitude", name="Attitude (Pitch/Roll/Yaw)"),
        rrb.TimeSeriesView(origin="imu/drift_noise", name="Drift + Noise"),
        rrb.TimeSeriesView(origin="imu/accel", name="Accelerometer"),
        rrb.TimeSeriesView(origin="imu/gravity", name="Gravity"),
        rrb.TimeSeriesView(origin="imu/gyro", name="Gyro"),
        rrb.TimeSeriesView(origin="imu/mag", name="Magnetometer"),
    ])

    blueprint = rrb.Blueprint(rrb.Grid(*grid_views))
    rr.send_blueprint(blueprint)

    # World coordinate system (Right-Handed Z-Up: +X East, +Y North, +Z Up)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log(
        "world/origin_triad",
        rr.Arrows3D(
            vectors=[[0.3, 0, 0], [0, 0.3, 0], [0, 0, 0.3]],
            colors=[[255, 50, 50], [50, 255, 50], [50, 100, 255]],
            labels=["East (X)", "North (Y)", "Up (Z)"],
        ),
        static=True,
    )

    # Initial camera pinhole setup (frustum visualization in portrait orientation)
    fov_y = np.radians(60)
    w_def, h_def = 1080, 1920
    fl_def = float((h_def / 2.0) / np.tan(fov_y / 2.0))
    rr.log(
        "world/camera",
        rr.Pinhole(
            resolution=[w_def, h_def],
            focal_length=fl_def,
            principal_point=[float(w_def / 2.0), float(h_def / 2.0)],
            camera_xyz=rr.ViewCoordinates.RDF,
            image_plane_distance=0.4,
        ),
        static=True,
    )

    # Pre-register series line colors and names
    rr.log("imu/attitude/pitch", rr.SeriesLines(colors=[255, 75, 75], names="Pitch (Beta)"), static=True)
    # rr.log("imu/attitude/roll", rr.SeriesLines(colors=[75, 255, 75], names="Roll (Gamma)"), static=True)
    # rr.log("imu/attitude/yaw", rr.SeriesLines(colors=[75, 150, 255], names="Yaw (Alpha)"), static=True)
    rr.log("imu/attitude/gyro_pitch", rr.SeriesLines(colors=[255, 200, 50], names="Gyro Pitch"), static=True)
    rr.log("imu/attitude/gyro_roll", rr.SeriesLines(colors=[50, 220, 255], names="Gyro Roll"), static=True)
    rr.log("imu/attitude/gyro_yaw", rr.SeriesLines(colors=[220, 100, 255], names="Gyro Yaw"), static=True)
    rr.log("imu/attitude/grav_pitch", rr.SeriesLines(colors=[255, 100, 100], names="Grav Pitch"), static=True)

    # # Drift + Noise comparison (Quat tilt from gravity vs Gyro integration)
    # rr.log("imu/drift_noise/quat_pitch", rr.SeriesLines(colors=[255, 100, 100], names="Quat Pitch (Gravity)"), static=True)
    # rr.log("imu/drift_noise/quat_roll", rr.SeriesLines(colors=[100, 255, 100], names="Quat Roll (Gravity)"), static=True)
    # rr.log("imu/drift_noise/gyro_pitch", rr.SeriesLines(colors=[255, 180, 50], names="Gyro Pitch (Drift)"), static=True)
    # rr.log("imu/drift_noise/gyro_roll", rr.SeriesLines(colors=[50, 220, 255], names="Gyro Roll (Drift)"), static=True)
    # rr.log("imu/drift_noise/gyro_yaw", rr.SeriesLines(colors=[220, 100, 255], names="Gyro Yaw (Drift)"), static=True)

    # rr.log("imu/accel/x", rr.SeriesLines(colors=[255, 75, 75], names="Accel X"), static=True)
    # rr.log("imu/accel/y", rr.SeriesLines(colors=[75, 255, 75], names="Accel Y"), static=True)
    # rr.log("imu/accel/z", rr.SeriesLines(colors=[75, 150, 255], names="Accel Z"), static=True)

    # rr.log("imu/gravity/x", rr.SeriesLines(colors=[255, 75, 75], names="Gravity X"), static=True)
    # rr.log("imu/gravity/y", rr.SeriesLines(colors=[75, 255, 75], names="Gravity Y"), static=True)
    # rr.log("imu/gravity/z", rr.SeriesLines(colors=[75, 150, 255], names="Gravity Z"), static=True)

    # # Gyroscope series line colors (Red, Green, Blue)
    # rr.log("imu/gyro/x", rr.SeriesLines(colors=[255, 75, 75], names="Gyro X"), static=True)
    # rr.log("imu/gyro/y", rr.SeriesLines(colors=[75, 255, 75], names="Gyro Y"), static=True)
    # rr.log("imu/gyro/z", rr.SeriesLines(colors=[75, 150, 255], names="Gyro Z"), static=True)

    # # Magnetometer series line colors (microteslas)
    # rr.log("imu/mag/x", rr.SeriesLines(colors=[255, 75, 75], names="Mag X (uT)"), static=True)
    # rr.log("imu/mag/y", rr.SeriesLines(colors=[75, 255, 75], names="Mag Y (uT)"), static=True)
    # rr.log("imu/mag/z", rr.SeriesLines(colors=[75, 150, 255], names="Mag Z (uT)"), static=True)


def get_trajectory_pose(trajectory, t_sec):
    if not trajectory:
        return [0.0, 0.0, 0.0], None
    if len(trajectory) == 1 or t_sec <= trajectory[0]["t_sec"]:
        return trajectory[0]["translation"], trajectory[0].get("rotation_xyzw")
    if t_sec >= trajectory[-1]["t_sec"]:
        return trajectory[-1]["translation"], trajectory[-1].get("rotation_xyzw")

    for i in range(len(trajectory) - 1):
        p0 = trajectory[i]
        p1 = trajectory[i + 1]
        if p0["t_sec"] <= t_sec <= p1["t_sec"]:
            dt = p1["t_sec"] - p0["t_sec"]
            alpha = (t_sec - p0["t_sec"]) / dt if dt > 1e-6 else 0.0
            t0 = np.array(p0["translation"], dtype=float)
            t1 = np.array(p1["translation"], dtype=float)
            t = (1.0 - alpha) * t0 + alpha * t1

            q0 = p0.get("rotation_xyzw")
            q1 = p1.get("rotation_xyzw")
            if q0 and q1:
                q = (1.0 - alpha) * np.array(q0, dtype=float) + alpha * np.array(q1, dtype=float)
                norm = np.linalg.norm(q)
                if norm > 1e-6:
                    q = q / norm
                return t.tolist(), q.tolist()
            return t.tolist(), None

    return trajectory[-1]["translation"], trajectory[-1].get("rotation_xyzw")


class PacketProcessor:
    """Unified processor for all IMU channels and algorithms."""

    def __init__(self, trajectory=None):
        self.prev_gyro_packet_t = None
        self.gyro_pred = GyroPredictor()
        self.kalman_filter = KalmanFilter()
        self.quat_smoother = QuaternionSmoother(alpha=0.35)
        self.trajectory = trajectory

    def process_packet(self, packet, t_sec):
        rr.set_time("timeline", duration=t_sec)
        ptype = packet.get("type")

        if ptype == "accel":
            rr.log("imu/accel/x", rr.Scalars(packet.get("x", 0)))
            rr.log("imu/accel/y", rr.Scalars(packet.get("y", 0)))
            rr.log("imu/accel/z", rr.Scalars(packet.get("z", 0)))

        elif ptype == "gyro":
            current_gyro_t = packet.get("timestamp")
            if current_gyro_t is not None and self.prev_gyro_packet_t is not None:
                dt = current_gyro_t - self.prev_gyro_packet_t
                if 0.0 < dt < 0.5:
                    self.kalman_filter.predict(packet, dt)
            self.prev_gyro_packet_t = current_gyro_t

            rr.log("imu/gyro/x", rr.Scalars(packet.get("x", 0)))
            rr.log("imu/gyro/y", rr.Scalars(packet.get("y", 0)))
            rr.log("imu/gyro/z", rr.Scalars(packet.get("z", 0)))

        elif ptype == "mag":
            self.kalman_filter.measure_mag(packet)
            rr.log("imu/mag/x", rr.Scalars(packet.get("x", 0)))
            rr.log("imu/mag/y", rr.Scalars(packet.get("y", 0)))
            rr.log("imu/mag/z", rr.Scalars(packet.get("z", 0)))

        elif ptype == "motion":
            if "gravity" in packet and packet["gravity"]:
                g = packet["gravity"]
                rr.log("imu/gravity/x", rr.Scalars(g.get("x", 0)))
                rr.log("imu/gravity/y", rr.Scalars(g.get("y", 0)))
                rr.log("imu/gravity/z", rr.Scalars(g.get("z", 0)))

                # Quaternion tilt from gravity (noisy during motion, zero long-term drift)
                q_roll, q_pitch = calc_euler_quat(g)
                self.kalman_filter.measure(g)
                rr.log("imu/drift_noise/quat_pitch", rr.Scalars(q_pitch))
                rr.log("imu/attitude/quat_pitch", rr.Scalars(q_pitch))
                rr.log("imu/drift_noise/quat_roll", rr.Scalars(q_roll))
                

            if "attitude" in packet and packet["attitude"]:
                gyro_pitch = self.kalman_filter.pitch
                gyro_roll = self.kalman_filter.roll
                gyro_yaw = self.kalman_filter.yaw
                att = packet["attitude"]
                pitch = float(att.get("pitch", att.get("beta", 0)))
                roll = float(att.get("roll", att.get("gamma", 0)))
                yaw = float(att.get("yaw", att.get("alpha", 0)))

                rr.log("imu/attitude/pitch", rr.Scalars(pitch))
                rr.log("imu/attitude/roll", rr.Scalars(roll))
                rr.log("imu/attitude/yaw", rr.Scalars(yaw))
                rr.log("imu/attitude/gyro_pitch", rr.Scalars(gyro_pitch))
                rr.log("imu/attitude/gyro_roll", rr.Scalars(gyro_roll))
                rr.log("imu/attitude/gyro_yaw", rr.Scalars(gyro_yaw))

                # Camera pose (frustum rotation + 3D translation if trajectory available)
                cam_quat = device_attitude_to_cam_quat(yaw, pitch, roll)
                smooth_quat = self.quat_smoother.update(cam_quat)

                cam_trans = [0.0, 0.0, 0.0]
                rot_quat = smooth_quat
                if self.trajectory:
                    t_pos, t_rot = get_trajectory_pose(self.trajectory, t_sec)
                    cam_trans = t_pos
                    if t_rot:
                        rot_quat = t_rot

                rr.log(
                    "world/camera",
                    rr.Transform3D(
                        translation=cam_trans,
                        rotation=rr.Quaternion(xyzw=rot_quat),
                    ),
                )

                if not self.gyro_pred.initialized:
                    self.gyro_pred.initialize(pitch, roll, yaw)

            if "rotationRate" in packet and packet["rotationRate"]:
                rot = packet["rotationRate"]
                rr.log("imu/gyro/x", rr.Scalars(rot.get("alpha", 0)))
                rr.log("imu/gyro/y", rr.Scalars(rot.get("beta", 0)))
                rr.log("imu/gyro/z", rr.Scalars(rot.get("gamma", 0)))

                # Dead-reckoning gyro prediction (smooth, accumulates bias drift)
                g_pitch, g_roll, g_yaw = self.gyro_pred.predict(rot, t_sec)
                rr.log("imu/drift_noise/gyro_pitch", rr.Scalars(g_pitch))
                rr.log("imu/drift_noise/gyro_roll", rr.Scalars(g_roll))
                rr.log("imu/drift_noise/gyro_yaw", rr.Scalars(g_yaw))


def replay_session(session_path):
    """Replays saved session JSON files through the exact same packet pipeline as live mode."""
    if not os.path.exists(session_path):
        print(f"Error: Session folder '{session_path}' not found.")
        return

    session_name = os.path.basename(os.path.normpath(session_path))
    video_file = os.path.join(session_path, "video.mov")
    if not os.path.exists(video_file):
        video_file = os.path.join(session_path, "video.webm")
    has_video = os.path.exists(video_file)

    print(f"Loading session: {session_name} into Rerun Viewer...")
    setup_rerun(f"3dScan_{session_name}", has_video=has_video)

    # Check for COLMAP reconstructed 3D points
    points_path = os.path.join(session_path, "colmap_points.json")
    if os.path.exists(points_path):
        try:
            with open(points_path, "r") as f:
                points_data = json.load(f)
            if points_data:
                xyz = [p["xyz"] for p in points_data]
                colors = [p["color"] for p in points_data]
                rr.log("world/sparse_points", rr.Points3D(positions=xyz, colors=colors), static=True)
                print(f"Loaded {len(xyz)} 3D points from COLMAP reconstruction.")
        except Exception as e:
            print(f"Warning loading {points_path}: {e}")

    # Check for COLMAP camera trajectory
    trajectory = None
    traj_path = os.path.join(session_path, "colmap_trajectory.json")
    if os.path.exists(traj_path):
        try:
            with open(traj_path, "r") as f:
                trajectory = json.load(f)
            if trajectory:
                path_pts = [p["translation"] for p in trajectory]
                rr.log("world/camera_path", rr.LineStrips3D(strips=[path_pts], colors=[[0, 220, 255]], radii=0.01), static=True)
                print(f"Loaded {len(trajectory)} 6-DoF camera poses from COLMAP trajectory.")
        except Exception as e:
            print(f"Warning loading {traj_path}: {e}")

    processor = PacketProcessor(trajectory=trajectory)

    # Collect packets from individual sensor files
    packets = []
    file_map = [
        ("accelerometer.json", "accel"),
        ("gyroscope.json", "gyro"),
        ("magnometer.json", "mag"),
        ("magnetometer.json", "mag"),
        ("device_motion.json", "motion"),
    ]

    for fname, ptype in file_map:
        fpath = os.path.join(session_path, fname)
        if os.path.exists(fpath):
            try:
                with open(fpath, "r") as f:
                    for item in json.load(f):
                        item["type"] = ptype
                        ts = item.get("timestamp")
                        if ts is None:
                            for sub in ("attitude", "gravity", "rotationRate"):
                                if isinstance(item.get(sub), dict) and "timestamp" in item[sub]:
                                    ts = item[sub]["timestamp"]
                                    break
                        if ts is not None:
                            item["_ts"] = float(ts)
                            packets.append(item)
            except Exception as e:
                print(f"Warning reading {fname}: {e}")

    if packets:
        packets.sort(key=lambda p: p["_ts"])
        start_ts = packets[0]["_ts"]
        for p in packets:
            t_sec = p["_ts"] - start_ts
            processor.process_packet(p, t_sec)
        print(f"Processed {len(packets)} sensor packets.")

    # Replay video if present
    if has_video:
        import cv2
        cap = cv2.VideoCapture(video_file)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rot_meta = cap.get(cv2.CAP_PROP_ORIENTATION_META)
        total_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT) or 1.0
        video_duration = total_frames / fps

        # If video was recorded in portrait on phone, rotate dimensions
        if rot_meta in (90.0, 270.0):
            disp_width, disp_height = height, width
        else:
            disp_width, disp_height = width, height

        # Align video by the end (both camera and sensors stop at the same instant)
        sensor_duration = (packets[-1]["_ts"] - start_ts) if packets else video_duration
        video_offset = max(0.0, sensor_duration - video_duration)
        print(f"Syncing video to IMU: offset = +{video_offset:.3f}s (sensor: {sensor_duration:.2f}s, video: {video_duration:.2f}s, rot: {rot_meta}deg)")

        if disp_width > 0 and disp_height > 0:
            fov_y = np.radians(60)
            focal_length = float((disp_height / 2.0) / np.tan(fov_y / 2.0))
            rr.log(
                "world/camera",
                rr.Pinhole(
                    resolution=[disp_width, disp_height],
                    focal_length=focal_length,
                    principal_point=[float(disp_width / 2.0), float(disp_height / 2.0)],
                    camera_xyz=rr.ViewCoordinates.RDF,
                    image_plane_distance=1.2,
                ),
                static=True,
            )

        frame_idx = 0
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Rotate frame upright based on QuickTime metadata
            if rot_meta == 90.0:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            elif rot_meta == 180.0:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            elif rot_meta == 270.0:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

            t_video = frame_idx / fps
            t_sec = video_offset + t_video
            rr.set_time("timeline", duration=t_sec)

            # Log compressed JPEG to prevent exceeding Rerun's 1.0 GiB memory limit
            _, enc_jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            rr.log("camera/video", rr.EncodedImage(contents=enc_jpg.tobytes(), media_type="image/jpeg"))

            if trajectory:
                t_pos, t_rot = get_trajectory_pose(trajectory, t_video)
                if t_rot:
                    rr.log(
                        "world/camera",
                        rr.Transform3D(
                            translation=t_pos,
                            rotation=rr.Quaternion(xyzw=t_rot),
                        ),
                    )
            frame_idx += 1
        cap.release()
        print(f"Logged {frame_idx} video frames.")

    print(f"Done! Rerun Viewer is displaying {session_name}.")


async def run_listener():
    setup_rerun("3dScan_LiveStream", has_video=False)

    print(f"==================================================")
    print(f" Connecting to Live Stream Server at {SERVER_URL}...")
    print(f" Tap 'Start Realtime Live Stream' on your iPhone")
    print(f"==================================================")

    while True:
        try:
            async with websockets.connect(SERVER_URL) as ws:
                print("\n[Live Stream] Connected to server! Waiting for iPhone sensor packets...")
                start_time = None
                processor = PacketProcessor()

                async for message in ws:
                    if start_time is None:
                        start_time = time.time()
                        print("[Live Stream] Receiving live sensor packets from iPhone!")

                    packet = json.loads(message)
                    t_sec = time.time() - start_time
                    processor.process_packet(packet, t_sec)

        except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError, OSError) as e:
            print(f"[Live Stream] Server connection lost ({e}). Reconnecting in 2s...")
            await asyncio.sleep(2)


if __name__ == "__main__":
    if len(sys.argv) > 1 and os.path.exists(sys.argv[1]):
        replay_session(sys.argv[1])
    else:
        try:
            asyncio.run(run_listener())
        except KeyboardInterrupt:
            print("\nStopping Live Stream...")

