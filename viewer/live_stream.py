import os
import sys
import json
import time
import asyncio
import websockets
import rerun as rr
import rerun.blueprint as rrb
from utils import calc_euler_quat, GyroPredictor, KalmanFilter

SERVER_URL = "ws://127.0.0.1:5050"


def setup_rerun(recording_name, has_video=False):
    """Initializes Rerun blueprint and registers line styles in one unified place."""
    rr.init(recording_name, spawn=True)

    grid_views = []
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


class PacketProcessor:
    """Unified processor for all IMU channels and algorithms."""

    def __init__(self):
        self.prev_gyro_packet_t = None
        self.gyro_pred = GyroPredictor()
        self.kalman_filter = KalmanFilter()

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
    has_video = os.path.exists(video_file)

    print(f"Loading session: {session_name} into Rerun Viewer...")
    setup_rerun(f"3dScan_{session_name}", has_video=has_video)
    processor = PacketProcessor()

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

