import asyncio
import json
import time
import rerun as rr
import rerun.blueprint as rrb
import websockets
from utils import calc_euler_quat, GyroPredictor

SERVER_URL = "ws://127.0.0.1:5050"

async def run_listener():
    rr.init("3dScan_LiveStream", spawn=True)

    # Set up layout blueprint so all channels and comparisons are organized
    blueprint = rrb.Blueprint(
        rrb.Grid(
            rrb.TimeSeriesView(origin="imu/attitude", name="Attitude (Pitch/Roll/Yaw)"),
            rrb.TimeSeriesView(origin="imu/drift_noise", name="Drift + Noise"),
            rrb.TimeSeriesView(origin="imu/accel", name="Accelerometer"),
            rrb.TimeSeriesView(origin="imu/gravity", name="Gravity"),
            rrb.TimeSeriesView(origin="imu/gyro", name="Gyro"),
        )
    )
    rr.send_blueprint(blueprint)

    # Pre-register series line colors and names so all 3 (pitch, roll, yaw) always appear grouped
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

    rr.log("imu/gravity/x", rr.SeriesLines(colors=[255, 75, 75], names="Gravity X"), static=True)
    rr.log("imu/gravity/y", rr.SeriesLines(colors=[75, 255, 75], names="Gravity Y"), static=True)
    rr.log("imu/gravity/z", rr.SeriesLines(colors=[75, 150, 255], names="Gravity Z"), static=True)

    # Gyroscope series line colors (Red, Green, Blue)
    rr.log("imu/gyro/x", rr.SeriesLines(colors=[255, 75, 75], names="Gyro X"), static=True)
    rr.log("imu/gyro/y", rr.SeriesLines(colors=[75, 255, 75], names="Gyro Y"), static=True)
    rr.log("imu/gyro/z", rr.SeriesLines(colors=[75, 150, 255], names="Gyro Z"), static=True)

    print(f"==================================================")
    print(f" Connecting to Live Stream Server at {SERVER_URL}...")
    print(f" Tap 'Start Realtime Live Stream' on your iPhone")
    print(f"==================================================")

    while True:
        try:
            async with websockets.connect(SERVER_URL) as ws:
                print("\n[Live Stream] Connected to server! Waiting for iPhone sensor packets...")
                start_time = None
                gyro_pred = GyroPredictor()

                async for message in ws:
                    if start_time is None:
                        start_time = time.time()
                        print("[Live Stream] Receiving live sensor packets from iPhone!")

                    packet = json.loads(message)
                    t_sec = time.time() - start_time
                    rr.set_time("timeline", duration=t_sec)

                    ptype = packet.get("type")

                    if ptype == "accel":
                        rr.log("imu/accel/x", rr.Scalars(packet.get("x", 0)))
                        rr.log("imu/accel/y", rr.Scalars(packet.get("y", 0)))
                        rr.log("imu/accel/z", rr.Scalars(packet.get("z", 0)))

                    elif ptype == "gyro":
                        rr.log("imu/gyro/x", rr.Scalars(packet.get("x", 0)))
                        rr.log("imu/gyro/y", rr.Scalars(packet.get("y", 0)))
                        rr.log("imu/gyro/z", rr.Scalars(packet.get("z", 0)))

                    elif ptype == "motion":
                        if "gravity" in packet and packet["gravity"]:
                            g = packet["gravity"]
                            rr.log("imu/gravity/x", rr.Scalars(g.get("x", 0)))
                            rr.log("imu/gravity/y", rr.Scalars(g.get("y", 0)))
                            rr.log("imu/gravity/z", rr.Scalars(g.get("z", 0)))

                            # Quaternion tilt from gravity (noisy during motion, zero long-term drift)
                            q_roll, q_pitch, _ = calc_euler_quat(g)
                            rr.log("imu/drift_noise/quat_pitch", rr.Scalars(q_pitch))
                            rr.log("imu/drift_noise/quat_roll", rr.Scalars(q_roll))

                        if "attitude" in packet and packet["attitude"]:
                            att = packet["attitude"]
                            pitch = float(att.get("pitch", att.get("beta", 0)))
                            roll = float(att.get("roll", att.get("gamma", 0)))
                            yaw = float(att.get("yaw", att.get("alpha", 0)))

                            rr.log("imu/attitude/pitch", rr.Scalars(pitch))
                            rr.log("imu/attitude/roll", rr.Scalars(roll))
                            rr.log("imu/attitude/yaw", rr.Scalars(yaw))
                            rr.log("imu/roll_standalone", rr.Scalars(roll))

                            if not gyro_pred.initialized:
                                gyro_pred.initialize(pitch, roll, yaw)

                        if "rotationRate" in packet and packet["rotationRate"]:
                            rot = packet["rotationRate"]
                            rr.log("imu/gyro/x", rr.Scalars(rot.get("alpha", 0)))
                            rr.log("imu/gyro/y", rr.Scalars(rot.get("beta", 0)))
                            rr.log("imu/gyro/z", rr.Scalars(rot.get("gamma", 0)))

                            # Dead-reckoning gyro prediction (smooth, accumulates bias drift)
                            g_pitch, g_roll, g_yaw = gyro_pred.predict(rot, t_sec)
                            rr.log("imu/drift_noise/gyro_pitch", rr.Scalars(g_pitch))
                            rr.log("imu/drift_noise/gyro_roll", rr.Scalars(g_roll))
                            rr.log("imu/drift_noise/gyro_yaw", rr.Scalars(g_yaw))

        except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError, OSError) as e:
            print(f"[Live Stream] Server connection lost ({e}). Reconnecting in 2s...")
            await asyncio.sleep(2)

if __name__ == "__main__":
    try:
        asyncio.run(run_listener())
    except KeyboardInterrupt:
        print("\nStopping Live Stream...")
