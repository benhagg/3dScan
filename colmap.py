import os
import sys
import json
import argparse
import shutil
import cv2
import numpy as np
import pycolmap


def get_latest_session(captures_root):
    if not os.path.exists(captures_root):
        return None
    sessions = sorted(
        [os.path.join(captures_root, d) for d in os.listdir(captures_root) if d.startswith("session_")],
        key=os.path.getmtime,
        reverse=True,
    )
    return sessions[0] if sessions else None


def extract_frames(video_path, output_dir, target_fps=3.0, max_frames=200):
    os.makedirs(output_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    native_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    rot_meta = cap.get(cv2.CAP_PROP_ORIENTATION_META)
    step = max(1, int(round(native_fps / target_fps)))

    frame_meta = []
    frame_idx = 0
    saved_count = 0

    print(f"Extracting keyframes from {os.path.basename(video_path)} at ~{target_fps} fps (step={step})...")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret or saved_count >= max_frames:
            break

        if frame_idx % step == 0:
            if rot_meta == 90.0:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            elif rot_meta == 180.0:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            elif rot_meta == 270.0:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

            filename = f"frame_{saved_count:05d}.jpg"
            out_path = os.path.join(output_dir, filename)
            cv2.imwrite(out_path, frame)

            t_sec = frame_idx / native_fps
            frame_meta.append({"filename": filename, "frame_idx": frame_idx, "t_sec": t_sec})
            saved_count += 1

        frame_idx += 1

    cap.release()
    print(f"Extracted {saved_count} frames into {output_dir}.")
    return frame_meta


def run_colmap_sfm(session_dir, target_fps=3.0, max_frames=200):
    video_path = os.path.join(session_dir, "video.mov")
    if not os.path.exists(video_path):
        video_path = os.path.join(session_dir, "video.webm")
    if not os.path.exists(video_path):
        print(f"Error: No video file found in {session_dir}")
        return False

    colmap_dir = os.path.join(session_dir, "colmap")
    images_dir = os.path.join(colmap_dir, "images")
    sparse_dir = os.path.join(colmap_dir, "sparse")
    database_path = os.path.join(colmap_dir, "colmap.db")

    # Clean old database/sparse folders if re-running
    if os.path.exists(database_path):
        try:
            os.remove(database_path)
        except OSError:
            pass
    if os.path.exists(sparse_dir):
        shutil.rmtree(sparse_dir, ignore_errors=True)
    os.makedirs(sparse_dir, exist_ok=True)

    # 1. Extract video frames
    frame_meta = extract_frames(video_path, images_dir, target_fps=target_fps, max_frames=max_frames)
    if not frame_meta:
        print("Error: No frames extracted.")
        return False

    meta_by_name = {m["filename"]: m for m in frame_meta}

    # 2. COLMAP Feature Extraction
    print("\n--- [COLMAP] 1/3 Feature Extraction ---")
    pycolmap.extract_features(
        database_path=database_path,
        image_path=images_dir,
        camera_mode=pycolmap.CameraMode.SINGLE,
    )

    # 3. COLMAP Sequential Feature Matching
    print("\n--- [COLMAP] 2/3 Sequential Matching ---")
    pycolmap.match_sequential(database_path=database_path)

    # 4. COLMAP Incremental Mapping
    print("\n--- [COLMAP] 3/3 Incremental Mapping ---")
    maps = pycolmap.incremental_mapping(
        database_path=database_path,
        image_path=images_dir,
        output_path=sparse_dir,
    )

    if not maps:
        print("\n[COLMAP Warning] Reconstruction failed to create a model.")
        print("Tip: Ensure the video has sufficient texture, lighting, and camera motion.")
        return False

    # Pick the reconstruction with the most registered images
    recon = max(maps.values(), key=lambda m: len(m.images))
    print(f"\n[COLMAP Success] Registered {len(recon.images)} / {len(frame_meta)} frames. Reconstructed {len(recon.points3D)} 3D points.")

    # 5. Extract Camera Trajectory
    trajectory = []
    for img_id, img in recon.images.items():
        name = img.name
        if name not in meta_by_name:
            continue

        world_from_cam = img.cam_from_world().inverse()
        trans = world_from_cam.translation.tolist()
        quat_xyzw = world_from_cam.rotation.quat.tolist()

        trajectory.append({
            "filename": name,
            "t_sec": meta_by_name[name]["t_sec"],
            "translation": trans,
            "rotation_xyzw": quat_xyzw,
        })

    trajectory.sort(key=lambda x: x["t_sec"])

    # 6. Extract 3D Points
    points = []
    for pt in recon.points3D.values():
        points.append({
            "xyz": pt.xyz.tolist(),
            "color": pt.color.tolist(),
        })

    # Save output artifacts
    traj_path = os.path.join(session_dir, "colmap_trajectory.json")
    points_path = os.path.join(session_dir, "colmap_points.json")

    with open(traj_path, "w") as f:
        json.dump(trajectory, f, indent=2)

    with open(points_path, "w") as f:
        json.dump(points, f)

    print(f"\nOutputs saved:")
    print(f" -> Trajectory: {traj_path} ({len(trajectory)} poses)")
    print(f" -> 3D Points:  {points_path} ({len(points)} points)")
    print("\nReady! Run `npm run view` to see the 3D camera trajectory and points in Rerun Viewer.")
    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run offline COLMAP SfM on captured session videos.")
    parser.add_argument("session", nargs="?", help="Path to session folder (captures/session_xxx)")
    parser.add_argument("--all", action="store_true", help="Process all sessions in captures/ directory")
    parser.add_argument("--force", action="store_true", help="Reprocess even if colmap_trajectory.json exists")
    parser.add_argument("--fps", type=float, default=3.0, help="Framerate to sample video keyframes (default: 3.0)")
    parser.add_argument("--max-frames", type=int, default=150, help="Maximum keyframes to reconstruct (default: 150)")
    args = parser.parse_args()

    captures_root = os.path.join(os.path.dirname(__file__), "captures")

    if args.all:
        if not os.path.exists(captures_root):
            print("No captures/ directory found.")
            sys.exit(1)
        sessions = sorted(
            [os.path.join(captures_root, d) for d in os.listdir(captures_root) if d.startswith("session_")]
        )
        print(f"Found {len(sessions)} session(s) in captures/.")
        for s in sessions:
            traj_file = os.path.join(s, "colmap_trajectory.json")
            if os.path.exists(traj_file) and not args.force:
                print(f"\n[Skipping] {os.path.basename(s)} already processed. Use --force to reprocess.")
                continue
            print(f"\n==========================================")
            print(f"Processing: {os.path.basename(s)}")
            print(f"==========================================")
            run_colmap_sfm(s, target_fps=args.fps, max_frames=args.max_frames)
    else:
        session_path = args.session
        if not session_path:
            session_path = get_latest_session(captures_root)

        if not session_path or not os.path.exists(session_path):
            print("Usage: python colmap.py [captures/session_xxx] or python colmap.py --all")
            sys.exit(1)

        print(f"Processing session: {session_path}")
        run_colmap_sfm(session_path, target_fps=args.fps, max_frames=args.max_frames)
