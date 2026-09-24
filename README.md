# Quickstart Commands

## Live Streaming (Real-Time)
- **Terminal 1 (Server):**
  ```bash
  npm run server
  ```
- **Terminal 2 (Visualizer):**
  ```bash
  npm run live
  ```
- **Terminal 3 (App):**
  ```bash
  npx expo start
  ```
  *(Tap "Start Realtime Live Stream" in the app)*

---

## 3D Reconstruction & Camera Tracking (COLMAP)
- **Process latest session:**
  ```bash
  npm run colmap
  ```
- **Process specific session:**
  ```bash
  python colmap.py captures/session_<timestamp>
  ```
- **Process all pending sessions:**
  ```bash
  python colmap.py --all
  ```
- **Force reprocess all:**
  ```bash
  python colmap.py --all --force
  ```

---

## View Session in Rerun
- **View latest session:**
  ```bash
  npm run view
  ```
- **View specific session:**
  ```bash
  python viewer/visualize.py captures/session_<timestamp>
  ```

---

## Native iOS Build (Mac / LiDAR)
- **Open in Xcode:**
  ```bash
  xed .
  ```
- **Build directly to plugged-in iPhone:**
  ```bash
  npx expo run:ios --device
  ```
