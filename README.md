# LiDAR / Video / Motion Capture App

Captures, per session: video (`video.mov`), accelerometer, gyroscope, fused
device motion (gravity/attitude), and raw ARKit LiDAR depth + camera pose per
frame — everything you'd feed into a COLMAP/Gaussian-Splat pipeline later.

## Two phases

**Phase 1 — Expo Go (right now, no build needed)**
Video recording + accelerometer/gyro/device-motion logging all work in Expo Go
out of the box. LiDAR will just silently no-op (`isAvailable()` returns
false/undefined) since Expo Go can't load custom native modules. Use this to
validate the recording/export flow immediately.

```bash
cd lidar-capture-app
npm install
npx expo start
```
Scan the QR code with Expo Go on your iPhone.

**Phase 2 — Dev client build (for LiDAR)**
The native Swift module in `modules/lidar-depth/` needs a real build. Requires
a Mac with Xcode, and an Apple ID (free tier is fine for on-device testing).

```bash
npx expo prebuild -p ios
npx expo run:ios --device
```
This compiles a custom version of the app (a "dev client") onto your phone
with the LiDAR module included. After that, `npx expo start` + opening that
custom app (not Expo Go) gets you live reload same as before, now with LiDAR
data flowing.

No Mac? Use `eas build --profile development --platform ios` instead (cloud
build via Expo's EAS service, still needs an Apple Developer account to
install on your phone).

## What's stubbed / needs finishing in the Swift module

- `confidenceMap` copy is a TODO (marked in the code) — it's UInt8 per pixel,
  not Float32 like the depth map, and needs its own copy routine.
- Depth frames arrive at full ARKit frame rate (~60Hz update, sceneDepth
  updates less often) — you may want to throttle/decimate before sending
  over the bridge if you see performance issues.
- Currently sends raw base64 depth over the JS bridge per frame. Fine for
  short clips; for long recordings, switch to writing depth frames straight
  to disk from Swift and just notify JS of the file path instead.

## Session output layout

```
session_<timestamp>/
  video.mov
  accelerometer.json      [{t, x, y, z}, ...]
  gyroscope.json          [{t, x, y, z}, ...]
  device_motion.json      [{t, rotationRate, gravity, attitude}, ...]
  lidar_depth.json         [{t, width, height, depthBase64, cameraTransform, intrinsics}, ...]
  meta.json                sample counts, start time, whether LiDAR was available
```
`t` is milliseconds since recording started, in all files — use it to align
video frames, IMU samples, and depth frames on your computer.

## Getting the session off the phone

Tap "Export session" — opens the iOS share sheet per file. Easiest path:
**AirDrop to your Mac.** If you're not on a Mac, save to Files app and pull
off via USB/Finder, or share to a cloud drive folder.

For less tapping on multi-file sessions, add `react-native-zip-archive` and
zip the session folder before sharing — left out here to keep the native
dependency surface small for a first build.
