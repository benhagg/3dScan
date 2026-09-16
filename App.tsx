import { useEffect, useRef, useState } from 'react';
import { Button, SafeAreaView, StyleSheet, Text, View } from 'react-native';
import { CameraView, useCameraPermissions } from 'expo-camera';
import { Accelerometer, Gyroscope, DeviceMotion } from 'expo-sensors';
import * as FileSystem from 'expo-file-system';
import * as Sharing from 'expo-sharing';

// Native LiDAR module — stub until you add the native code (see modules/lidar-depth).
// In Expo Go this import will simply be unavailable; app still runs, LiDAR logging just no-ops.
let LidarDepth: any = null;
try {
  LidarDepth = require('./modules/lidar-depth').default;
} catch {
  // native module not built yet — fine for Expo Go testing of video/sensors
}

type SensorSample = { t: number; x: number; y: number; z: number };
type MotionSample = {
  t: number;
  rotationRate?: { alpha: number; beta: number; gamma: number };
  gravity?: { x: number; y: number; z: number };
  attitude?: { pitch: number; roll: number; yaw: number };
};

export default function App() {
  const [cameraPermission, requestCameraPermission] = useCameraPermissions();
  const cameraRef = useRef<CameraView>(null);

  const [recording, setRecording] = useState(false);
  const [sessionDir, setSessionDir] = useState<string | null>(null);
  const [status, setStatus] = useState('idle');

  const accelBuf = useRef<SensorSample[]>([]);
  const gyroBuf = useRef<SensorSample[]>([]);
  const motionBuf = useRef<MotionSample[]>([]);
  const lidarBuf = useRef<any[]>([]);
  const startTimeRef = useRef<number>(0);

  useEffect(() => {
    if (!cameraPermission?.granted) requestCameraPermission();
  }, [cameraPermission]);

  async function startRecording() {
    if (!cameraRef.current) return;

    // Fresh session folder in the app's document directory
    const dir = `${FileSystem.documentDirectory}session_${Date.now()}/`;
    await FileSystem.makeDirectoryAsync(dir, { intermediates: true });
    setSessionDir(dir);

    accelBuf.current = [];
    gyroBuf.current = [];
    motionBuf.current = [];
    lidarBuf.current = [];
    startTimeRef.current = Date.now();

    // High-frequency sensor sampling
    Accelerometer.setUpdateInterval(10); // ~100Hz
    Gyroscope.setUpdateInterval(10);
    DeviceMotion.setUpdateInterval(20); // ~50Hz, includes fused gravity/attitude

    Accelerometer.addListener((d) => {
      accelBuf.current.push({ t: Date.now() - startTimeRef.current, ...d });
    });
    Gyroscope.addListener((d) => {
      gyroBuf.current.push({ t: Date.now() - startTimeRef.current, ...d });
    });
    DeviceMotion.addListener((d) => {
      motionBuf.current.push({
        t: Date.now() - startTimeRef.current,
        rotationRate: d.rotationRate as any,
        // @ts-ignore - accelerationIncludingGravity gives us the gravity vector
        gravity: d.accelerationIncludingGravity,
        attitude: d.rotation as any,
      });
    });

    // Native LiDAR depth stream, if the module is built and available
    if (LidarDepth?.isAvailable?.()) {
      LidarDepth.startDepthCapture((frame: any) => {
        lidarBuf.current.push({ t: Date.now() - startTimeRef.current, ...frame });
      });
    }

    setRecording(true);
    setStatus('recording');
    cameraRef.current.recordAsync({ maxDuration: 300 }).then(async (video) => {
      if (video?.uri) {
        await FileSystem.copyAsync({ from: video.uri, to: `${dir}video.mov` });
      }
    });
  }

  async function stopRecording() {
    if (!cameraRef.current || !sessionDir) return;
    cameraRef.current.stopRecording();

    Accelerometer.removeAllListeners();
    Gyroscope.removeAllListeners();
    DeviceMotion.removeAllListeners();
    if (LidarDepth?.isAvailable?.()) LidarDepth.stopDepthCapture();

    setRecording(false);
    setStatus('writing sensor logs...');

    // Give recordAsync's promise a moment to finish copying the video file
    await new Promise((r) => setTimeout(r, 500));

    await FileSystem.writeAsStringAsync(
      `${sessionDir}accelerometer.json`,
      JSON.stringify(accelBuf.current)
    );
    await FileSystem.writeAsStringAsync(
      `${sessionDir}gyroscope.json`,
      JSON.stringify(gyroBuf.current)
    );
    await FileSystem.writeAsStringAsync(
      `${sessionDir}device_motion.json`,
      JSON.stringify(motionBuf.current)
    );
    await FileSystem.writeAsStringAsync(
      `${sessionDir}lidar_depth.json`,
      JSON.stringify(lidarBuf.current)
    );
    await FileSystem.writeAsStringAsync(
      `${sessionDir}meta.json`,
      JSON.stringify(
        {
          startTime: startTimeRef.current,
          hasLidar: !!LidarDepth?.isAvailable?.(),
          accelSamples: accelBuf.current.length,
          gyroSamples: gyroBuf.current.length,
          motionSamples: motionBuf.current.length,
          lidarSamples: lidarBuf.current.length,
        },
        null,
        2
      )
    );

    setStatus('done — ready to export');
  }

  async function exportSession() {
    if (!sessionDir) return;
    // Share the whole session folder — AirDrop to your Mac is the simplest transfer.
    // (Sharing a directory shares each file; for a single zip, add react-native-zip-archive.)
    const files = await FileSystem.readDirectoryAsync(sessionDir);
    for (const f of files) {
      await Sharing.shareAsync(`${sessionDir}${f}`);
    }
  }

  if (!cameraPermission) return <View />;
  if (!cameraPermission.granted) {
    return (
      <SafeAreaView style={styles.center}>
        <Text>Camera permission required</Text>
        <Button title="Grant permission" onPress={requestCameraPermission} />
      </SafeAreaView>
    );
  }

  return (
    <SafeAreaView style={styles.container}>
      <CameraView ref={cameraRef} style={styles.camera} mode="video" facing="back" />
      <View style={styles.controls}>
        <Text style={styles.status}>{status}</Text>
        {!recording ? (
          <Button title="Start capture" onPress={startRecording} />
        ) : (
          <Button title="Stop capture" onPress={stopRecording} color="#d33" />
        )}
        {sessionDir && !recording && <Button title="Export session" onPress={exportSession} />}
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: '#000' },
  camera: { flex: 1 },
  controls: { padding: 16, gap: 8, backgroundColor: '#111' },
  status: { color: '#fff', marginBottom: 8 },
  center: { flex: 1, justifyContent: 'center', alignItems: 'center' },
});
