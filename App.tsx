import { useEffect, useRef, useState } from 'react';
import { Button, SafeAreaView, StyleSheet, Text, View } from 'react-native';
import { CameraView, useCameraPermissions } from 'expo-camera';
import { Accelerometer, Gyroscope, DeviceMotion } from 'expo-sensors';
import * as FileSystem from 'expo-file-system/legacy';
import Constants from 'expo-constants';

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
  const [liveStreaming, setLiveStreaming] = useState(false);
  const [sessionDir, setSessionDir] = useState<string | null>(null);
  const [status, setStatus] = useState('idle');

  const wsRef = useRef<WebSocket | null>(null);
  const accelBuf = useRef<SensorSample[]>([]);
  const gyroBuf = useRef<SensorSample[]>([]);
  const motionBuf = useRef<MotionSample[]>([]);
  const lidarBuf = useRef<any[]>([]);
  const startTimeRef = useRef<number>(0);

  useEffect(() => {
    if (!cameraPermission?.granted) requestCameraPermission();
  }, [cameraPermission]);

  const recordingPromiseRef = useRef<Promise<any> | null>(null);

  function startLiveStream() {
    try {
      const wsUrl = `ws://172.20.10.3:5050`;
      setStatus(`Connecting to live stream at ${wsUrl}...`);
      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        setLiveStreaming(true);
        setStatus('🟢 Live Streaming sensors to Rerun...');
        Accelerometer.setUpdateInterval(20); // 50Hz
        Gyroscope.setUpdateInterval(20);
        DeviceMotion.setUpdateInterval(20);

        Accelerometer.addListener((d) => {
          if (ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'accel', ...d }));
          }
        });
        Gyroscope.addListener((d) => {
          if (ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'gyro', ...d }));
          }
        });
        DeviceMotion.addListener((d) => {
          if (ws.readyState === WebSocket.OPEN && d.rotation) {
            ws.send(
              JSON.stringify({
                type: 'motion',
                rotationRate: d.rotationRate,
                gravity: d.accelerationIncludingGravity,
                attitude: {
                  pitch: d.rotation.beta,
                  roll: d.rotation.gamma,
                  yaw: d.rotation.alpha,
                  beta: d.rotation.beta,
                  gamma: d.rotation.gamma,
                  alpha: d.rotation.alpha,
                },
              })
            );
          }
        });
      };

      ws.onerror = (e: any) => {
        console.error('Live stream WebSocket error:', e?.message || e);
        setStatus('❌ Live Stream failed to connect. Run `npm run live` on PC.');
        stopLiveStream();
      };

      ws.onclose = () => {
        setLiveStreaming(false);
        setStatus('Live Stream ended.');
      };
    } catch (e: any) {
      console.error(e);
      setStatus(`Error starting stream: ${e?.message}`);
    }
  }

  function stopLiveStream() {
    Accelerometer.removeAllListeners();
    Gyroscope.removeAllListeners();
    DeviceMotion.removeAllListeners();
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    setLiveStreaming(false);
    setStatus('idle');
  }

  async function startRecording() {
    if (!cameraRef.current) return;

    // Fresh session folder in the app's document directory
    const dir = `${FileSystem.documentDirectory}session_${Date.now()}/`;
    await FileSystem.makeDirectoryAsync(dir, { intermediates: true });
    setSessionDir(dir);

    console.log(`\n========================================`);
    console.log(`[RECORDING STARTED] Session: ${dir}`);
    console.log(`========================================`);

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
    recordingPromiseRef.current = cameraRef.current.recordAsync({ maxDuration: 300, mute: true });
  }

  async function stopRecording() {
    if (!cameraRef.current || !sessionDir) return;

    setStatus('saving video & sensor logs...');
    console.log(`\n[RECORDING STOPPED] Stopping camera and processing sensors...`);

    cameraRef.current.stopRecording();

    Accelerometer.removeAllListeners();
    Gyroscope.removeAllListeners();
    DeviceMotion.removeAllListeners();
    if (LidarDepth?.isAvailable?.()) LidarDepth.stopDepthCapture();

    setRecording(false);

    // Wait for the native camera video recording to finalize
    if (recordingPromiseRef.current) {
      try {
        const video = await recordingPromiseRef.current;
        if (video?.uri) {
          console.log(`[Video Captured] Copying ${video.uri} -> ${sessionDir}video.mov`);
          await FileSystem.copyAsync({ from: video.uri, to: `${sessionDir}video.mov` });
          console.log(`[Video Saved] Successfully saved video.mov`);
        }
      } catch (err: any) {
        console.error(`[Video Save Error]`, err?.message || err);
      }
    }

    console.log(`Samples collected: ${accelBuf.current.length} Accel, ${gyroBuf.current.length} Gyro, ${motionBuf.current.length} Motion`);

    // Save individual sensor files directly to disk
    await FileSystem.writeAsStringAsync(
      `${sessionDir}accelerometer.json`,
      JSON.stringify(accelBuf.current, null, 2)
    );
    await FileSystem.writeAsStringAsync(
      `${sessionDir}gyroscope.json`,
      JSON.stringify(gyroBuf.current, null, 2)
    );
    await FileSystem.writeAsStringAsync(
      `${sessionDir}device_motion.json`,
      JSON.stringify(motionBuf.current, null, 2)
    );
    await FileSystem.writeAsStringAsync(
      `${sessionDir}lidar_depth.json`,
      JSON.stringify(lidarBuf.current, null, 2)
    );
    
    // Save metadata
    await FileSystem.writeAsStringAsync(
      `${sessionDir}meta.json`,
      JSON.stringify(
        {
          sessionFolder: sessionDir.split('/').filter(Boolean).pop(),
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

    const folderName = sessionDir.split('/').filter(Boolean).pop() || `session_${Date.now()}`;
    console.log(`[Saved to Phone Disk] All sensor logs and video written.`);
    setStatus(`💾 Saved. Syncing to PC...`);

    // Auto-sync files directly to PC over the USB wired connection
    try {
      const pcIp = '172.20.10.3';
      const uploadUrl = `http://${pcIp}:5050/upload`;

      console.log(`[Auto-Sync] Connecting to PC server at: ${uploadUrl}`);
      const files = await FileSystem.readDirectoryAsync(sessionDir);
      for (const file of files) {
        console.log(`[Uploading] ${folderName}/${file}...`);
        await FileSystem.uploadAsync(uploadUrl, `${sessionDir}${file}`, {
          httpMethod: 'POST',
          uploadType: FileSystem.FileSystemUploadType.BINARY_CONTENT,
          headers: {
            'x-session-id': folderName,
            'x-filename': file,
          },
        });
        console.log(`[Uploaded] ${folderName}/${file}`);
      }
      console.log(`[SYNC COMPLETE] All session files synced to PC!\n`);
      setStatus(
        `Synced ${folderName} to PC (${accelBuf.current.length} accel, ${gyroBuf.current.length} gyro, ${motionBuf.current.length} motion). Ready.`
      );
    } catch (err: any) {
      console.error(`[Sync Failed] Could not reach PC server at port 5050:`, err?.message || err);
      setStatus(
        `Saved locally to ${folderName} (${accelBuf.current.length} accel, ${gyroBuf.current.length} gyro). PC sync offline.`
      );
    }
  }

  if (!cameraPermission) {
    return (
      <SafeAreaView style={[styles.container, styles.center]}>
        <Text style={styles.permissionText}>Loading camera permissions...</Text>
      </SafeAreaView>
    );
  }

  if (!cameraPermission.granted) {
    return (
      <SafeAreaView style={[styles.container, styles.center]}>
        <Text style={styles.permissionText}>Camera permission is required to record video.</Text>
        <View style={{ marginTop: 20 }}>
          <Button title="Grant Camera Permission" onPress={requestCameraPermission} color="#007AFF" />
        </View>
      </SafeAreaView>
    );
  }

  return (
    <SafeAreaView style={styles.container}>
      <CameraView ref={cameraRef} style={styles.camera} mode="video" facing="back" mute={true} />
      <View style={styles.controls}>
        <Text style={styles.status}>{status}</Text>
        
        {/* Session Recording Button */}
        {!recording ? (
          <Button
            title="Start Capture & Save"
            onPress={startRecording}
            color="#007AFF"
            disabled={liveStreaming}
          />
        ) : (
          <Button title="Stop Capture & Save" onPress={stopRecording} color="#FF3B30" />
        )}

        {/* Real-time Live Stream Button */}
        {!liveStreaming ? (
          <Button
            title="Start Realtime Live Stream"
            onPress={startLiveStream}
            color="#34C759"
            disabled={recording}
          />
        ) : (
          <Button
            title="Stop Realtime Live Stream"
            onPress={stopLiveStream}
            color="#FF9500"
          />
        )}
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: '#1c1c1e' },
  camera: { flex: 1, backgroundColor: '#000' },
  controls: { padding: 16, gap: 10, backgroundColor: '#2c2c2e' },
  status: { color: '#ffffff', fontSize: 14, fontWeight: '600', marginBottom: 2, textAlign: 'center' },
  permissionText: { color: '#ffffff', fontSize: 16, textAlign: 'center', marginHorizontal: 24 },
  center: { flex: 1, justifyContent: 'center', alignItems: 'center' },
});
