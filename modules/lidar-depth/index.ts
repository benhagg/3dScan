import { requireNativeModule, EventEmitter } from 'expo-modules-core';

const NativeModule = requireNativeModule('LidarDepthModule');
const emitter = new EventEmitter(NativeModule as any);

export type DepthFrame = {
  timestamp: number; // ARFrame nanosecond timestamp (seconds as float)
  width: number;
  height: number;
  // Base64-encoded Float32 depth buffer (meters), width*height*4 bytes (empty if non-LiDAR).
  depthBase64: string;
  // Base64-encoded UInt8 confidence buffer (0=low, 1=med, 2=high), width*height bytes.
  confidenceBase64?: string;
  cameraTransform: number[]; // 4x4 row-major, ARKit camera-to-world
  intrinsics: number[]; // 3x3 row-major
  trackingState: string; // 'normal' | 'limited_...' | 'notAvailable'
  hasDepth: boolean;
};

export default {
  isAvailable(): boolean {
    try {
      return NativeModule.isAvailable();
    } catch {
      return false;
    }
  },
  hasLidar(): boolean {
    try {
      return NativeModule.hasLidar();
    } catch {
      return false;
    }
  },
  startDepthCapture(onFrame: (frame: DepthFrame) => void) {
    (emitter as any).addListener('onDepthFrame', onFrame);
    NativeModule.startDepthCapture();
  },
  stopDepthCapture() {
    NativeModule.stopDepthCapture();
    (emitter as any).removeAllListeners('onDepthFrame');
  },
};
