import { requireNativeModule, EventEmitter } from 'expo-modules-core';

const NativeModule = requireNativeModule('LidarDepthModule');
const emitter = new EventEmitter(NativeModule as any);

export type DepthFrame = {
  width: number;
  height: number;
  // Base64-encoded Float32 depth buffer (meters), width*height*4 bytes.
  // Decode on the computer side, not on-device — keep the phone side simple.
  depthBase64: string;
  confidenceBase64?: string;
  cameraTransform: number[]; // 4x4 row-major, ARKit camera-to-world
  intrinsics: number[]; // 3x3 row-major
};

export default {
  isAvailable(): boolean {
    return NativeModule.isAvailable();
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
