import ExpoModulesCore
import ARKit

public class LidarDepthModule: Module {
  private var arSession: ARSession?
  private var isCapturing = false

  public func definition() -> ModuleDefinition {
    Name("LidarDepthModule")

    Events("onDepthFrame")

    Function("isAvailable") { () -> Bool in
      return ARWorldTrackingConfiguration.isSupported
    }

    Function("hasLidar") { () -> Bool in
      return ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
    }

    Function("startDepthCapture") { () in
      self.startCapture()
    }

    Function("stopDepthCapture") { () in
      self.stopCapture()
    }
  }

  private func startCapture() {
    guard ARWorldTrackingConfiguration.isSupported else { return }
    guard !isCapturing else { return }

    let session = ARSession()
    session.delegate = self
    let config = ARWorldTrackingConfiguration()

    // Enable smoothed depth if supported (cleaner edges and temporal stability),
    // or standard scene depth on LiDAR iPhones.
    if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
      config.frameSemantics.insert(.smoothedSceneDepth)
    } else if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
      config.frameSemantics.insert(.sceneDepth)
    }

    session.run(config)
    self.arSession = session
    self.isCapturing = true
  }

  private func stopCapture() {
    arSession?.pause()
    arSession = nil
    isCapturing = false
  }

  // Converts a CVPixelBuffer of depth (Float32, single channel) to base64.
  private func pixelBufferToBase64(_ pixelBuffer: CVPixelBuffer) -> (String, Int, Int) {
    CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
    defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

    let width = CVPixelBufferGetWidth(pixelBuffer)
    let height = CVPixelBufferGetHeight(pixelBuffer)
    let bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer)
    guard let base = CVPixelBufferGetBaseAddress(pixelBuffer) else { return ("", 0, 0) }

    var tight = Data(capacity: width * height * 4)
    for row in 0..<height {
      let rowPtr = base.advanced(by: row * bytesPerRow)
      tight.append(rowPtr.assumingMemoryBound(to: UInt8.self), count: width * 4)
    }
    return (tight.base64EncodedString(), width, height)
  }

  // Converts a CVPixelBuffer of confidence (UInt8 per pixel: 0=low, 1=med, 2=high) to base64.
  private func confidenceBufferToBase64(_ pixelBuffer: CVPixelBuffer) -> String {
    CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
    defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

    let width = CVPixelBufferGetWidth(pixelBuffer)
    let height = CVPixelBufferGetHeight(pixelBuffer)
    let bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer)
    guard let base = CVPixelBufferGetBaseAddress(pixelBuffer) else { return "" }

    var tight = Data(capacity: width * height)
    for row in 0..<height {
      let rowPtr = base.advanced(by: row * bytesPerRow)
      tight.append(rowPtr.assumingMemoryBound(to: UInt8.self), count: width)
    }
    return tight.base64EncodedString()
  }
}

extension LidarDepthModule: ARSessionDelegate {
  public func session(_ session: ARSession, didUpdate frame: ARFrame) {
    guard isCapturing else { return }

    // Prefer smoothed depth if available, fallback to raw sceneDepth, or nil if non-LiDAR
    let depthData = frame.smoothedSceneDepth ?? frame.sceneDepth
    var depthBase64 = ""
    var width = 0
    var height = 0
    var confidenceBase64: String? = nil

    if let depth = depthData {
      let res = pixelBufferToBase64(depth.depthMap)
      depthBase64 = res.0
      width = res.1
      height = res.2
      if let conf = depth.confidenceMap {
        confidenceBase64 = confidenceBufferToBase64(conf)
      }
    }

    let t = frame.camera.transform // 4x4 simd_float4x4
    let transformArray: [Float] = [
      t.columns.0.x, t.columns.1.x, t.columns.2.x, t.columns.3.x,
      t.columns.0.y, t.columns.1.y, t.columns.2.y, t.columns.3.y,
      t.columns.0.z, t.columns.1.z, t.columns.2.z, t.columns.3.z,
      t.columns.0.w, t.columns.1.w, t.columns.2.w, t.columns.3.w,
    ]

    let k = frame.camera.intrinsics // 3x3 simd_float3x3
    let intrinsicsArray: [Float] = [
      k.columns.0.x, k.columns.1.x, k.columns.2.x,
      k.columns.0.y, k.columns.1.y, k.columns.2.y,
      k.columns.0.z, k.columns.1.z, k.columns.2.z,
    ]

    var trackingStatus = "normal"
    switch frame.camera.trackingState {
    case .notAvailable:
      trackingStatus = "notAvailable"
    case .limited(let reason):
      switch reason {
      case .excessiveMotion: trackingStatus = "limited_excessiveMotion"
      case .insufficientFeatures: trackingStatus = "limited_insufficientFeatures"
      case .initializing: trackingStatus = "limited_initializing"
      case .relocalizing: trackingStatus = "limited_relocalizing"
      @unknown default: trackingStatus = "limited"
      }
    case .normal:
      trackingStatus = "normal"
    }

    self.sendEvent("onDepthFrame", [
      "timestamp": frame.timestamp,
      "width": width,
      "height": height,
      "depthBase64": depthBase64,
      "confidenceBase64": confidenceBase64 as Any,
      "cameraTransform": transformArray,
      "intrinsics": intrinsicsArray,
      "trackingState": trackingStatus,
      "hasDepth": depthData != nil,
    ])
  }
}
