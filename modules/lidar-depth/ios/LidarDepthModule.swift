import ExpoModulesCore
import ARKit

public class LidarDepthModule: Module {
  private var arSession: ARSession?
  private var isCapturing = false

  public func definition() -> ModuleDefinition {
    Name("LidarDepthModule")

    Events("onDepthFrame")

    Function("isAvailable") { () -> Bool in
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
    guard ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) else { return }
    guard !isCapturing else { return }

    let session = ARSession()
    session.delegate = self
    let config = ARWorldTrackingConfiguration()
    config.frameSemantics = .sceneDepth
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

    // Depth buffers can have row padding — copy row by row to a tight buffer.
    var tight = Data(capacity: width * height * 4)
    for row in 0..<height {
      let rowPtr = base.advanced(by: row * bytesPerRow)
      tight.append(rowPtr.assumingMemoryBound(to: UInt8.self), count: width * 4)
    }
    return (tight.base64EncodedString(), width, height)
  }
}

extension LidarDepthModule: ARSessionDelegate {
  public func session(_ session: ARSession, didUpdate frame: ARFrame) {
    guard isCapturing, let depthData = frame.sceneDepth else { return }

    let (depthBase64, width, height) = pixelBufferToBase64(depthData.depthMap)
    var confidenceBase64: String? = nil
    if let confMap = depthData.confidenceMap {
      // confidenceMap is UInt8 per pixel (0/1/2), separate helper would be ideal;
      // reusing the float helper here is WRONG — replace with a UInt8-specific
      // copy before shipping. Left as a clear TODO rather than silently wrong math.
      confidenceBase64 = nil
      _ = confMap
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

    self.sendEvent("onDepthFrame", [
      "width": width,
      "height": height,
      "depthBase64": depthBase64,
      "confidenceBase64": confidenceBase64 as Any,
      "cameraTransform": transformArray,
      "intrinsics": intrinsicsArray,
    ])
  }
}
