//
//  MotionKit.swift
//  AcceloCubePro
//
//  Created by francisco eduardo aramburo reyes on 17/11/25.
//
//
//  MotionKit.swift
//  AcceloCubePro
//
//  Created by francisco eduardo aramburo reyes on 17/11/25.
//

import Foundation
import SceneKit
import CoreMotion
import SwiftUI
import simd
import Combine
import QuartzCore


// MARK: - Config & Utilities
struct MotionConfig {
    var sampleHz: Double = 60
    var smoothing: Double = 0.2    // 0..1 (low-pass factor)
    var damping: Double = 0.02     // 0..0.2 per-tick velocity damping
    var maxSpeed: Float = 5.0      // m/s
    var maxRange: Float = 2.0      // meters (position clamp)
    var loggingEnabled: Bool = false
}


@MainActor
final class MotionVM: ObservableObject {
    // Published state for UI/Scene
    @Published var cfg = MotionConfig()
    @Published var quat: simd_quatf = simd_quatf(angle: 0, axis: SIMD3<Float>(0,1,0))
    @Published var pos: SIMD3<Float> = .zero
    @Published var status: String = "Idle"
    @Published var sampleLatencyMs: Double = 0
    @Published var usingDeviceMotion: Bool = false

    // Lifecycle
    private let mgr = CMMotionManager()
    private let queue = OperationQueue()

    // Kinematics
    private var v: SIMD3<Float> = .zero
    private var lastTimestamp: Double?

    // Calibration: neutral orientation alignment
    private var neutralInv: simd_quatf = simd_quatf(angle: 0, axis: SIMD3<Float>(0,1,0))

    // Logging
    private var logger: CSVLogger? = nil

    init() {
        queue.name = "MotionVM.queue"
        queue.qualityOfService = .userInteractive
    }

    func start() {
        guard CMMotionManager().isDeviceMotionAvailable else {
            status = "DeviceMotion unavailable"
            usingDeviceMotion = false
            return
        }
        stop() // ensure clean start
        usingDeviceMotion = true
        mgr.deviceMotionUpdateInterval = 1.0 / max(1.0, cfg.sampleHz)
        lastTimestamp = nil
        v = .zero
        status = "Starting…"
        if cfg.loggingEnabled {
            logger = CSVLogger(filename: "accelocube_log.csv")
            logger?.writeHeaderIfNeeded()
        } else {
            logger = nil
        }

        mgr.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: queue) { [weak self] dm, err in
            guard let self = self else { return }
            if let err = err {
                Task { @MainActor in
                    self.status = "Error: \(err.localizedDescription)"
                }
                return
            }
            guard let dm = dm else { return }

            let now = CACurrentMediaTime()
            let ts = dm.timestamp // seconds since boot (monotonic)
            let dt: Double
            if let last = self.lastTimestamp {
                dt = max(0, ts - last)
            } else {
                dt = 0
            }
            self.lastTimestamp = ts

            // Quaternion from attitude
            let aq = dm.attitude.quaternion
            var q = simd_quatf(ix: Float(aq.x), iy: Float(aq.y), iz: Float(aq.z), r: Float(aq.w))
            // Apply neutral calibration
            q = self.neutralInv * q

            // User acceleration (already gravity-removed) in device frame
            let ua = SIMD3<Float>(Float(dm.userAcceleration.x), Float(dm.userAcceleration.y), Float(dm.userAcceleration.z))
            // Approximate world transform: revert neutral, then apply current q to accel
            let qWorld = self.neutralInv.inverse * q
            let aWorld = qWorld.act(ua)

            // Integrate with damping/clamps
            var vNew = self.v + aWorld * Float(dt)
            if !vNew.allFinite { vNew = .zero }
            let speed = length(vNew)
            if speed > self.cfg.maxSpeed {
                vNew = normalize(vNew) * self.cfg.maxSpeed
            }
            vNew *= max(0, 1.0 - Float(self.cfg.damping))

            var pNew = self.pos + vNew * Float(dt)
            pNew = simd_clamp(pNew, SIMD3<Float>(repeating: -self.cfg.maxRange), SIMD3<Float>(repeating: self.cfg.maxRange))

            // Smooth quaternion (low-pass slerp)
            let alpha = Float(min(max(self.cfg.smoothing, 0.0), 0.98))
            let qSmoothed = simd_slerp(self.quat, q, 1 - alpha)

            Task { @MainActor in
                self.quat = qSmoothed
                self.v = vNew
                self.pos = pNew
                self.sampleLatencyMs = (CACurrentMediaTime() - now) * 1000.0
                self.status = "OK \(Int(self.cfg.sampleHz)) Hz | v=\(String(format: "%.2f", length(vNew))) m/s | pos=\(self.formatVec(pNew)) m"
            }

            if let logger = self.logger, dt > 0 {
                logger.writeRow(timestamp: ts,
                                 q: qSmoothed,
                                 userAccel: ua,
                                 pos: pNew)
            }
        }
    }

    func stop() {
        if mgr.isDeviceMotionActive {
            mgr.stopDeviceMotionUpdates()
        }
        usingDeviceMotion = false
        status = "Stopped"
    }

    func toggle() {
        usingDeviceMotion ? stop() : start()
    }

    func recenter() {
        v = .zero
        pos = .zero
    }

    func calibrateNeutral(currentAttitude: CMQuaternion?) {
        guard let cq = currentAttitude else { return }
        let q = simd_quatf(ix: Float(cq.x), iy: Float(cq.y), iz: Float(cq.z), r: Float(cq.w))
        neutralInv = q.inverse
    }

    func applySampleRate() {
        if usingDeviceMotion { start() }
    }

    private func formatVec(_ v: SIMD3<Float>) -> String {
        String(format: "%.2f, %.2f, %.2f", v.x, v.y, v.z)
    }
}

private extension SIMD3 where Scalar == Float {
    var allFinite: Bool {
        x.isFinite && y.isFinite && z.isFinite
    }
}

// MARK: - CSV Logger (Optional)
final class CSVLogger {
    private let url: URL
    private var wroteHeader = false
    private let fm = FileManager.default

    init?(filename: String) {
        do {
            let docs = try fm.url(for: .documentDirectory, in: .userDomainMask, appropriateFor: nil, create: true)
            url = docs.appendingPathComponent(filename)
        } catch {
            return nil
        }
    }

    func writeHeaderIfNeeded() {
        guard wroteHeader == false else { return }
        let header = "timestamp,qx,qy,qz,qw,ax,ay,az,px,py,pz\n"
        append(text: header)
        wroteHeader = true
    }

    func writeRow(timestamp: Double, q: simd_quatf, userAccel: SIMD3<Float>, pos: SIMD3<Float>) {
        let row = String(format: "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                         timestamp, q.imag.x, q.imag.y, q.imag.z, q.real,
                         userAccel.x, userAccel.y, userAccel.z,
                         pos.x, pos.y, pos.z)
        append(text: row)
    }

    private func append(text: String) {
        guard let data = text.data(using: .utf8) else { return }
        if FileManager.default.fileExists(atPath: url.path) {
            if let handle = try? FileHandle(forWritingTo: url) {
                try? handle.seekToEnd()
                try? handle.write(contentsOf: data)
                try? handle.close()
            }
        } else {
            try? data.write(to: url)
        }
    }
}

// MARK: - SceneKit Bridge
struct SceneViewBridge: UIViewRepresentable {
    @ObservedObject var vm: MotionVM

    func makeUIView(context: Context) -> SCNView {
        let view = SCNView()
        view.scene = makeScene()
        view.allowsCameraControl = true
        view.autoenablesDefaultLighting = false
        view.backgroundColor = .black
        context.coordinator.cubeNode = view.scene?.rootNode.childNode(withName: "cube", recursively: true)
        context.coordinator.cameraNode = view.scene?.rootNode.childNode(withName: "camera", recursively: true)
        return view
    }

    func updateUIView(_ view: SCNView, context: Context) {
        guard let cube = context.coordinator.cubeNode else { return }
        let q = vm.quat
        cube.orientation = SCNQuaternion(q.imag.x, q.imag.y, q.imag.z, q.real)
        cube.position = SCNVector3(vm.pos.x, vm.pos.y, vm.pos.z)
    }

    func makeCoordinator() -> Coordinator {
        Coordinator()
    }

    final class Coordinator {
        var cubeNode: SCNNode?
        var cameraNode: SCNNode?
    }

    private func makeScene() -> SCNScene {
        let scene = SCNScene()

        // Ground grid
        let floor = SCNFloor()
        floor.firstMaterial?.diffuse.contents = UIColor(red: 0.1, green: 0.1, blue: 0.5, alpha: 1)
        floor.firstMaterial?.roughness.contents = 0.8
        let floorNode = SCNNode(geometry: floor)
        scene.rootNode.addChildNode(floorNode)

        // Cube
        let box = SCNBox(width: 0.3, height: 0.3, length: 0.3, chamferRadius: 0.01)
        let mat = SCNMaterial()
        mat.diffuse.contents = UIColor.systemOrange
        box.materials = [mat]
        let cubeNode = SCNNode(geometry: box)
        cubeNode.name = "cube"
        cubeNode.position = SCNVector3(0, 0.1, 0)
        scene.rootNode.addChildNode(cubeNode)

        // Lights
        let ambient = SCNLight()
        ambient.type = .ambient
        ambient.intensity = 200
        let ambientNode = SCNNode()
        ambientNode.light = ambient
        scene.rootNode.addChildNode(ambientNode)

        let directional = SCNLight()
        directional.type = .directional
        directional.intensity = 700
        let dirNode = SCNNode()
        dirNode.light = directional
        dirNode.eulerAngles = SCNVector3(-Float.pi/3, Float.pi/4, 0)
        scene.rootNode.addChildNode(dirNode)

        // Camera
        let camera = SCNCamera()
        camera.zNear = 0.01
        camera.zFar = 100
        camera.wantsHDR = true
        let camNode = SCNNode()
        camNode.name = "camera"
        camNode.camera = camera
        camNode.position = SCNVector3(0, 0.5, 2.0)
        camNode.look(at: SCNVector3(0, 0.1, 0))
        scene.rootNode.addChildNode(camNode)

        return scene
    }
}
