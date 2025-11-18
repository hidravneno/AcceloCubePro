//
//  ContentView.swift
//  AcceloCubePro
//

import SwiftUI
import SceneKit
import CoreMotion

struct ContentView: View {
    @StateObject private var vm = MotionVM()
    @State private var showCalibrationAlert = false
    @State private var showControls = true
    
    var body: some View {
        ZStack {
            SceneViewBridge(vm: vm)
                .ignoresSafeArea()
            
            // Botón flotante para mostrar/ocultar controles
            VStack {
                HStack {
                    Spacer()
                    Button(action: {
                        withAnimation(.spring(response: 0.3)) {
                            showControls.toggle()
                        }
                    }) {
                        Image(systemName: showControls ? "chevron.down.circle.fill" : "chevron.up.circle.fill")
                            .font(.system(size: 28))
                            .foregroundColor(.white)
                            .shadow(color: .black.opacity(0.3), radius: 4)
                            .padding()
                    }
                }
                Spacer()
            }
            .zIndex(1)
            
            // Panel de controles
            VStack {
                Spacer()
                
                if showControls {
                    VStack(spacing: 16) {
                        
                        // Header
                        HStack {
                            Text("AcceloCube Pro")
                                .font(.title2)
                                .fontWeight(.bold)
                            Spacer()
                            Circle()
                                .fill(vm.usingDeviceMotion ? Color.green : Color.red)
                                .frame(width: 12, height: 12)
                        }
                        .padding(.bottom, 4)
                        
                        Divider()
                        
                        // Status
                        VStack(alignment: .leading, spacing: 6) {
                            Text("Status: \(vm.status)")
                                .font(.caption)
                                .foregroundColor(.secondary)
                            
                            Text("Latency: \(String(format: "%.1f", vm.sampleLatencyMs)) ms")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                        .frame(maxWidth: .infinity, alignment: .leading)
                        
                        Divider()
                        
                        // Smoothing
                        VStack(alignment: .leading, spacing: 8) {
                            HStack {
                                Text("Smoothing")
                                    .font(.subheadline)
                                Spacer()
                                Text(String(format: "%.2f", vm.cfg.smoothing))
                                    .font(.caption)
                                    .foregroundColor(.secondary)
                            }
                            Slider(value: $vm.cfg.smoothing, in: 0...1)
                                .tint(.blue)
                        }
                        
                        // Damping
                        VStack(alignment: .leading, spacing: 8) {
                            HStack {
                                Text("Damping")
                                    .font(.subheadline)
                                Spacer()
                                Text(String(format: "%.3f", vm.cfg.damping))
                                    .font(.caption)
                                    .foregroundColor(.secondary)
                            }
                            Slider(value: $vm.cfg.damping, in: 0...0.2)
                                .tint(.orange)
                        }
                        
                        // Sample Rate
                        VStack(alignment: .leading, spacing: 8) {
                            Text("Sample Rate")
                                .font(.subheadline)
                            
                            HStack(spacing: 12) {
                                ForEach([30.0, 60.0, 100.0], id: \.self) { hz in
                                    Button(action: {
                                        vm.cfg.sampleHz = hz
                                        vm.applySampleRate()
                                    }) {
                                        Text("\(Int(hz)) Hz")
                                            .font(.caption)
                                            .fontWeight(vm.cfg.sampleHz == hz ? .bold : .regular)
                                            .foregroundColor(vm.cfg.sampleHz == hz ? .white : .primary)
                                            .padding(.horizontal, 16)
                                            .padding(.vertical, 8)
                                            .background(
                                                Capsule()
                                                    .fill(vm.cfg.sampleHz == hz ? Color.blue : Color.gray.opacity(0.2))
                                            )
                                    }
                                }
                            }
                        }
                        
                        Divider()
                        
                        // ACTION BUTTONS
                        HStack(spacing: 12) {
                            Button(action: {
                                vm.toggle()
                            }) {
                                HStack {
                                    Image(systemName: vm.usingDeviceMotion ? "stop.fill" : "play.fill")
                                    Text(vm.usingDeviceMotion ? "Stop" : "Start")
                                }
                                .font(.headline)
                                .foregroundColor(.white)
                                .frame(maxWidth: .infinity)
                                .padding()
                                .background(vm.usingDeviceMotion ? Color.red : Color.green)
                                .cornerRadius(12)
                            }
                            
                            Button(action: {
                                vm.recenter()
                            }) {
                                HStack {
                                    Image(systemName: "arrow.clockwise")
                                    Text("Re-Centre")
                                }
                                .font(.headline)
                                .foregroundColor(.white)
                                .frame(maxWidth: .infinity)
                                .padding()
                                .background(Color.purple)
                                .cornerRadius(12)
                            }
                        }
                        
                        // CALIBRATE BUTTON
                        Button(action: {
                            showCalibrationAlert = true
                        }) {
                            HStack {
                                Image(systemName: "scope")
                                Text("Calibrate Neutral")
                            }
                            .font(.subheadline)
                            .foregroundColor(.white)
                            .frame(maxWidth: .infinity)
                            .padding()
                            .background(Color.orange)
                            .cornerRadius(12)
                        }
                        
                        Toggle("CSV Logging", isOn: $vm.cfg.loggingEnabled)
                            .font(.subheadline)
                    }
                    .padding()
                    .background(
                        RoundedRectangle(cornerRadius: 20)
                            .fill(.ultraThinMaterial)
                    )
                    .padding()
                    .transition(.move(edge: .bottom).combined(with: .opacity))
                }
            }
        }
        
        .alert("Calibración", isPresented: $showCalibrationAlert) {
            Button("Cancelar", role: .cancel) {}
            Button("Calibrar Ahora") {
                let cmQuat = CMQuaternion(
                    x: Double(vm.quat.imag.x),
                    y: Double(vm.quat.imag.y),
                    z: Double(vm.quat.imag.z),
                    w: Double(vm.quat.real)
                )
                vm.calibrateNeutral(currentAttitude: cmQuat)
                vm.recenter()
            }
        } message: {
            Text("Hold the device in the position you want to set as 'neutral' and press Calibrate.")
        }
        
        .onAppear { vm.start() }
        .onDisappear { vm.stop() }
    }
}

#Preview {
    ContentView()
}
