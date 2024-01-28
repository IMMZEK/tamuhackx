//
//  ContentView.swift
//  BLETRANSFERFINAL
//
//  Created by Cesar Magana on 1/28/24.
//

import SwiftUI
import CoreBluetooth
import Foundation
import AVFoundation
import CoreImage
import os
import Metal
import MetalKit


// MARK: Lidar
/*
See LICENSE folder for this sample’s licensing information.

Abstract:
An object that configures and manages the capture pipeline to stream video and LiDAR depth data.
*/

protocol CaptureDataReceiver: AnyObject {
    func onNewData(capturedData: CameraCapturedData)
        func onNewPhotoData(capturedData: CameraCapturedData)
        func onProcessedArrayData(processedArray: [[String]]) // The array of Strings will represent 'low', 'medium', 'high'
}

class CameraController: NSObject, ObservableObject {
    
    var bluetoothViewModel: BluetoothViewModel?
    
    enum ConfigurationError: Error {
        case lidarDeviceUnavailable
        case requiredFormatUnavailable
    }
    
    private let preferredWidthResolution = 1920
    
    private let videoQueue = DispatchQueue(label: "com.example.apple-samplecode.VideoQueue", qos: .userInteractive)
    
    private(set) var captureSession: AVCaptureSession!
    
    private var photoOutput: AVCapturePhotoOutput!
    private var depthDataOutput: AVCaptureDepthDataOutput!
    private var videoDataOutput: AVCaptureVideoDataOutput!
    private var outputVideoSync: AVCaptureDataOutputSynchronizer!
    
    private var textureCache: CVMetalTextureCache!
    
    weak var delegate: CaptureDataReceiver?
    
    var isFilteringEnabled = true {
        didSet {
            depthDataOutput.isFilteringEnabled = isFilteringEnabled
        }
    }
    
    
    override init() {
        super.init()
        
        // Create a texture cache to hold sample buffer textures.
        var textureCache: CVMetalTextureCache?
        CVMetalTextureCacheCreate(kCFAllocatorDefault, nil, MetalEnvironment.shared.metalDevice, nil, &textureCache)
        self.textureCache = textureCache

        do {
            try setupSession()
        } catch {
            fatalError("Unable to configure the capture session.")
        }
    }
    
    private func setupSession() throws {
        captureSession = AVCaptureSession()
        captureSession.sessionPreset = .inputPriority

        // Configure the capture session.
        captureSession.beginConfiguration()
        
        try setupCaptureInput()
        setupCaptureOutputs()
        
        // Finalize the capture session configuration.
        captureSession.commitConfiguration()
    }
    
    private func setupCaptureInput() throws {
        // Look up the LiDAR camera.
        guard let device = AVCaptureDevice.default(.builtInLiDARDepthCamera, for: .video, position: .back) else {
            throw ConfigurationError.lidarDeviceUnavailable
        }
        
        // Find a match that outputs video data in the format the app's custom Metal views require.
        guard let format = (device.formats.last { format in
            format.formatDescription.dimensions.width == preferredWidthResolution &&
            format.formatDescription.mediaSubType.rawValue == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange &&
            !format.isVideoBinned &&
            !format.supportedDepthDataFormats.isEmpty
        }) else {
            throw ConfigurationError.requiredFormatUnavailable
        }
        
        // Find a match that outputs depth data in the format the app's custom Metal views require.
        guard let depthFormat = (format.supportedDepthDataFormats.last { depthFormat in
            depthFormat.formatDescription.mediaSubType.rawValue == kCVPixelFormatType_DepthFloat16
        }) else {
            throw ConfigurationError.requiredFormatUnavailable
        }
        
        // Begin the device configuration.
        try device.lockForConfiguration()

        // Configure the device and depth formats.
        device.activeFormat = format
        device.activeDepthDataFormat = depthFormat

        // Finish the device configuration.
        device.unlockForConfiguration()
        
        print("Selected video format: \(device.activeFormat)")
        print("Selected depth format: \(String(describing: device.activeDepthDataFormat))")
        
        // Add a device input to the capture session.
        let deviceInput = try AVCaptureDeviceInput(device: device)
        captureSession.addInput(deviceInput)
    }
    
    private func setupCaptureOutputs() {
        // Create an object to output video sample buffers.
        videoDataOutput = AVCaptureVideoDataOutput()
        captureSession.addOutput(videoDataOutput)
        
        // Create an object to output depth data.
        depthDataOutput = AVCaptureDepthDataOutput()
        depthDataOutput.isFilteringEnabled = isFilteringEnabled
        captureSession.addOutput(depthDataOutput)

        // Create an object to synchronize the delivery of depth and video data.
        outputVideoSync = AVCaptureDataOutputSynchronizer(dataOutputs: [depthDataOutput, videoDataOutput])
        outputVideoSync.setDelegate(self, queue: videoQueue)

        // Enable camera intrinsics matrix delivery.
        guard let outputConnection = videoDataOutput.connection(with: .video) else { return }
        if outputConnection.isCameraIntrinsicMatrixDeliverySupported {
            outputConnection.isCameraIntrinsicMatrixDeliveryEnabled = true
        }
        
        // Create an object to output photos.
        photoOutput = AVCapturePhotoOutput()
        photoOutput.maxPhotoQualityPrioritization = .quality
        captureSession.addOutput(photoOutput)

        // Enable delivery of depth data after adding the output to the capture session.
        photoOutput.isDepthDataDeliveryEnabled = true
    }

    // MARK: - LiDAR
    func startStream() {
        captureSession.startRunning()
    }
    
    func stopStream() {
        captureSession.stopRunning()
    }
    
    // Code to process LiDAR info:
    // Step 2: Add a function to process the depth data.
    func processDepthData(depthData: AVDepthData) -> [[Float]] {
        let depthPixelBuffer = depthData.depthDataMap
        let width = CVPixelBufferGetWidth(depthPixelBuffer)
        let height = CVPixelBufferGetHeight(depthPixelBuffer)
        
        CVMetalTextureCacheCreate(kCFAllocatorDefault, nil, MetalEnvironment.shared.metalDevice, nil, &textureCache)
        CVMetalTextureCacheFlush(textureCache, 0)

        CVPixelBufferLockBaseAddress(depthPixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthPixelBuffer, .readOnly) }
        
        guard let pixelData = CVPixelBufferGetBaseAddress(depthPixelBuffer) else {
            print("Pixel data is not available.")
            return []
        }
        
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthPixelBuffer)
        var depthArray = [[Float]](repeating: [Float](repeating: 0, count: width), count: height)

        for y in 0..<height {
            for x in 0..<width {
                let pixelOffset = y * bytesPerRow + x * MemoryLayout<Float>.size
                let pixel = pixelData.load(fromByteOffset: pixelOffset, as: Float.self)
                depthArray[y][x] = pixel
            }
        }
        
        let processedData = divideIntoGrid(depthArray: depthArray)
//        print("Processed Data: \(processedData)")
        return processedData
    }
    
    // Mock implementation of depth data conversion.
    func convertDepthDataTo2DArray(depthData: AVDepthData) -> [[Float]] {
        // You will replace this mock logic with actual depth data conversion.
        // This is just to illustrate the point.
        let width = 1920 // Example width
        let height = 1440 // Example height
        var depthArray = [[Float]](repeating: [Float](repeating: 0, count: width), count: height)
        
        // Populate the array with mock data.
        for i in 0..<height {
            for j in 0..<width {
                // Replace with actual depth data extraction.
                depthArray[i][j] = Float.random(in: 0...1) // Mock distance value
            }
        }
        
        return depthArray
    }
    
    // Step 3: Implement the 'divideIntoGrid' function.
    func divideIntoGrid(depthArray: [[Float]]) -> [[Float]] {
        // Define the grid size.
        let gridRows = 4
        let gridColumns = 1
        
        // Determine the size of each grid cell.
        let cellWidth = depthArray[0].count / gridColumns
        let cellHeight = depthArray.count / gridRows
        
        // Initialize the 4x3 array with empty strings.
        var gridArray = [[Float]](repeating: [Float](repeating: 0, count: gridColumns), count: gridRows)

        
        for row in 0..<gridRows {
               for column in 0..<gridColumns {
                   // Calculate the average depth for the current cell.
                   var sum: Float = 0
                   for i in row*cellHeight..<(row+1)*cellHeight {
                       for j in column*cellWidth..<(column+1)*cellWidth {
                           sum += depthArray[i][j]
                       }
                   }
                   let averageDepth = sum / Float(cellWidth * cellHeight)
                   
                   // Store the average depth in the grid.
                   gridArray[row][column] = averageDepth
               }
           }
        
        // Print the entire grid in a formatted manner
//            for row in gridArray {
//                let rowString = row.map { String(format: "%.2f", $0) }.joined(separator: ", ")
//                print("[\(rowString)] \n")
//            }
        return gridArray.map { $0 }
    }
    
    // Function to categorize the depth.
    func categorizeDepth(averageDepth: Float) -> String {
        // Define your thresholds for low, medium, and high distances.
        let lowThreshold: Float = 0.33
        let highThreshold: Float = 0.66
        
        switch averageDepth {
        case 0..<lowThreshold:
            return "low"
        case lowThreshold..<highThreshold:
            return "medium"
        default:
            return "high"
        }
    }
}

// MARK: Output Synchronizer Delegate
extension CameraController: AVCaptureDataOutputSynchronizerDelegate {
    
    func dataOutputSynchronizer(_ synchronizer: AVCaptureDataOutputSynchronizer,
                                didOutput synchronizedDataCollection: AVCaptureSynchronizedDataCollection) {
        // Retrieve the synchronized depth and sample buffer container objects.
        guard let syncedDepthData = synchronizedDataCollection.synchronizedData(for: depthDataOutput) as? AVCaptureSynchronizedDepthData,
              let syncedVideoData = synchronizedDataCollection.synchronizedData(for: videoDataOutput) as? AVCaptureSynchronizedSampleBufferData else { return }
        
        guard let pixelBuffer = syncedVideoData.sampleBuffer.imageBuffer,
              let cameraCalibrationData = syncedDepthData.depthData.cameraCalibrationData else { return }
        
        // Package the captured data.
        let data = CameraCapturedData(depth: syncedDepthData.depthData.depthDataMap.texture(withFormat: .r16Float, planeIndex: 0, addToCache: textureCache),
                                      colorY: pixelBuffer.texture(withFormat: .r8Unorm, planeIndex: 0, addToCache: textureCache),
                                      colorCbCr: pixelBuffer.texture(withFormat: .rg8Unorm, planeIndex: 1, addToCache: textureCache),
                                      cameraIntrinsics: cameraCalibrationData.intrinsicMatrix,
                                      cameraReferenceDimensions: cameraCalibrationData.intrinsicMatrixReferenceDimensions)
        
        delegate?.onNewData(capturedData: data)
        
        // Use the real depth data to process and categorize it into the 4x3 grid.
        let processedArray = processDepthData(depthData: syncedDepthData.depthData)
        
        //            guard let processedArray = delegate?.onProcessedArrayData(processedArray: processedArray) else {
        //                    print("No processed data available to send.")
        //                    return
        //            }
        
        //        bluetoothViewModel?.sendData("ON")
        let processed2DArray = processDepthData(depthData: syncedDepthData.depthData)
            print("Processing synchronized data for transmission")
            self.bluetoothViewModel?.sendData(processed2DArray.flatMap { $0 })
        

        
//        DispatchQueue.main.asyncAfter(deadline: .now() + delaySeconds) {
//            do {
//                let jsonData = try JSONSerialization.data(withJSONObject: processedArray, options: [])
//                self.bluetoothViewModel?.sendData(jsonData)
//            } catch {
//                print("Error serializing JSON: \(error.localizedDescription)")
//            }
        }
        
        
        //        do {
        //            let jsonData = try JSONSerialization.data(withJSONObject: processedArray, options: [])
        //            bluetoothViewModel?.sendData(jsonData)
        //        } catch {
        //            print("Error serializing JSON: \(error.localizedDescription)")
        //        }
}

// MARK: Photo Capture Delegate
extension CameraController: AVCapturePhotoCaptureDelegate {
    
    func capturePhoto() {
        var photoSettings: AVCapturePhotoSettings
        if  photoOutput.availablePhotoPixelFormatTypes.contains(kCVPixelFormatType_420YpCbCr8BiPlanarFullRange) {
            photoSettings = AVCapturePhotoSettings(format: [
                kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
            ])
        } else {
            photoSettings = AVCapturePhotoSettings()
        }
        
        // Capture depth data with this photo capture.
        photoSettings.isDepthDataDeliveryEnabled = true
        photoOutput.capturePhoto(with: photoSettings, delegate: self)
    }
    
    func photoOutput(_ output: AVCapturePhotoOutput, didFinishProcessingPhoto photo: AVCapturePhoto, error: Error?) {
        
        // Retrieve the image and depth data.
        guard let pixelBuffer = photo.pixelBuffer,
              let depthData = photo.depthData,
              let cameraCalibrationData = depthData.cameraCalibrationData else { return }
        
        // Stop the stream until the user returns to streaming mode.
        stopStream()
        
        // Convert the depth data to the expected format.
        let convertedDepth = depthData.converting(toDepthDataType: kCVPixelFormatType_DepthFloat16)
        
        // Package the captured data.
        let data = CameraCapturedData(depth: convertedDepth.depthDataMap.texture(withFormat: .r16Float, planeIndex: 0, addToCache: textureCache),
                                      colorY: pixelBuffer.texture(withFormat: .r8Unorm, planeIndex: 0, addToCache: textureCache),
                                      colorCbCr: pixelBuffer.texture(withFormat: .rg8Unorm, planeIndex: 1, addToCache: textureCache),
                                      cameraIntrinsics: cameraCalibrationData.intrinsicMatrix,
                                      cameraReferenceDimensions: cameraCalibrationData.intrinsicMatrixReferenceDimensions)
        
        delegate?.onNewPhotoData(capturedData: data)
    }
}


// MARK: BT

class BluetoothViewModel: NSObject, ObservableObject, CBPeripheralDelegate {
    private var centralManager: CBCentralManager?
    private var myPeripheral: CBPeripheral?
    private var myCharacteristic: CBCharacteristic?
    private var writableCharacteristic: CBCharacteristic?
    @Published var isConnected: Bool = false
    
    
    override init() {
        super.init()
        self.centralManager = CBCentralManager(delegate: self, queue: .main)
    }
}

extension BluetoothViewModel: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            self.centralManager?.scanForPeripherals(withServices: nil)
            
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi RSSI: NSNumber) {
        if peripheral.name == "BT24" {
            centralManager?.stopScan()
            myPeripheral = peripheral
            centralManager?.connect(peripheral, options: nil)
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
            isConnected = true
            peripheral.delegate = self
            peripheral.discoverServices(nil)
        }
    
    // Peripheral Delegate Methods
        func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
            guard let services = peripheral.services else { return }
            for service in services {
                peripheral.discoverCharacteristics(nil, for: service)
            }
        }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
            peripheral.maximumWriteValueLength(for: .withoutResponse)
            guard let characteristics = service.characteristics else { return }
            for characteristic in characteristics {
                if characteristic.properties.contains(.writeWithoutResponse) {
                    writableCharacteristic = characteristic
                    break
                }
            }
        }

    
//    // Function to Send Data
//        func sendData(_ data: Int) {
//            guard let writableCharacteristic = writableCharacteristic else { return }
//            let dataToSend = Data([UInt8(data)])
//            myPeripheral?.writeValue(dataToSend, for: writableCharacteristic, type: .withoutResponse)
//        }
    
//     Function to Send Data (as String)
//    func sendData(_ string: String) {
//        guard let writableCharacteristic = writableCharacteristic else { return }
//        if let dataToSend = string.data(using: .utf8) {
//            myPeripheral?.writeValue(dataToSend, for: writableCharacteristic, type: .withoutResponse)
//        }
//    }
    
//    func sendData(_ data: [Float]) {
//        guard let writableCharacteristic = writableCharacteristic else {
//            print("No writable characteristic available.")
//            return
//        }
//
//        var binaryData = Data()
//        for value in data {
//            // Convert each float to a binary format
//            var binaryValue = value.bitPattern
//            let valueData = Data(bytes: &binaryValue, count: MemoryLayout<UInt32>.size)
//            binaryData.append(valueData)
//        }
//
//        // Debug: Print binary data size and content
//        print("Sending data of size: \(binaryData.count) bytes")
//        print("Binary data: \(binaryData as NSData)")
//
//        // Check if data size exceeds BLE packet limit
//        let maxDataSize = myPeripheral?.maximumWriteValueLength(for: .withoutResponse) ?? 0
//        if binaryData.count > maxDataSize {
//            print("Data size exceeds maximum BLE packet size of \(maxDataSize) bytes.")
//            return
//        }
//
//        // Send the binary data
//        myPeripheral?.writeValue(binaryData, for: writableCharacteristic, type: .withoutResponse)
//    }
    
    func sendData(_ data: [Float]) {
            guard let writableCharacteristic = writableCharacteristic else {
                print("No writable characteristic available.")
                return
            }

            // Convert the array of floats to a string
            let stringRepresentation = data.map { String($0) }.joined(separator: ",")
            
            // Convert the string to Data
            if let dataToSend = stringRepresentation.data(using: .utf8) {
                print("Sending string data: \(stringRepresentation)")
                myPeripheral?.writeValue(dataToSend, for: writableCharacteristic, type: .withoutResponse)
            }
        }


}

// Mark: LiDAR Sensor

// MARK: SwiftUI View
struct ContentView: View {
    @ObservedObject private var bluetoothViewModel = BluetoothViewModel()
    @ObservedObject private var cameraController = CameraController()
    
    @State private var isScanning = false
    @State private var scanTimer: Timer?

    var body: some View {
        NavigationView {
            VStack {
                if bluetoothViewModel.isConnected {
                    Text("Connected to BT24")
                    Button("On Data") {
                        bluetoothViewModel.sendData([0.001745131, 680.66675, 2161.2075, 2024.9708])
                        bluetoothViewModel.sendData([0.001745131, 680.66675, 2161.2075, 2024.9708])
                        bluetoothViewModel.sendData([0.001745131, 680.66675, 2161.2075, 2024.9708])
                        bluetoothViewModel.sendData([0.001745131, 680.66675, 2161.2075, 2024.9708])
                        bluetoothViewModel.sendData([0.001745131, 680.66675, 2161.2075, 2024.9708])
                        bluetoothViewModel.sendData([0.001745131, 680.66675, 2161.2075, 2024.9708])
                        bluetoothViewModel.sendData([0.001745131, 680.66675, 2161.2075, 2024.9708])
                        bluetoothViewModel.sendData([0.001745131, 680.66675, 2161.2075, 2024.9708])
                    }
                    Button("Off Data") {
                        bluetoothViewModel.sendData([2.0])
                    }
                    // LiDAR Control Button
                    Button("Toggle LiDAR") {
                        DispatchQueue.global().async {
                            if self.cameraController.captureSession.isRunning {
                                self.cameraController.stopStream()
                            } else {
                                self.cameraController.startStream()
                            }
                        }
                    }                } else {
                    Text("Scanning for BT24...")
                }
            }
            .navigationTitle("BLE Control")
        }
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}

#Preview {
    ContentView()
}
