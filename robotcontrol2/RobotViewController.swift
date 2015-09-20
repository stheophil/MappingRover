//
//  ViewController.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 22.11.14.
//  Copyright (c) 2014 Sebastian Theophil. All rights reserved.
//

import Cocoa

class RobotViewController: NSViewController {

    // View interface
    override func viewDidLoad() {
        super.viewDidLoad()
        
        viewRender.becomeFirstResponder()
        viewRender.controller = self
        
        m_robotcontroller = robot_new_controller();
        
        m_ble = BLE(controller: self)
        
        m_bezierpath.moveToPoint(NSPoint(x: 0, y: 0))
        
        // NSTimer.scheduledTimerWithTimeInterval(2, target: self, selector: Selector("onTimer"), userInfo: nil, repeats: true)
        
        assert(sizeof(SSensorData)==18)
        assert(sizeof(SRobotCommand)==8)
    }

    /*
    var nYawPrev : Int16 = 0
    func onTimer() {
        nYawPrev += 200
        let dataLeft = SSensorData(m_nPitch: 0, m_nRoll: 0, m_nYaw: nYawPrev, m_nAngle: 90, m_nDistance: 200, m_anEncoderTicks: (10, 10, 10, 10))
        receivedSensorData(dataLeft)
        
        let dataRight = SSensorData(m_nPitch: 0, m_nRoll: 0, m_nYaw: nYawPrev, m_nAngle: -90, m_nDistance: 150, m_anEncoderTicks: (10, 10, 10, 10))
        receivedSensorData(dataRight)
        
        let dataFront = SSensorData(m_nPitch: 0, m_nRoll: 0, m_nYaw: nYawPrev, m_nAngle: 0, m_nDistance: 170, m_anEncoderTicks: (10, 10, 10, 10))
        receivedSensorData(dataFront)
    }
    */
    
    override func viewDidDisappear() {
        m_ble = nil
    }

    override var representedObject: AnyObject? {
        didSet {
        // Update the view, if already loaded.
        }
    }
    
    // RobotController interface
    func sendCommand(var cmd: SRobotCommand) {
        m_ble.sendControlCommand(NSData(bytes:&cmd, length:sizeof(SRobotCommand)))
    }
    
    func moveForward() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: c_nMaxFwdSpeed, m_nSpeedRight: c_nMaxFwdSpeed))
    }
    
    func moveBackward() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: -c_nMaxFwdSpeed, m_nSpeedRight: -c_nMaxFwdSpeed))
    }
    
    func turnLeft() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: -c_nMaxTurnSpeed, m_nSpeedRight: c_nMaxTurnSpeed))
    }
    
    func turnRight() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: c_nMaxTurnSpeed, m_nSpeedRight: -c_nMaxTurnSpeed))
    }
    
    func receivedSensorData(data: SSensorData) {
        log("Sensor data Z: \(data.m_nYaw) Sonar: \(data.m_nDistance) @ \(data.m_nAngle) Distances: \(data.m_anEncoderTicks.0), \(data.m_anEncoderTicks.1), \(data.m_anEncoderTicks.2), \(data.m_anEncoderTicks.3)\n")
        
        let pose = robot_received_sensor_data(m_robotcontroller, data)
        
        m_apairptf.append( (CGPoint(x: Int(pose.x), y: Int(pose.y)), CGFloat(pose.fYaw)) )
        m_bezierpath.lineToPoint(m_apairptf.last!.0)
        viewRender.needsDisplay = true
        
        // TODO: Get current planned path from m_robotcontroller
    }
    
    func positions() -> [(CGPoint, CGFloat)] {
        return m_apairptf
    }
    
    func draw() {
        let btnSelected = radiobtnShowMap.selectedCell() as! NSButtonCell
        let bShowErodedMap = btnSelected.tag == 1
        
        var bitmap = robot_get_map(m_robotcontroller, bShowErodedMap)
        if let bitmaprep = NSBitmapImageRep(bitmapDataPlanes: &bitmap.m_pbImage,
            pixelsWide: bitmap.m_nWidth,
            pixelsHigh: bitmap.m_nHeight,
            bitsPerSample: 8,
            samplesPerPixel: 1,
            hasAlpha: false,
            isPlanar: false,
            colorSpaceName: NSDeviceWhiteColorSpace,
            bytesPerRow: bitmap.m_cbBytesPerRow,
            bitsPerPixel: 0) {
                
            let sizeImage = bitmaprep.size * CGFloat(bitmap.m_nScale)
            bitmaprep.drawInRect(NSRect(
                x: -sizeImage.width/2,
                y: sizeImage.height/2,
                width: sizeImage.width,
                height: -sizeImage.height
            ))
        }
        
        m_bezierpath.stroke()
        
        if 0<m_cPath.count {
            NSColor.greenColor().set()
            let path = NSBezierPath()
            path.moveToPoint(m_cPath.first!)
            for pt in m_cPath {
                path.lineToPoint(pt)
            }
            path.stroke()
        }
    }
    
    func log(msg: String) {
        print(msg, terminator: "")
        // textview.string? += msg
        // textview.scrollRangeToVisible(NSMakeRange(textview.string!.lengthOfBytesUsingEncoding(NSUTF8StringEncoding), 0))
    }

    var m_robotcontroller : COpaquePointer = nil
    var m_apairptf = [(CGPoint, CGFloat)]()
    var m_bezierpath = NSBezierPath()

    var m_cPath = [CGPoint]()
    
    var m_ble : BLE!
    
    @IBOutlet var viewRender: QuartzView!
    @IBOutlet var textview: NSTextView!
    @IBOutlet var radiobtnShowMap: NSMatrix!
}

