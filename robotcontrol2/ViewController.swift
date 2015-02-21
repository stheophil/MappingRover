//
//  ViewController.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 22.11.14.
//  Copyright (c) 2014 Sebastian Theophil. All rights reserved.
//

import Cocoa

func sign<T : IntegerType>(x : T) -> Int {
    return x < 0 ? -1 : (0 < x ? 1 : 0)
}

protocol RobotController {
    // Robot control
    func moveForward()
    func moveBackward()
    func turnLeft()
    func turnRight()
    func sendCommand(cmd: SRobotCommand)
    
    // Robot sensors
    func receivedSensorData(data: SSensorData)
    func sensorData() -> [(SSensorData, CGPoint)]
    
    func path() -> NSBezierPath
    func occupancy() -> OccupancyGrid
    
    func log(msg: String)
}

// Robot configuration. Needs calibration.
let wheelRadius = CGFloat(3) // cm
// TODO: robot dimensions, sensor x-y offset
let sonarMaxDistance = CGFloat(300.0) // cm = Sonar max distance depends on mounting height
let sonarOpeningAngle = CGFloat(M_PI_2 / 6) // 15 degrees = Sensor opening angle
let sonarDistanceTolerance = CGFloat(5.0)

func encoderTicksToCm(ticks: Int16) -> CGFloat {
    return CGFloat(ticks) * CGFloat(6.0) * CGFloat(M_PI) * wheelRadius / 1000.0
}

func yawToRadians(yaw: Int16) -> CGFloat {
    return CGFloat(-yaw)/1000.0
}

class ViewController: NSViewController, RobotController {
    let maxTurnSpeed : Int16 = 200
    let maxFwdSpeed : Int16 = 300
    
    // View interface
    override func viewDidLoad() {
        super.viewDidLoad()
        
        viewRender.becomeFirstResponder()
        viewRender.controller = self
        
        ble = BLE(controller: self)
        
        bezierpath.moveToPoint(NSPoint(x: 0, y: 0))
        
        // NSTimer.scheduledTimerWithTimeInterval(2, target: self, selector: Selector("onTimer"), userInfo: nil, repeats: true)
        
        assert(sizeof(SSensorData)==18)
        assert(sizeof(SRobotCommand)==8)
    }
    
    /*
    var nYawPrev : Int16 = 0
    func onTimer() {
        nYawPrev += 300
        let data = SSensorData(m_nPitch: 0, m_nRoll: 0, m_nYaw: nYawPrev, m_nAngle: 90, m_nDistance: 200, m_anEncoderTicks: (400, 400, 400, 400))
        receivedSensorData(data)
    }
    */
    
    override func viewDidDisappear() {
        ble = nil
    }

    override var representedObject: AnyObject? {
        didSet {
        // Update the view, if already loaded.
        }
    }
    
    // RobotController interface
    func sendCommand(var cmd: SRobotCommand) {
        ble.sendControlCommand(NSData(bytes:&cmd, length:sizeof(SRobotCommand)))
    }
    
    func moveForward() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: maxFwdSpeed, m_nSpeedRight: maxFwdSpeed))
    }
    
    func moveBackward() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: -maxFwdSpeed, m_nSpeedRight: -maxFwdSpeed))
    }
    
    func turnLeft() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: -maxTurnSpeed, m_nSpeedRight: maxTurnSpeed))
    }
    
    func turnRight() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: maxTurnSpeed, m_nSpeedRight: -maxTurnSpeed))
    }
    
    func receivedSensorData(data: SSensorData) {
        log("Sensor data Z: \(data.m_nYaw) Sonar: \(data.m_nDistance) @ \(data.m_nAngle) Distances: \(data.m_anEncoderTicks.0), \(data.m_anEncoderTicks.1), \(data.m_anEncoderTicks.2), \(data.m_anEncoderTicks.3)\n")
        
        // left motors are 0 and 2, right motors 1, 3
        // assert( sign(data.m_anEncoderTicks.0) == sign(data.m_anEncoderTicks.2) )
        // assert( sign(data.m_anEncoderTicks.1) == sign(data.m_anEncoderTicks.3) )
        
        let ptPrev = sensorDataArray.last?.1 ?? CGPointZero
        var pt : CGPoint
        if sign(data.m_anEncoderTicks.0) != sign(data.m_anEncoderTicks.1) { // turn
            // position does not change
            pt = ptPrev
        } else {
            pt = toPoint(yawToRadians(data.m_nYaw), encoderTicksToCm(data.m_anEncoderTicks.0))
            pt.x += ptPrev.x
            pt.y += ptPrev.y
            
            bezierpath.lineToPoint(pt)
        }
        sensorDataArray.append((data, pt))
        viewRender.needsDisplay = true
        
        occgrid.update(pt, fYaw: yawToRadians(data.m_nYaw), nAngle: data.m_nAngle, nDistance: data.m_nDistance)
    }
    
    func sensorData() -> [(SSensorData, CGPoint)] {
        return sensorDataArray
    }
    
    func path() -> NSBezierPath {
        return bezierpath
    }
    
    func occupancy() -> OccupancyGrid {
        return occgrid
    }
    
    func log(msg: String) {
        print(msg)
        // textview.string? += msg
        // textview.scrollRangeToVisible(NSMakeRange(textview.string!.lengthOfBytesUsingEncoding(NSUTF8StringEncoding), 0))
    }
    
    var sensorDataArray = [(SSensorData, CGPoint)]() // sensor data including accumulated position
    var bezierpath = NSBezierPath()
    var occgrid = OccupancyGrid()
    
    var ble : BLE!
    
    @IBOutlet var viewRender: QuartzView!
    @IBOutlet var textview: NSTextView!
    @IBOutlet var btnMeasurement: NSButton!
}

