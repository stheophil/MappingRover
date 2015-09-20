//
//  ViewController.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 22.11.14.
//  Copyright (c) 2014 Sebastian Theophil. All rights reserved.
//

import Cocoa

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
    
    // draw
    func draw()
    
    func log(msg: String)
}

// Robot configuration. Needs calibration.
let wheelRadius = CGFloat(6) // cm
let sizeRobot = CGSize(width: 30, height: 30) // cm
let anSonarOffset = [6, 7, 2] // cm for -90, 0, 90 Angle
func sonarOffset(nAngle: Int16) -> Int {
    assert(nAngle==0 || abs(nAngle)==90)
    return anSonarOffset[sign(nAngle) + 1]
}

let sonarMaxDistance = CGFloat(300.0) // cm = Sonar max distance depends on mounting height
let sonarOpeningAngle = CGFloat(M_PI_2 / 6) // 15 degrees = Sensor opening angle
let sonarDistanceTolerance = CGFloat(5.0)

func encoderTicksToCm(ticks: Int16) -> CGFloat {
    return CGFloat(ticks) * CGFloat(6.0) * CGFloat(M_PI) * wheelRadius / 1000.0
}

func yawToRadians(yaw: Int16) -> CGFloat {
    return CGFloat(-yaw)/1000.0
}

func withTimer(fn: () -> Void) {
    var timebase = mach_timebase_info_data_t()
    mach_timebase_info(&timebase)
    
    let tBegin = mach_absolute_time();
    
    fn()
    
    let tEnd = mach_absolute_time();
    let tNanoseconds = (tEnd - tBegin) * UInt64(timebase.numer) / UInt64(timebase.denom)
    
    NSLog("\(tNanoseconds / 1000000) ms")
}

class ViewController: NSViewController, RobotController {
    let maxTurnSpeed : Int16 = 200
    let maxFwdSpeed : Int16 = 200
    
    // View interface
    override func viewDidLoad() {
        super.viewDidLoad()
        
        viewRender.becomeFirstResponder()
        viewRender.controller = self
        
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
        
        withTimer({() in
            let ptPrev = self.m_apairsdatapt.last?.1 ?? CGPointZero
            var pt : CGPoint
            if sign(data.m_anEncoderTicks.0) != sign(data.m_anEncoderTicks.1) { // turn
                // position does not change
                pt = ptPrev
            } else {
                pt = CGPoint(fromAngle: yawToRadians(data.m_nYaw), distance: encoderTicksToCm(data.m_anEncoderTicks.0))
                pt.x += ptPrev.x
                pt.y += ptPrev.y
                
                self.m_bezierpath.lineToPoint(pt)
            }
            self.m_apairsdatapt.append((data, pt))
            self.viewRender.needsDisplay = true
            
            let fYaw = yawToRadians(data.m_nYaw)
            self.m_occgrid.update(pt, fYaw: fYaw, nAngle: data.m_nAngle, nDistance: data.m_nDistance + sonarOffset(data.m_nAngle))
            
            // Update path to closest points
            self.m_cPathToClosest = self.m_occgrid.closestUnknownPoint(pt, fYaw: fYaw)
        })
    }
    
    func sensorData() -> [(SSensorData, CGPoint)] {
        return m_apairsdatapt
    }
    
    func draw() {
        let btnSelected = radiobtnShowMap.selectedCell() as! NSButtonCell
        m_occgrid.draw(btnSelected.tag == 0)
        m_bezierpath.stroke()
        
        if 0<m_cPathToClosest.count {
        NSColor.greenColor().set()
            let path = NSBezierPath()
            path.moveToPoint(m_cPathToClosest.first!)
            for pt in m_cPathToClosest {
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
    
    var m_apairsdatapt = [(SSensorData, CGPoint)]() // sensor data including accumulated position
    var m_bezierpath = NSBezierPath()
    var m_occgrid = OccupancyGrid()
    
    var m_cPathToClosest = [CGPoint]()
    
    var m_ble : BLE!
    
    @IBOutlet var viewRender: QuartzView!
    @IBOutlet var textview: NSTextView!
    @IBOutlet var radiobtnShowMap: NSMatrix!
}

