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
    
    func log(msg: String)
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
        
        assert(sizeof(SSensorData)==18)
        assert(sizeof(SRobotCommand)==8)
    }
    
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
        if sensorDataArray.last?.0.m_nYaw != data.m_nYaw
            || data.m_anEncoderTicks.0 != 0
            || data.m_anEncoderTicks.1 != 0
            || data.m_anEncoderTicks.2 != 0
            || data.m_anEncoderTicks.3 != 0
        {
            log("Sensor data Z: \(data.m_nYaw) Sonar: \(data.m_nDistance) Distances: \(data.m_anEncoderTicks.0), \(data.m_anEncoderTicks.1), \(data.m_anEncoderTicks.2), \(data.m_anEncoderTicks.3)\n")
            
            // left motors are 0 and 2, right motors 1, 3
            // assert( sign(data.m_anEncoderTicks.0) == sign(data.m_anEncoderTicks.2) )
            // assert( sign(data.m_anEncoderTicks.1) == sign(data.m_anEncoderTicks.3) )
            
            // TODO: Create path shape here
            // TODO: Collapse turns into single point
            
            let ptPrev = sensorDataArray.last?.1 ?? CGPoint(x: 0, y: 0)
            var pt : CGPoint
            if sign(data.m_anEncoderTicks.0) != sign(data.m_anEncoderTicks.1) { // turn
                // position does not change
                pt = ptPrev
            } else {
                let distance = data.m_anEncoderTicks.0 // average the encoder? convert to cm here?
                pt = CGPoint(
                    x: CGFloat(data.m_anEncoderTicks.0) * cos(CGFloat(yawToRadians(data.m_nYaw))) + ptPrev.x,
                    y: CGFloat(data.m_anEncoderTicks.0) * sin(CGFloat(yawToRadians(data.m_nYaw))) + ptPrev.y
                )
            }
            
            sensorDataArray.append((data, pt))
            viewRender.needsDisplay = true
        }
    }
    
    func sensorData() -> [(SSensorData, CGPoint)] {
        return sensorDataArray
    }
    
    func log(msg: String) {
        print(msg)
        // textview.string? += msg
        // textview.scrollRangeToVisible(NSMakeRange(textview.string!.lengthOfBytesUsingEncoding(NSUTF8StringEncoding), 0))
    }
    
    var sensorDataArray = [(SSensorData, CGPoint)]() // sensor data including accumulated position
    
    var ble : BLE!;
    
    @IBOutlet var viewRender: QuartzView!
    @IBOutlet var textview: NSTextView!
    @IBOutlet var btnMeasurement: NSButton!
}

