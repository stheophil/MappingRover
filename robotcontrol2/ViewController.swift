//
//  ViewController.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 22.11.14.
//  Copyright (c) 2014 Sebastian Theophil. All rights reserved.
//

import Cocoa

protocol RobotController {
    func moveForward()
    func moveBackward()
    func turnLeft()
    func turnRight()
    func sendCommand(cmd: SRobotCommand)
    func receivedSensorData(data: SSensorData)
    
    func log(msg: String)
}

class ViewController: NSViewController, RobotController {
    // View interface
    override func viewDidLoad() {
        super.viewDidLoad()
        
        viewRender.becomeFirstResponder()
        viewRender.controller = self
        
        ble = BLE(controller: self)
        
        assert(sizeof(SSensorData)==14)
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
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: 255, m_nSpeedRight: 255))
    }
    
    func moveBackward() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: -255, m_nSpeedRight: -255))
    }
    
    func turnLeft() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: -255, m_nSpeedRight: 255))
    }
    
    func turnRight() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: 255, m_nSpeedRight: -255))
    }
    
    func receivedSensorData(data: SSensorData) {
        log("Sensor data Z: \(data.m_nYaw) Distances: \(data.m_anEncoderTicks.0), \(data.m_anEncoderTicks.1), \(data.m_anEncoderTicks.2), \(data.m_anEncoderTicks.3)\n")
    }
    
    func log(msg: String) {
        textview.string? += msg
        textview.scrollRangeToVisible(NSMakeRange(textview.string!.lengthOfBytesUsingEncoding(NSUTF8StringEncoding), 0))
    }
    
    var ble : BLE!;
    
    @IBOutlet var viewRender: QuartzView!
    @IBOutlet var textview: NSTextView!
    @IBOutlet var btnMeasurement: NSButton!
}

