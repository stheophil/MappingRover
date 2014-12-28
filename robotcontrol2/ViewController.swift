//
//  ViewController.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 22.11.14.
//  Copyright (c) 2014 Sebastian Theophil. All rights reserved.
//

import Cocoa

// TODO: Take definitions from common C header
let MoveCommand : UInt16 = 0x1010  // arg1: movement direction & speed left, arg2: right

struct SRobotCommand {
    var cmd : UInt16
    var arg1 : Int16
    var arg2 : Int16
    
    init(leftSpeed speedLeft: Int16, rightSpeed speedRight: Int16) {
        cmd = MoveCommand
        arg1 = speedLeft
        arg2 = speedRight
    }
};

struct SSensorData {
    var pitch : Int16
    var roll : Int16
    var yaw : Int16
    var ticks0 : Int16;
    var ticks1 : Int16;
    var ticks2 : Int16;
    var ticks3 : Int16;
};

let SIZEOF_SSENSORDATA = 14; // sizeof(SSensorData) in Swift != sizeof(SSensorData) in C

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
        assert(sizeof(SRobotCommand)==6)
        ble.sendControlCommand(NSData(bytes:&cmd, length:sizeof(SRobotCommand)))
    }
    
    func moveForward() {
        sendCommand(SRobotCommand(leftSpeed: 255, rightSpeed: 255))
    }
    
    func moveBackward() {
        sendCommand(SRobotCommand(leftSpeed: -255, rightSpeed: -255))
    }
    
    func turnLeft() {
        sendCommand(SRobotCommand(leftSpeed: -255, rightSpeed: 255))
    }
    
    func turnRight() {
        sendCommand(SRobotCommand(leftSpeed: 255, rightSpeed: -255))
    }
    
    func receivedSensorData(data: SSensorData) {
        log("Sensor data Z: \(data.yaw) Distances: \(data.ticks0), \(data.ticks1), \(data.ticks2), \(data.ticks3)\n")
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

