//
//  ViewController.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 22.11.14.
//  Copyright (c) 2014 Sebastian Theophil. All rights reserved.
//

import Cocoa

enum ECommand : Int {
    case Forward
    case Left
    case Right
    case Back
}

struct SSensorData {
    // TODO
}

protocol RobotController {
    func sendCommand(cmd: ECommand)
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
    func sendCommand(cmd: ECommand) {
        ble.sendControlCommand(cmd.rawValue)
    }
    
    func receivedSensorData(data: SSensorData) {
    }
    
    func log(msg: String) {
        textview.string? += msg
    }
    
    var ble : BLE!;
    
    @IBOutlet var viewRender: QuartzView!
    @IBOutlet var textview: NSTextView!
    @IBOutlet var btnMeasurement: NSButton!
}

