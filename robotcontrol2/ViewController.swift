//
//  ViewController.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 22.11.14.
//  Copyright (c) 2014 Sebastian Theophil. All rights reserved.
//

import Cocoa

class DataModel : NSObject {
    override init() {}
}

class ViewController: NSViewController {
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        for i in 0 ..< 10 {
            textview.string? += "View Loaded\n"
            let v = btnMeasurement.state
            textview.string? += "Button is \(v)"
        }
        
        model = DataModel()
        viewRender.model = model
    }

    override var representedObject: AnyObject? {
        didSet {
        // Update the view, if already loaded.
        }
    }
    
    var model : DataModel!;
    
    @IBOutlet weak var viewRender: QuartzView!
    @IBOutlet weak var textview: NSTextView!
    @IBOutlet weak var btnMeasurement: NSButton!
}

