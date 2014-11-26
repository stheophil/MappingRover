//
//  QuartzView.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 22.11.14.
//  Copyright (c) 2014 Sebastian Theophil. All rights reserved.
//

import Foundation
import Cocoa

class QuartzView : NSView {
    // unit: 1 pt = 10cm
    
    override func drawRect(dirtyRect: NSRect) {        
        //       println(dirtyRect)
        let background = NSBezierPath(rect: self.bounds)
        let black = NSColor.blackColor()
        black.set()
        background.fill()
        
        var transform = NSAffineTransform()
        transform.translateXBy(self.bounds.width/2, yBy: self.bounds.height/2)
        transform.concat()
        
        let bPath = NSBezierPath(rect: NSMakeRect(-5.0, -5.0, 10.0, 10.0))
        let fillColor = NSColor.whiteColor()
        fillColor.set()
        bPath.stroke()
    }
    
    weak var model : DataModel!
}

