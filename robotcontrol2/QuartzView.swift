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
    let centimetersPerPoint = 1
    
    override func drawRect(dirtyRect: NSRect) {
        let background = NSBezierPath(rect: self.bounds)
        let black = NSColor.blackColor()
        black.set()
        background.fill()
        
        let fillColor = NSColor.whiteColor()
        fillColor.set()
        
        let sensordata = controller.sensorData()
        let fYaw = yawToRadians(sensordata.last?.0.m_nYaw ?? 0)
        let ptLast = sensordata.last?.1 ?? CGPointZero
        
        // TODO: Always unscaled
        let sizeImage = controller.occupancy().size
        controller.occupancy().drawInRect(NSRect(
            x: self.bounds.width/2 - ptLast.x - sizeImage.width/2,
            y: self.bounds.height/2 - ptLast.y + sizeImage.height/2,
            width: sizeImage.width,
            height: -sizeImage.height
        ))
        
        var transformPath = NSAffineTransform()
        transformPath.translateXBy(self.bounds.width/2, yBy: self.bounds.height/2)
        // transformPath.scaleBy(CGFloat(1.0 / Double(centimetersPerPoint))) // 1000 encoders ticks = 3 rotations
        transformPath.translateXBy(-ptLast.x, yBy: -ptLast.y)
        transformPath.transformBezierPath(controller.path()).stroke()
        
        var transform = NSAffineTransform()
        transform.translateXBy(self.bounds.width/2, yBy: self.bounds.height/2)
        // transform.scaleBy(CGFloat(1.0 / Double(centimetersPerPoint)))
        transform.rotateByRadians(CGFloat(fYaw))
        
        var outline = NSBezierPath(rect: NSMakeRect(-5.0, -5.0, 10.0, 10.0))
        outline.moveToPoint(NSMakePoint(7.5, 0))
        outline.lineToPoint(NSMakePoint(2.5, 0))
        transform.transformBezierPath(outline).stroke()
    }
    
    override func keyDown(theEvent: NSEvent) {
        if theEvent.modifierFlags & .NumericPadKeyMask != nil {
            interpretKeyEvents([theEvent])
        } else {
            super.keyDown(theEvent)
        }
    }
    
    override func moveUp(sender: AnyObject?) {
        controller.moveForward()
    }
    
    override func moveDown(sender: AnyObject?) {
        controller.moveBackward()
    }
    
    override func moveLeft(sender: AnyObject?) {
        controller.turnLeft()
    }
    
    override func moveRight(sender: AnyObject?) {
        controller.turnRight()
    }
    
    override var acceptsFirstResponder : Bool {
        get {
            return true
        }
    }
    
    override func viewDidHide() {
        controller = nil
    }
    
    var controller : RobotController!
}

