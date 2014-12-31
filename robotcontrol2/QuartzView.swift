//
//  QuartzView.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 22.11.14.
//  Copyright (c) 2014 Sebastian Theophil. All rights reserved.
//

import Foundation
import Cocoa

func yawToRadians(yaw: Int16) -> Double {
     return Double(-yaw)/1000.0
}

class QuartzView : NSView {
    let centimetersPerPoint = 2
    let wheelRadius = 3 // cm
    
    override func drawRect(dirtyRect: NSRect) {        
        //       println(dirtyRect)
        let background = NSBezierPath(rect: self.bounds)
        let black = NSColor.blackColor()
        black.set()
        background.fill()
        
        let fillColor = NSColor.whiteColor()
        fillColor.set()
        
        let sensordata = controller.sensorData()
        let fYaw = sensordata.isEmpty ? 0.0 : yawToRadians(sensordata.last!.0.m_nYaw)
        let ptLast = sensordata.last?.1 ?? CGPointZero
        
        var transform = NSAffineTransform()
        var transformPath = NSAffineTransform()
        
        transform.translateXBy(self.bounds.width/2, yBy: self.bounds.height/2)
        transform.rotateByRadians(CGFloat(fYaw))
        
        transformPath.translateXBy(self.bounds.width/2, yBy: self.bounds.height/2)
        transformPath.scaleBy(CGFloat(6.0 * M_PI * Double(wheelRadius) / 1000.0 / Double(centimetersPerPoint))) // 1000 encoders ticks = 3 rotations
        transformPath.translateXBy(-ptLast.x, yBy: -ptLast.y)
        
        var ptPrev = transformPath.transformPoint(CGPointZero)
        for (_, pt) in sensordata {
            let path = NSBezierPath()
            let ptScaled = transformPath.transformPoint(pt)
            path.moveToPoint(ptPrev)
            path.lineToPoint(ptScaled)
            path.stroke()
            
            ptPrev = ptScaled
        }

        transform.concat()
        let bPath = NSBezierPath(rect: NSMakeRect(-5.0, -5.0, 10.0, 10.0))
        bPath.stroke()
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

