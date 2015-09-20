//
//  QuartzView.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 22.11.14.
//  Copyright (c) 2014 Sebastian Theophil. All rights reserved.
//

import Foundation
import Cocoa

func -(left: CGPoint, right: CGVector) -> CGPoint {
    return CGPointMake(left.x - right.dx, left.y - right.dy)
}

func *(left: CGSize, right: CGFloat) -> CGSize {
    return CGSize(width: left.width * right, height: left.height * right)
}

func /(left: CGSize, right: CGFloat) -> CGSize {
    return CGSize(width: left.width / right, height: left.height / right)
}

extension CGVector {
    init(_ sz: CGSize) {
        dx = sz.width
        dy = sz.height
    }
}

extension CGRect {
    init(centeredAt pt: CGPoint, size: CGSize) {
        self.origin = pt - CGVector(size/2)
        self.size = size
    }
}

class QuartzView : NSView {
    let centimetersPerPoint = 1
    
    func withGraphicsState( function : () -> () ) {
        NSGraphicsContext.saveGraphicsState()
        function()
        NSGraphicsContext.restoreGraphicsState()
    }
    
    override func drawRect(dirtyRect: NSRect) {
        let background = NSBezierPath(rect: self.bounds)
        let black = NSColor.blackColor()
        black.set()
        background.fill()
        
        let fillColor = NSColor.redColor()
        fillColor.set()
        
        let aptfPositions = controller.positions()
        let ptLast = aptfPositions.last?.0 ?? CGPointZero
        let fYaw = aptfPositions.last?.1 ?? 0
        
        withGraphicsState() {
            let transform = NSAffineTransform()
            transform.translateXBy(self.bounds.width/2, yBy: self.bounds.height/2)
            transform.translateXBy(-ptLast.x, yBy: -ptLast.y)
            transform.concat()
            
            self.controller.draw()
        }
        
        // Draw robot outline rotated by fYaw
        let transformRotate = NSAffineTransform()
        transformRotate.translateXBy(self.bounds.width/2, yBy: self.bounds.height/2)
        transformRotate.rotateByRadians(CGFloat(fYaw))
        
        let outline = NSBezierPath(rect: NSRect(centeredAt: NSMakePoint(0, 0), size: NSMakeSize(CGFloat(c_nRobotWidth), CGFloat(c_nRobotHeight))))
        outline.moveToPoint(NSMakePoint(CGFloat(c_nRobotHeight) / 2 + 2.5, 0))
        outline.lineToPoint(NSMakePoint(CGFloat(c_nRobotHeight) / 2 - 2.5, 0))
        transformRotate.transformBezierPath(outline).stroke()
    }
    
    override func keyDown(theEvent: NSEvent) {
        if theEvent.modifierFlags.intersect(.NumericPadKeyMask) != [] {
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
    
    var controller : RobotViewController!
}

