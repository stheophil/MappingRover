//
//  Occupancy.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 20.02.15.
//  Copyright (c) 2015 Sebastian Theophil. All rights reserved.
//

import Foundation
import Cocoa

// Geometry extensions
func toPoint(angle: CGFloat, distance: CGFloat) -> CGPoint {
    return CGPoint(
        x: distance * cos(angle),
        y: distance * sin(angle)
    )
}

func angularDistance( angleA : CGFloat, angleB : CGFloat ) -> CGFloat {
    var angle = angleA - angleB
    // normalize angle to be between (-pi, +pi]
    while(angle < CGFloat(-M_PI)) {
        angle+=2*CGFloat(M_PI)
    }
    while(CGFloat(M_PI) <= angle) {
        angle-=2*CGFloat(M_PI)
    }
    return angle
}

func + (left: CGPoint, right: CGVector) -> CGPoint {
    return CGPointMake(left.x + right.dx, left.y + right.dy)
}

func + (left: CGPoint, right: CGPoint) -> CGPoint {
    return CGPointMake(left.x + right.x, left.y + right.y)
}

func /(left: CGPoint, right: CGFloat) -> CGPoint {
    return CGPointMake(left.x / right, left.y / right)
}

func *(left: CGSize, right: CGFloat) -> CGSize {
    return CGSize(width: left.width * right, height: left.height * right)
}

extension CGPoint {
    func distance(pt : CGPoint) -> CGFloat {
        return hypot(x - pt.x, y - pt.y)
    }
}

func boundRect(points: CGPoint...) -> CGRect {
    let bottomLeft = CGPoint(
        x: minElement(points.map{ $0.x }),
        y: minElement(points.map{ $0.y })
    )
    let topRight = CGPoint(
        x: maxElement(points.map{ $0.x }),
        y: maxElement(points.map{ $0.y })
    )
    return CGRect(x: bottomLeft.x,
        y: bottomLeft.y,
        width: topRight.x-bottomLeft.x,
        height: topRight.y-bottomLeft.y
    )
}

class OccupancyGrid {
    init() {
        // occupancy grid in log odds representation
        grid = [[Double]](count: extent,
            repeatedValue: [Double](count: extent, repeatedValue: 0.0))
        
        // occupancy grid as 8-bit greyscale image
        // (0,0) is top-left, different from other Cocoa coordinate systems
        image = NSBitmapImageRep(bitmapDataPlanes: nil,
            pixelsWide: extent, pixelsHigh: extent,
            bitsPerSample: 8,
            samplesPerPixel: 1,
            hasAlpha: false,
            isPlanar: true,
            colorSpaceName: NSDeviceWhiteColorSpace,
            bitmapFormat: NSBitmapFormat.allZeros,
            bytesPerRow: 0, bitsPerPixel: 0)!

        
        var color = 128
        for x in 0 ..< Int(image.size.width) {
            for y in 0 ..< Int(image.size.height) {
                image.setPixel(&color, atX: x, y: y)
            }
        }
    }
    
    func update(pt: CGPoint, fYaw: CGFloat, nAngle : Int16, nDistance : Int16 ) {
        // update occgrid & occgridImage
        assert(nAngle==0 || abs(nAngle)==90)
        let angleSonar = CGFloat(fYaw) + CGFloat(M_PI_2) * CGFloat(sign(nAngle))
        
        let gridOffset = CGVector(dx: extent/2, dy: extent/2)
        let sonarPosition = (pt / scale) + gridOffset // TODO: Sonar offset relative to robot center
        
        // approximate line scan bound rect
        let bounds = boundRect(
            sonarPosition,
            toPoint(angleSonar - sonarOpeningAngle/2, sonarMaxDistance/scale) + sonarPosition,
            toPoint(angleSonar, sonarMaxDistance/scale) + sonarPosition,
            toPoint(angleSonar + sonarOpeningAngle/2, sonarMaxDistance/scale) + sonarPosition
            ).integerRect // bound rect in integer coordinates
        
        // log("Sensor bound rect: (\(bounds.minX), \(bounds.minY), \(bounds.maxX), \(bounds.maxY))\n")
        
        // TODO: Resize occupancy grids
        assert(NSContainsRect(
            NSRect(x: 0, y: 0, width: extent, height: extent),
            bounds
            ))
        
        for y in Int(bounds.minY) ... Int(bounds.maxY) {
            var bFoundPointsInLine = false
            for x in Int(bounds.minX) ... Int(bounds.maxX) {
                // distance and angle of vector from sonar sensor to point (x, y)
                // distance in unscaled coordinates
                let distance = sonarPosition.distance(CGPoint(x: x, y: y)) * scale
                let angle = atan2(CGFloat(y) - sonarPosition.y, CGFloat(x) - sonarPosition.x)
                
                if CGFloat(nDistance) < sonarMaxDistance
                    && distance < CGFloat(nDistance) + sonarDistanceTolerance/2
                    && abs(angularDistance(angle, angleSonar)) <= sonarOpeningAngle/2 {
                        bFoundPointsInLine = true
                        
                        var inverseSensorModel : Double
                        if distance < CGFloat(nDistance) - sonarDistanceTolerance/2 {
                            inverseSensorModel = -0.5 // free
                        } else {
                            inverseSensorModel = 0.5 // occupied
                        }
                        grid[x][y] = grid[x][y] + inverseSensorModel // - prior which is 0
                        
                        var color = Int( 1 / ( 1 + exp( grid[x][y] ))  * 255)
                        image.setPixel(&color, atX: x, y: y)
                } else if(bFoundPointsInLine) {
                    break // go to next line
                }
            }
        }
        
        // TODO: update occgrid points that collide with robot to probability 0
    }
    
    let extent = 1000
    let scale : CGFloat = 5 // cm per pixel
    
    // occupancy grid in log odds representation
    var grid : [[Double]]
    
    // occupancy grid as 8-bit greyscale image
    // (0,0) is top-left, different from other Cocoa coordinate systems
    var image : NSBitmapImageRep
}

