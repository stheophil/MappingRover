//
//  Occupancy.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 20.02.15.
//  Copyright (c) 2015 Sebastian Theophil. All rights reserved.
//

import Foundation
import Cocoa

protocol ScalableOffsetable {
    func +(left: Self, right: CGVector) -> Self
    func -(left: Self, right: CGVector) -> Self
    
    func *(left: Self, right: CGFloat) -> Self
    func /(left: Self, right: CGFloat) -> Self
}

// Geometry extensions
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

// CGPoint operators
func + (left: CGPoint, right: CGVector) -> CGPoint {
    return CGPointMake(left.x + right.dx, left.y + right.dy)
}

func -(left: CGPoint, right: CGVector) -> CGPoint {
    return CGPointMake(left.x - right.dx, left.y - right.dy)
}

func *(left: CGPoint, right: CGFloat) -> CGPoint {
    return CGPointMake(left.x * right, left.y * right)
}

func /(left: CGPoint, right: CGFloat) -> CGPoint {
    return CGPointMake(left.x / right, left.y / right)
}

func -(left: CGPoint, right: CGPoint) -> CGVector {
    return CGVector(dx: left.x - right.x, dy: left.y - right.y)
}


// CGRect operators
func +(left: CGRect, right: CGVector) -> CGRect {
    return left.rectByOffsetting(dx: right.dx, dy: right.dy)
}

func -(left: CGRect, right: CGVector) -> CGRect {
    return left.rectByOffsetting(dx: -right.dx, dy: -right.dy)
}

func *(left: CGRect, right: CGFloat) -> CGRect {
    return CGRect(origin: left.origin*right, size: left.size*right)
}

func /(left: CGRect, right: CGFloat) -> CGRect {
    return CGRect(origin: left.origin/right, size: left.size/right)
}

func |=(left: CGRect, right: CGPoint) -> CGRect {
    let origin = CGPoint(
        x: min(left.minX, right.x),
        y: min(left.minY, right.y))
    
    let size = CGSize(
        width: max(left.width, right.x - origin.x),
        height: max(left.height, right.y - origin.y)
    )
    return CGRect(origin: origin, size: size)
}

// CGSize operators
func *(left: CGSize, right: CGFloat) -> CGSize {
    return CGSize(width: left.width * right, height: left.height * right)
}

func /(left: CGSize, right: CGFloat) -> CGSize {
    return CGSize(width: left.width / right, height: left.height / right)
}

// CGVector operators
func *(left: CGVector, right: CGFloat) -> CGVector {
    return CGVector(dx: left.dx * right, dy: left.dy * right)
}

func /(left: CGVector, right: CGFloat) -> CGVector {
    return CGVector(dx: left.dx / right, dy: left.dy / right)
}

extension CGVector {
    init(_ pt: CGPoint) {
        dx = pt.x
        dy = pt.y
    }
    
    init(_ sz: CGSize) {
        dx = sz.width
        dy = sz.height
    }
}

extension CGSize {
    init(_ v: CGVector) {
        width = v.dx
        height = v.dy
    }
}
    
extension CGPoint : ScalableOffsetable {
    init(fromAngle angle: CGFloat, distance: CGFloat) {
        x = distance * cos(angle)
        y = distance * sin(angle)
    }

    func distance(pt : CGPoint) -> CGFloat {
        return hypot(x - pt.x, y - pt.y)
    }
    
    func SqrAbs() -> CGFloat {
        return x*x + y*y
    }
    
    func quadrant() -> Int {
        return x < 0
            ? (y < 0 ? 2 : 1)
            : (y < 0 ? 3 : 0)
    }
    
    // < 0 -> v is left of this
    // > 0 -> v is right of this
    func compare(v: CGPoint) -> Int {
        let d = y * v.x - x * v.y
        return sign(d)
    }
}

extension CGRect : ScalableOffsetable {
    init(fromPoints points: CGPoint...) {
        let bottomLeft = CGPoint(
            x: minElement(points.map{ $0.x }),
            y: minElement(points.map{ $0.y })
        )
        let topRight = CGPoint(
            x: maxElement(points.map{ $0.x }),
            y: maxElement(points.map{ $0.y })
        )
        origin = bottomLeft
        size = CGSize(topRight - bottomLeft)
    }
    
    init(centeredAt pt: CGPoint, size: CGSize) {
        self.origin = pt - CGVector(size/2)
        self.size = size
    }
}

func sign<T : SignedNumberType>(x : T) -> Int {
    return x < 0 ? -1 : (0 < x ? 1 : 0)
}

struct SArc {
    var ptCenter: CGPoint
    var angleFrom: CGFloat
    var angleTo: CGFloat
    var radius: CGFloat
    
    init(_ pt: CGPoint, fromAngle: CGFloat, toAngle: CGFloat, withRadius: CGFloat) {
        self.ptCenter = pt
        self.angleFrom = fromAngle
        self.angleTo = toAngle
        self.radius = withRadius
    }
    
    func forEachPoint(foreach : (Int, Int, CGFloat) -> Void) {
        var ptFrom =  CGPoint(fromAngle: angleFrom, distance: radius) // relative to ptCenter
        var ptTo =  CGPoint(fromAngle: angleTo, distance: radius)
        
        assert(angularDistance(angleFrom, angleTo)<CGFloat(M_PI/2))
        assert(ptFrom.compare(ptTo) != 0)
        
        if ptFrom.compare(ptTo) < 0 { // ptFrom should be left of ptTo
            swap(&ptFrom, &ptTo)
        }
        
        let nQuadrantFrom = ptFrom.quadrant()
        let nQuadrantTo = ptTo.quadrant()
        
        assert(nQuadrantFrom==nQuadrantTo || nQuadrantFrom==(nQuadrantTo+1)%4)
        
        var rectBound = CGRect(fromPoints: ptFrom, ptTo, CGPoint(x: 0, y: 0))
        if nQuadrantFrom != nQuadrantTo {
            switch nQuadrantFrom {
            case 0: rectBound |= CGPoint(x: radius, y: 0)
            case 1: rectBound |= CGPoint(x: 0, y: radius)
            case 2: rectBound |= CGPoint(x: -radius, y: 0)
            case 3: rectBound |= CGPoint(x: 0, y: -radius)
            default: assert(false)
            }
        }
        
        rectBound.integerize()
        let fSqrRadius = CGFloat(radius * radius)
        for y in Int(rectBound.minY) ... Int(rectBound.maxY) {
            var bFoundPointsInLine = false
            for x in Int(rectBound.minX) ... Int(rectBound.maxX) {
                let pt = CGPoint(x: x, y: y)
                let fSqrDistance = pt.SqrAbs()
                
                if fSqrDistance < fSqrRadius
                && 0<ptFrom.compare(pt)
                && 0<pt.compare(ptTo) {
                    bFoundPointsInLine = true
               
                    foreach(x + Int(ptCenter.x), y + Int(ptCenter.y), fSqrDistance)
                    
                } else if bFoundPointsInLine {
                    break // go to next line
                }
            }
        }
    }
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
        let fAngleSonar = CGFloat(fYaw) + CGFloat(M_PI_2) * CGFloat(sign(nAngle))
        
        let arc = SArc(transform(pt),
            fromAngle: fAngleSonar - sonarOpeningAngle/2,
            toAngle: fAngleSonar + sonarOpeningAngle/2,
            withRadius: (CGFloat(nDistance) + sonarDistanceTolerance/2)/scale)
        
        let fSqrMaxDistance = pow(sonarMaxDistance/scale, 2)
        let fSqrMeasuredDistance = pow((CGFloat(nDistance) - sonarDistanceTolerance/2)/scale, 2)
        
        arc.forEachPoint({ (x: Int, y: Int, fSqrDistance: CGFloat) -> Void in
            if fSqrDistance < fSqrMaxDistance {
                let fInverseSensorModel = fSqrDistance < fSqrMeasuredDistance
                    ? -0.5 // free
                    : 100.0 / sqrt(Double(fSqrDistance)) // occupied
                self.setGrid(x, y, self.grid[x][y] + fInverseSensorModel) // - prior which is 0
            }
        })
    }
    
    func setGrid(x: Int, _ y: Int, _ value: Double) {
        grid[x][y] = value
        var color = Int( 1 / ( 1 + exp( value ))  * 255)
        image.setPixel(&color, atX: x, y: y)
    }
    
    func clear(rect: CGRect) {
        let rectGrid = transform(rect)
        for y in Int(rectGrid.minY) ... Int(rectGrid.maxY) {
            for x in Int(rectGrid.minX) ... Int(rectGrid.maxX) {
                setGrid( x, y, 0.0)
            }
        }
    }
    
    func draw() {
        let sizeImage = image.size * scale
        image.drawInRect(NSRect(
            x: -sizeImage.width/2,
            y: sizeImage.height/2,
            width: sizeImage.width,
            height: -sizeImage.height
            ))
    }
    
    private func transform<T: ScalableOffsetable>(geom: T) -> T {
        let gridOffset = CGVector(dx: extent/2, dy: extent/2)
        return (geom / scale) + gridOffset
    }
    
    let extent = 1000
    let scale : CGFloat = 5 // cm per pixel
    
    // occupancy grid in log odds representation
    var grid : [[Double]]
    
    // occupancy grid as 8-bit greyscale image
    // (0,0) is top-left, different from other Cocoa coordinate systems
    var image : NSBitmapImageRep
}

