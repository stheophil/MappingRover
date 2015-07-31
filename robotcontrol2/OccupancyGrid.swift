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
    
    func rotated(fAngle: CGFloat) -> CGPoint {
        let c = cos(fAngle)
        let s = sin(fAngle)
        return CGPoint(x: x * c - y * s, y: x * s + y * c)
    }
    
    // < 0 -> v is left of this
    // > 0 -> v is right of this
    func compare(v: CGPoint) -> Int {
        let d = y * v.x - x * v.y
        return sign(d)
    }
}

func RectFromPoints(points: [CGPoint]) -> (CGPoint, CGSize) {
    let bottomLeft = CGPoint(
        x: minElement(points.map{ $0.x }),
        y: minElement(points.map{ $0.y })
    )
    let topRight = CGPoint(
        x: maxElement(points.map{ $0.x }),
        y: maxElement(points.map{ $0.y })
    )
    return (bottomLeft, CGSize(topRight - bottomLeft))
}

extension CGRect : ScalableOffsetable {
    init(fromPoints points: CGPoint...) {
        (origin, size) = RectFromPoints(points)
    }
    
    init(fromPoints points: [CGPoint]) {
        (origin, size) = RectFromPoints(points)
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
        
        let fSqrRadius = CGFloat(radius * radius)
        for y in Int(floor(rectBound.minY)) ... Int(floor(rectBound.maxY)) {
            var bFoundPointsInLine = false
            for x in Int(floor(rectBound.minX)) ... Int(floor(rectBound.maxX)) {
                let pt = CGPoint(x: x, y: y)
                let fSqrDistance = pt.SqrAbs()
                
                if fSqrDistance < fSqrRadius
                && 0<=ptFrom.compare(pt) // this is very strict and does not
                && 0<=pt.compare(ptTo) { // count grid cells partially inside arc
                    bFoundPointsInLine = true
               
                    foreach(x + Int(floor(ptCenter.x)), y + Int(floor(ptCenter.y)), fSqrDistance)
                    
                } else if bFoundPointsInLine {
                    break // go to next line
                }
            }
        }
    }
}

struct SRotatedRect {
    var ptCenter : CGPoint
    var size : CGSize
    var fAngle : CGFloat
    
    func forEachPoint(foreach : (Int, Int) -> Void) {
        let apt = [
            CGPoint(x: -size.width/2, y: -size.height/2).rotated(fAngle) + CGVector(ptCenter),
            CGPoint(x: size.width/2, y: -size.height/2).rotated(fAngle) + CGVector(ptCenter),
            CGPoint(x: size.width/2, y: size.height/2).rotated(fAngle) + CGVector(ptCenter),
            CGPoint(x: -size.width/2, y: size.height/2).rotated(fAngle) + CGVector(ptCenter)
        ]
        
        func rasterize(var ptA: CGPoint, var ptB: CGPoint, f : (Int, Int) -> Void) {
            if(ptB.x<ptA.x) {
                swap(&ptB, &ptA)
            }
            
            // Always round down instead of rounding to nearest
            // if ptA.x == 10.6, it occupies grid cell 10
            let nBegin = Int(floor(ptA.x))
            let nEnd = Int(floor(ptB.x))
            
            if(nBegin == nEnd) {
                // straight vertical line
                let nBeginY = Int( floor(min(ptA.y, ptB.y)) )
                let nEndY = Int( floor(max(ptA.y, ptB.y)) )
                for y in nBeginY ... nEndY {
                    f(nBegin, y)
                }
            } else {
                let m : CGFloat = (ptB.y - ptA.y) / (ptB.x - ptA.x)
                if(abs(m)<=1) { // x-step
                    for x in nBegin ... nEnd {
                        f(x, Int(floor(ptA.y + m * CGFloat(x - nBegin))))
                    }
                } else { // y-step
                    if(ptB.y < ptA.y) {
                        swap(&ptB, &ptA);
                    }
                    let nBeginY = Int( floor(ptA.y) )
                    let nEndY = Int( floor(ptB.y) )
                    
                    for y in nBeginY ... nEndY {
                        f(Int(floor(ptA.x + CGFloat(y - nBeginY) / m)), y)
                    }
                }
            }
        }
        
        // TODO: The map could be avoided by sorting the line segments
        var dictMinMaxX = [Int: (Int, Int)]()
        for i in 0 ..< apt.count {
            rasterize(apt[i], apt[(i+1)%apt.count], { (x: Int, y: Int) -> Void in
                if let pairMinMax = dictMinMaxX[y] {
                    dictMinMaxX[y] = (min(pairMinMax.0, x), max(pairMinMax.1, x))
                } else {
                    dictMinMaxX[y] = (x, x)
                }
            })
        }
        
        for (y, pairMinMaxX) in dictMinMaxX {
            for x in pairMinMaxX.0 ... pairMinMaxX.1 {
                foreach(x, y)
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
        
        // Clear position of robot itself
        let rectRobot = SRotatedRect(ptCenter: transform(pt), size: robotSize/scale, fAngle: fYaw)
        rectRobot.forEachPoint({ (x: Int, y:Int) -> Void in
            self.setGrid(x, y, -100)
        })
    }
    
    func setGrid(x: Int, _ y: Int, _ value: Double) {
        grid[x][y] = value
        var color = Int(round(1 / ( 1 + exp( value ))  * 255))
        image.setPixel(&color, atX: x, y: y)
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

