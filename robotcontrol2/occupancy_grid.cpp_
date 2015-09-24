//
//  Occupancy.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 20.02.15.
//  Copyright (c) 2015 Sebastian Theophil. All rights reserved.
//

import Foundation
import Cocoa
import Accelerate

@inline(__always) func sign<T : SignedNumberType>(x : T) -> Int {
    return x < 0 ? -1 : (0 < x ? 1 : 0)
}

@inline(__always) func sqr<T : IntegerArithmeticType>(x : T) -> T {
    return x * x
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
        
        assert(angularDistance(angleFrom, angleB: angleTo)<CGFloat(M_PI/2))
        assert(ptFrom.compare(ptTo) != 0)
        
        if ptFrom.compare(ptTo) < 0 { // ptFrom should be left of ptTo
            swap(&ptFrom, &ptTo)
        }
        
        let nQuadrantFrom = ptFrom.quadrant()
        let nQuadrantTo = ptTo.quadrant()
        
        assert(nQuadrantFrom==nQuadrantTo || nQuadrantFrom==(nQuadrantTo+1)%4)
        
        let rectBound = CGRect(fromPoints: ptFrom, ptTo, CGPoint(x: 0, y: 0))
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
        for y in Int(round(rectBound.minY)) ... Int(round(rectBound.maxY)) {
            var bFoundPointsInLine = false
            for x in Int(round(rectBound.minX)) ... Int(round(rectBound.maxX)) {
                let pt = CGPoint(x: x, y: y)
                let fSqrDistance = pt.SqrAbs()
                
                if fSqrDistance < fSqrRadius
                && 0<=ptFrom.compare(pt) // this is very strict and does not
                && 0<=pt.compare(ptTo) { // count grid cells partially inside arc
                    bFoundPointsInLine = true
               
                    foreach(x + Int(round(ptCenter.x)), y + Int(round(ptCenter.y)), fSqrDistance)
                    
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
            
            let nBegin = Int(round(ptA.x))
            let nEnd = Int(round(ptB.x))
            
            if(nBegin == nEnd) {
                // straight vertical line
                let nBeginY = Int( round(min(ptA.y, ptB.y)) )
                let nEndY = Int( round(max(ptA.y, ptB.y)) )
                for y in nBeginY ... nEndY {
                    f(nBegin, y)
                }
            } else {
                let m : CGFloat = (ptB.y - ptA.y) / (ptB.x - ptA.x)
                if(abs(m)<=1) { // x-step
                    for x in nBegin ... nEnd {
                        f(x, Int(round(ptA.y + m * CGFloat(x - nBegin))))
                    }
                } else { // y-step
                    if(ptB.y < ptA.y) {
                        swap(&ptB, &ptA);
                    }
                    let nBeginY = Int( round(ptA.y) )
                    let nEndY = Int( round(ptB.y) )
                    
                    for y in nBeginY ... nEndY {
                        f(Int(round(ptA.x + CGFloat(y - nBeginY) / m)), y)
                    }
                }
            }
        }
        
        // TODO: The map could be avoided by sorting the line segments
        var dictMinMaxX = [Int: (Int, Int)]()
        for i in 0 ..< apt.count {
            rasterize(apt[i], ptB: apt[(i+1)%apt.count], f: { (x: Int, y: Int) -> Void in
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

struct SBitmapData {
    var m_bitmapData : UnsafeMutablePointer<UInt8>
    var m_cbBytesPerRow : Int
    init(_ image: NSBitmapImageRep) {
        m_bitmapData = image.bitmapData
        m_cbBytesPerRow = image.bytesPerRow
    }
    
    func color(x: Int, _ y: Int) -> UInt8 {
        return m_bitmapData[y * m_cbBytesPerRow + x]
    }
    
    func setColor(x: Int, _ y: Int, var _ color: UInt8 ) {
        let p = m_bitmapData.advancedBy(y * m_cbBytesPerRow + x)
        p.assignFrom(&color, count: 1)
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
            bitmapFormat: NSBitmapFormat(),
            bytesPerRow: 0, bitsPerPixel: 0)!

        imageEroded = image.copy() as! NSBitmapImageRep
        
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
        
        let bitmap = SBitmapData(image)
        let UpdateGrid = {(x: Int, y: Int, value: Double) -> Void in
            self.grid[x][y] = value
            let color = UInt8(round(1 / ( 1 + exp( value ))  * 255))
            bitmap.setColor(x, y, color)
        }
        
        arc.forEachPoint({ (x: Int, y: Int, fSqrDistance: CGFloat) -> Void in
            if fSqrDistance < fSqrMaxDistance {
                let fInverseSensorModel = fSqrDistance < fSqrMeasuredDistance
                    ? -0.5 // free
                    : Double(100.0 / self.scale) / sqrt(Double(fSqrDistance)) // occupied
                UpdateGrid(x, y, self.grid[x][y] + fInverseSensorModel) // - prior which is 0
            }
        })
        
        // Clear position of robot itself
        let rectRobot = SRotatedRect(ptCenter: transform(pt), size: sizeRobot/scale, fAngle: fYaw)
        rectRobot.forEachPoint({ (x: Int, y:Int) -> Void in
            UpdateGrid(x, y, -100)
        })
        
        // Erode image
        // A pixel p in imageEroded is marked free when the robot centered at p does not occupy an occupied pixel in self.image
        // i.e. the pixel p has the maximum value of the surrounding pixels inside the diameter defined by the robot's size
        // We overestimate robot size by taking robot diagonal 
        let nKernelDiameter = UInt( ceil( sqrt( pow(sizeRobot.width, 2) + pow(sizeRobot.height, 2) ) / scale ) )
        let anKernel = [UInt8](count: Int(nKernelDiameter * nKernelDiameter), repeatedValue: 0)
        var vimgbufInput = vImage_Buffer(data: image.bitmapData, height: UInt(image.pixelsHigh), width: UInt(image.pixelsWide), rowBytes: image.bytesPerRow)
        var vimgbufOutput = vImage_Buffer(data: imageEroded.bitmapData, height: UInt(image.pixelsHigh), width: UInt(image.pixelsWide), rowBytes: image.bytesPerRow)
        
        vImageErode_Planar8( &vimgbufInput, &vimgbufOutput, 0, 0, anKernel, nKernelDiameter, nKernelDiameter, UInt32(kvImageNoFlags) )
    }
    
    func draw(bShowOriginalMap : Bool) {
        let sizeImage = image.size * scale
        (bShowOriginalMap ? image : imageEroded).drawInRect(NSRect(
            x: -sizeImage.width/2,
            y: sizeImage.height/2,
            width: sizeImage.width,
            height: -sizeImage.height
        ))
    }
    
    func closestUnknownPoint(ptCenter: CGPoint, fYaw: CGFloat) -> [CGPoint] {
        let intvlUnknown = (UInt8(0.4 * 255), UInt8(0.6 * 255))
        
        let ptCenterTransformed = transform(ptCenter)
        let ptnCenter = (Int(round(ptCenterTransformed.x)), Int(round(ptCenterTransformed.y)))
        // Convert fYaw (CCW) to C dir enum (see find_path.h): left = 0, up = 1, right = 2, down = 3
        let dirInitial = (-Int32(round(Double(fYaw) * 2 / M_PI)) + 2)%4
        
        var ptnMin = ptnCenter
        var nSqrMinDistance = Int.max
        
        // nDistanceMax is inclusive
        var nDistanceMax = max(extent - ptnCenter.0 - 1, ptnCenter.0,
            extent - ptnCenter.1 - 1, ptnCenter.1)
        
        let bitmap = SBitmapData(image)
        
        let UpdateClosest = { (x: Int, y: Int, nDistance: Int) -> Void in
            let nColor = bitmap.color(x, y)
            if intvlUnknown.0 <= nColor && nColor < intvlUnknown.1 {
                let nSqrDistance = sqr(x - ptnCenter.0) + sqr(y - ptnCenter.1)
                if nSqrDistance < nSqrMinDistance {
                    // Point x, y is on the boundary of a rectangle around ptCenter
                    // When we find a new closest point, calculate the size of the largest rectangle
                    // that contains all points that may be closer than x,y
                    nDistanceMax = Int(ceil(sqrt(CGFloat(nSqrDistance)) / CGFloat(nDistance)))
                    
                    ptnMin = (x, y)
                    nSqrMinDistance = nSqrDistance
                }
            }
        }
        
        for nDistance in 1 ... nDistanceMax {
            let intvlnX = (max(0, ptnCenter.0-nDistance), min(extent-1, ptnCenter.0+nDistance))
            for x in intvlnX.0 ... intvlnX.1 {
                if 0 <= ptnCenter.1 - nDistance {
                    UpdateClosest(x, ptnCenter.1 - nDistance, nDistance)
                }
                if ptnCenter.1 + nDistance < extent {
                    UpdateClosest(x, ptnCenter.1 + nDistance, nDistance)
                }
            }
            
            let intvlnY = (max(0, ptnCenter.1-nDistance), min(extent-1, ptnCenter.1+nDistance))
            for y in intvlnY.0 ... intvlnY.1 {
                if 0 <= ptnCenter.0 - nDistance {
                    UpdateClosest(ptnCenter.0 - nDistance, y, nDistance)
                }
                if ptnCenter.0 + nDistance < extent {
                    UpdateClosest(ptnCenter.0 + nDistance, y, nDistance)
                }
            }
        }
        
        // Assume ptnMin is reachable
        // TODO: Search path on eroded map
        var cptnPath = [CGPoint]()
        find_path(
            Int32(ptnCenter.0), Int32(ptnCenter.1),
            dirInitial,
            Int32(ptnMin.0), Int32(ptnMin.1),
            bitmap.m_bitmapData, UInt32(bitmap.m_cbBytesPerRow), Int32(extent),
            {(x: Int32, y: Int32) -> Void in
                cptnPath.append( self.inverseTransform(CGPoint(x: Int(x), y: Int(y))) )
            }
        )
        return cptnPath
    }
    
    private func transform<T: ScalableOffsetable>(geom: T) -> T { // to pixel coordinates
        let gridOffset = CGVector(dx: extent/2, dy: extent/2)
        return (geom / scale) + gridOffset
    }
    
    private func inverseTransform<T: ScalableOffsetable>(geom: T) -> T { // from pixel coordinate
        let gridOffset = CGVector(dx: extent/2, dy: extent/2)
        return (geom - gridOffset) * scale
    }
    
    // When transforming coordinates to grid coordinates, always round to nearest
    // i.e., the grid cell (0,0) is centered at point (0,0)
    let extent = 1000 // grid size in pixel, even number
    let scale : CGFloat = 5 // cm per pixel
    
    // occupancy grid in log odds representation
    var grid : [[Double]]
    
    // occupancy grid as 8-bit greyscale image
    // (0,0) is top-left, different from other Cocoa coordinate systems
    var image : NSBitmapImageRep
    var imageEroded : NSBitmapImageRep
}

