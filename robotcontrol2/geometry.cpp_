//
//  Utilities.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 21.08.15.
//  Copyright (c) 2015 Sebastian Theophil. All rights reserved.
//

import Foundation

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
    return left.offsetBy(dx: right.dx, dy: right.dy)
}

func -(left: CGRect, right: CGVector) -> CGRect {
    return left.offsetBy(dx: -right.dx, dy: -right.dy)
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
        x: points.map{ $0.x }.minElement()!,
        y: points.map{ $0.y }.minElement()!
    )
    let topRight = CGPoint(
        x: points.map{ $0.x }.maxElement()!,
        y: points.map{ $0.y }.maxElement()!
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
