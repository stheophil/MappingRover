//
//  ViewController.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 22.11.14.
//  Copyright (c) 2014 Sebastian Theophil. All rights reserved.
//

import Cocoa

func sign<T : IntegerType>(x : T) -> Int {
    return x < 0 ? -1 : (0 < x ? 1 : 0)
}

protocol RobotController {
    // Robot control
    func moveForward()
    func moveBackward()
    func turnLeft()
    func turnRight()
    func sendCommand(cmd: SRobotCommand)
    
    // Robot sensors
    func receivedSensorData(data: SSensorData)
    func sensorData() -> [(SSensorData, CGPoint)]
    
    func path() -> NSBezierPath
    func occupancy() -> NSBitmapImageRep
    
    func log(msg: String)
}

let occupancy_dimension = 1000

// Robot configuration. Needs calibration.

let wheelRadius = CGFloat(3) // cm
// TODO: robot dimensions, sensor x-y offset
let sonarMaxDistance = CGFloat(300.0) // cm = Sonar max distance depends on mounting height
let sonarOpeningAngle = CGFloat(M_PI_2 / 6) // 15 degrees = Sensor opening angle
let sonarDistanceTolerance = CGFloat(5.0)

func encoderTicksToCm(ticks: Int16) -> CGFloat {
    return CGFloat(ticks) * CGFloat(6.0) * CGFloat(M_PI) * wheelRadius / 1000.0
}

func yawToRadians(yaw: Int16) -> CGFloat {
    return CGFloat(-yaw)/1000.0
}

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

class ViewController: NSViewController, RobotController {
    let maxTurnSpeed : Int16 = 200
    let maxFwdSpeed : Int16 = 300
    
    // View interface
    override func viewDidLoad() {
        super.viewDidLoad()
        
        viewRender.becomeFirstResponder()
        viewRender.controller = self
        
        ble = BLE(controller: self)
        
        bezierpath.moveToPoint(NSPoint(x: 0, y: 0))
        var color = 128
        for x in 0 ..< Int(occgridImage.size.width) {
            for y in 0 ..< Int(occgridImage.size.height) {
                occgridImage.setPixel(&color, atX: x, y: y)
            }
        }
        
        // NSTimer.scheduledTimerWithTimeInterval(2, target: self, selector: Selector("onTimer"), userInfo: nil, repeats: true)
        
        assert(sizeof(SSensorData)==18)
        assert(sizeof(SRobotCommand)==8)
    }
    
    /*
    var nYawPrev : Int16 = 0
    func onTimer() {
        nYawPrev += 300
        let data = SSensorData(m_nPitch: 0, m_nRoll: 0, m_nYaw: nYawPrev, m_nAngle: 90, m_nDistance: 200, m_anEncoderTicks: (400, 400, 400, 400))
        receivedSensorData(data)
    }
    */
    
    override func viewDidDisappear() {
        ble = nil
    }

    override var representedObject: AnyObject? {
        didSet {
        // Update the view, if already loaded.
        }
    }
    
    // RobotController interface
    func sendCommand(var cmd: SRobotCommand) {
        ble.sendControlCommand(NSData(bytes:&cmd, length:sizeof(SRobotCommand)))
    }
    
    func moveForward() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: maxFwdSpeed, m_nSpeedRight: maxFwdSpeed))
    }
    
    func moveBackward() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: -maxFwdSpeed, m_nSpeedRight: -maxFwdSpeed))
    }
    
    func turnLeft() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: -maxTurnSpeed, m_nSpeedRight: maxTurnSpeed))
    }
    
    func turnRight() {
        sendCommand(SRobotCommand(m_cmd: ecmdMOVE, m_nSpeedLeft: maxTurnSpeed, m_nSpeedRight: -maxTurnSpeed))
    }
    
    func receivedSensorData(data: SSensorData) {
        log("Sensor data Z: \(data.m_nYaw) Sonar: \(data.m_nDistance) @ \(data.m_nAngle) Distances: \(data.m_anEncoderTicks.0), \(data.m_anEncoderTicks.1), \(data.m_anEncoderTicks.2), \(data.m_anEncoderTicks.3)\n")
        
        // left motors are 0 and 2, right motors 1, 3
        // assert( sign(data.m_anEncoderTicks.0) == sign(data.m_anEncoderTicks.2) )
        // assert( sign(data.m_anEncoderTicks.1) == sign(data.m_anEncoderTicks.3) )
        
        let ptPrev = sensorDataArray.last?.1 ?? CGPointZero
        var pt : CGPoint
        if sign(data.m_anEncoderTicks.0) != sign(data.m_anEncoderTicks.1) { // turn
            // position does not change
            pt = ptPrev
        } else {
            pt = toPoint(yawToRadians(data.m_nYaw), encoderTicksToCm(data.m_anEncoderTicks.0))
            pt.x += ptPrev.x
            pt.y += ptPrev.y
            
            bezierpath.lineToPoint(pt)
        }
        sensorDataArray.append((data, pt))
        viewRender.needsDisplay = true
        
        // updateOccupancyGrid(pt, fYaw: yawToRadians(data.m_nYaw), nAngle: data.m_nAngle, nDistance: data.m_nDistance)
    }
    
    func updateOccupancyGrid(pt: CGPoint, fYaw: CGFloat, nAngle : Int16, nDistance : Int16 ) {
        // update occgrid & occgridImage
        assert(nAngle==0 || abs(nAngle)==90)
        let angleSonar = CGFloat(fYaw) + CGFloat(M_PI_2) * CGFloat(sign(nAngle))
        
        let gridOffset = CGVector(dx: occupancy_dimension/2, dy: occupancy_dimension/2)
        let sonarPosition = pt + gridOffset // TODO: Sonar offset relative to robot center pt
        
        // approximate line scan bound rect
        let bounds = boundRect(
            sonarPosition,
            toPoint(angleSonar - sonarOpeningAngle/2, sonarMaxDistance) + sonarPosition,
            toPoint(angleSonar, sonarMaxDistance) + sonarPosition,
            toPoint(angleSonar + sonarOpeningAngle/2, sonarMaxDistance) + sonarPosition
            ).integerRect // bound rect in integer coordinates
        
        // log("Sensor bound rect: (\(bounds.minX), \(bounds.minY), \(bounds.maxX), \(bounds.maxY))\n")
        
        // TODO: Resize occupancy grids
        assert(NSContainsRect(
            NSRect(x: 0, y: 0, width: occupancy_dimension, height: occupancy_dimension),
            bounds
            ))
        
        for y in Int(bounds.minY) ... Int(bounds.maxY) {
            var bFoundPointsInLine = false
            for x in Int(bounds.minX) ... Int(bounds.maxX) {
                // distance and angle of vector from sonar sensor to point (x, y)
                let distance = sonarPosition.distance(CGPoint(x: x, y: y))
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
                        occgrid[x][y] = occgrid[x][y] + inverseSensorModel // - prior which is 0
                        
                        var color = Int( 1 / ( 1 + exp( occgrid[x][y] ))  * 255)
                        occgridImage.setPixel(&color, atX: x, y: y)
                } else if(bFoundPointsInLine) {
                    break // go to next line
                }
            }
        }
        
        // TODO: update occgrid points that collide with robot to probability 0
    }
    
    func sensorData() -> [(SSensorData, CGPoint)] {
        return sensorDataArray
    }
    
    func path() -> NSBezierPath {
        return bezierpath
    }
    
    func occupancy() -> NSBitmapImageRep {
        return occgridImage
    }
    
    func log(msg: String) {
        print(msg)
        // textview.string? += msg
        // textview.scrollRangeToVisible(NSMakeRange(textview.string!.lengthOfBytesUsingEncoding(NSUTF8StringEncoding), 0))
    }
    
    var sensorDataArray = [(SSensorData, CGPoint)]() // sensor data including accumulated position
    var bezierpath = NSBezierPath()
    
    let scaleGrid = 5 // cm per pixel
    
    // occupancy grid in log odds representation
    var occgrid = [[Double]](count: occupancy_dimension,
        repeatedValue: [Double](count: occupancy_dimension, repeatedValue: 0.0))
    
    // occupancy grid as 8-bit greyscale image
    // (0,0) is top-left, different from other Cocoa coordinate systems
    var occgridImage = NSBitmapImageRep(bitmapDataPlanes: nil,
        pixelsWide: occupancy_dimension, pixelsHigh: occupancy_dimension,
        bitsPerSample: 8,
        samplesPerPixel: 1,
        hasAlpha: false,
        isPlanar: true,
        colorSpaceName: NSDeviceWhiteColorSpace,
        bitmapFormat: NSBitmapFormat.allZeros,
        bytesPerRow: 0, bitsPerPixel: 0)!
    
    var ble : BLE!;
    
    @IBOutlet var viewRender: QuartzView!
    @IBOutlet var textview: NSTextView!
    @IBOutlet var btnMeasurement: NSButton!
}

