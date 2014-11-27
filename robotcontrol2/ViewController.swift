//
//  ViewController.swift
//  robotcontrol2
//
//  Created by Sebastian Theophil on 22.11.14.
//  Copyright (c) 2014 Sebastian Theophil. All rights reserved.
//

import Cocoa
import CoreBluetooth

class DataModel : NSObject {
    override init() {}
}

class BLE : NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    override init() {
        super.init()
        centralmgr = CBCentralManager(delegate:self, queue:nil)
    }
    
    func scanForPeripherals() {
        println("start scanning")
        
        centralmgr.scanForPeripheralsWithServices(UUID_SERVICES, options:nil)
    }
    
    // CBCentralManagerDelegate interface
    func centralManagerDidUpdateState(central: CBCentralManager!) {
        // check central.state
        println("centralManagerDidUpdateState: \(central.state.rawValue)")
        
        if(central.state==CBCentralManagerState.PoweredOn) {
            scanForPeripherals()
        }
        // TODO: Disconnect
    }
    
    func centralManager(central: CBCentralManager!,
        didDiscoverPeripheral peripheral: CBPeripheral!,
        advertisementData: [NSObject : AnyObject]!,
        RSSI: NSNumber!)
    {
        println("Discovered peripheral \(peripheral.name)")
        
        peripheralActive = peripheral
        peripheralActive?.delegate = self
        centralmgr.connectPeripheral(peripheral, options: nil)
        
        centralmgr.stopScan()
    }
    
    func centralManager(central : CBCentralManager!,
        didConnectPeripheral peripheral : CBPeripheral!)
    {
        println("Connected peripheral \(peripheral.name)")        
        peripheralActive?.discoverServices(nil) // No need to filter services.
    }
    
    // CBPeripheralDelegate interface
    func peripheral(peripheral: CBPeripheral!,
        didDiscoverServices error: NSError!)
    {
        assert(error==nil)
        // assert(peripheralActive?.services.count==1)
        println("Discovered service")
        
        peripheralActive?.discoverCharacteristics(nil, forService: peripheralActive?.services[0] as CBService)
    }
    
    func peripheral(peripheral: CBPeripheral!,
        didDiscoverCharacteristicsForService service: CBService!,
        error: NSError!)
    {
        assert(error==nil)
        // assert(service.characteristics.count==2)
        println("Discovered characteristics")
        
        for characteristic in service.characteristics {
            
        }
    }
    
    var centralmgr : CBCentralManager!
    var peripheralActive : CBPeripheral?
    
    let UUID_SERVICES = [CBUUID(string:"713D0000-503E-4C75-BA94-3148F18D941E")]
}

class ViewController: NSViewController {
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        for i in 0 ..< 10 {
            textview.string? += "View Loaded\n"
            let v = btnMeasurement.state
            textview.string? += "Button is \(v)"
        }
        
        model = DataModel()
        viewRender.model = model
        
        ble = BLE()
    }

    override var representedObject: AnyObject? {
        didSet {
        // Update the view, if already loaded.
        }
    }
    
    var model : DataModel!;
    var ble : BLE!;
    
    @IBOutlet var viewRender: QuartzView!
    @IBOutlet var textview: NSTextView!
    @IBOutlet var btnMeasurement: NSButton!
}

