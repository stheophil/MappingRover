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
    
    deinit {
        if let p = peripheralActive {
            centralmgr.cancelPeripheralConnection(p)
            // Is this explicitly necessary?
            charRX = nil
            charTX = nil
            peripheralActive = nil
            centralmgr = nil
        }
    }
    
    func scanForPeripherals() {
        println("start scanning")
        
        centralmgr.scanForPeripheralsWithServices(nil, options:nil)
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
        peripheralActive!.delegate = self
        centralmgr.connectPeripheral(peripheral, options: nil)
        
        centralmgr.stopScan()
    }
    
    func centralManager(central : CBCentralManager!,
        didConnectPeripheral peripheral : CBPeripheral!)
    {
        println("Connected peripheral \(peripheral.name)")
        peripheralActive!.discoverServices(UUID_SERVICES)
    }
    
    // CBPeripheralDelegate interface
    func peripheral(peripheral: CBPeripheral!,
        didDiscoverServices error: NSError!)
    {
        assert(error==nil)
        assert(peripheralActive!.services.count==1)
        
        for service in peripheralActive!.services {
            println("Discovered service \((service as CBService).UUID.UUIDString)")
            peripheralActive!.discoverCharacteristics(nil, forService: service as CBService)
        }
    }
    
    func peripheral(peripheral: CBPeripheral!,
        didDiscoverCharacteristicsForService service: CBService!,
        error: NSError!)
    {
        assert(error==nil)
        assert(service.characteristics.count==2)
        for characteristic in service.characteristics {
            println("Discovered characteristic \((characteristic as CBCharacteristic).UUID.UUIDString)")
            if (characteristic as CBCharacteristic).UUID.UUIDString == UUID_TX_CHARACTERISTICS.UUIDString {
                println("TX Characteristics found")
                charTX = characteristic as CBCharacteristic
            } else {
                assert( (characteristic as CBCharacteristic).UUID.UUIDString == UUID_RX_CHARACTERISTICS.UUIDString )
                println("RX Characteristics found")
                charRX = characteristic as CBCharacteristic
            }
        }
        assert(initialized())
    }
    
    // Data interface
    func initialized() -> Bool {
        return peripheralActive != nil
            && charTX != nil
            && charRX != nil
    }
    
    func sendControlCommand() {
        assert(initialized())
        
        let str = "U"
        peripheralActive!.writeValue(str.dataUsingEncoding(NSUTF8StringEncoding),
            forCharacteristic: charTX,
            type: CBCharacteristicWriteType.WithoutResponse)
    }
    
    var centralmgr : CBCentralManager!
    var peripheralActive : CBPeripheral?
    
    var charRX : CBCharacteristic!
    var charTX : CBCharacteristic!
    
    let UUID_SERVICES = [CBUUID(string:"713D0000-503E-4C75-BA94-3148F18D941E")]
    let UUID_RX_CHARACTERISTICS = CBUUID(string:"713D0002-503E-4C75-BA94-3148F18D941E")
    let UUID_TX_CHARACTERISTICS = CBUUID(string:"713D0003-503E-4C75-BA94-3148F18D941E")
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
        
        viewRender.becomeFirstResponder()
        
        model = DataModel()
        viewRender.model = model
        
        ble = BLE()
    }
    
    override func viewDidDisappear() {
        ble = nil
        model = nil
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

