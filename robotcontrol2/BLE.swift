import CoreBluetooth

class BLE : NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    init(controller: RobotController) {
        self.controller = controller
        
        super.init()
        centralmgr = CBCentralManager(delegate:self, queue:nil)
    }
    
    deinit {
        if let p = peripheralActive {
            centralmgr.cancelPeripheralConnection(p)

            // TODO: Is this explicitly necessary?
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
                
                peripheralActive!.setNotifyValue(true, forCharacteristic: charRX)
            }
        }
        assert(initialized())
    }
    
    func peripheral(peripheral: CBPeripheral!, didUpdateNotificationStateForCharacteristic characteristic: CBCharacteristic!, error: NSError!)
    {
        println("Subscribing to Characteristics succeeded")
        assert(error==nil)
    }
    
    func peripheral(peripheral: CBPeripheral!, didUpdateValueForCharacteristic characteristic: CBCharacteristic!, error: NSError!)
    {
        assert(error==nil)
        var data = charRX.value()
        if(data.length==sizeof(SSensorData)) {
            controller.receivedSensorData(UnsafePointer<SSensorData>(data.bytes).memory)
        }
    }
    
    // Data interface
    func initialized() -> Bool {
        return peripheralActive != nil
            && charTX != nil
            && charRX != nil
    }
    
    func sendControlCommand(data : NSData) {
        if initialized() {
            peripheralActive!.writeValue(
                data,
                forCharacteristic: charTX,
                type: CBCharacteristicWriteType.WithoutResponse)
        }
    }
    
    var controller : RobotController // TODO: Can't set to weak var
    
    var centralmgr : CBCentralManager!
    var peripheralActive : CBPeripheral?
    
    var charRX : CBCharacteristic!
    var charTX : CBCharacteristic!
    
    let UUID_SERVICES = [CBUUID(string:"713D0000-503E-4C75-BA94-3148F18D941E")]
    let UUID_RX_CHARACTERISTICS = CBUUID(string:"713D0002-503E-4C75-BA94-3148F18D941E")
    let UUID_TX_CHARACTERISTICS = CBUUID(string:"713D0003-503E-4C75-BA94-3148F18D941E")
}