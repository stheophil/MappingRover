import CoreBluetooth

class BLE : NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    init(controller: RobotViewController) {
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
        print("start scanning")
        
        centralmgr.scanForPeripheralsWithServices(nil, options:nil)
    }
    
    // CBCentralManagerDelegate interface
    func centralManagerDidUpdateState(central: CBCentralManager) {
        // check central.state
        print("centralManagerDidUpdateState: \(central.state.rawValue)")
        
        if(central.state==CBCentralManagerState.PoweredOn) {
            scanForPeripherals()
        }
        // TODO: Disconnect
    }
    
    func centralManager(central: CBCentralManager,
        didDiscoverPeripheral peripheral: CBPeripheral,
        advertisementData: [String : AnyObject],
        RSSI: NSNumber)
    {
        if let name = peripheral.name {
            print("Discovered peripheral \(peripheral.name)")
            if name ==  "rcontrol2" {
                peripheralActive = peripheral
                peripheralActive!.delegate = self
                centralmgr.connectPeripheral(peripheral, options: nil)
                
                centralmgr.stopScan()
            }
        }
    }
    
    func centralManager(central : CBCentralManager,
        didConnectPeripheral peripheral : CBPeripheral)
    {
        print("Connected peripheral \(peripheral.name)")
        peripheralActive!.discoverServices(UUID_SERVICES)
    }
    
    // CBPeripheralDelegate interface
    func peripheral(peripheral: CBPeripheral,
        didDiscoverServices error: NSError?)
    {
        assert(error==nil)
        // assert(peripheralActive!.services.count==1)
        
        for service in peripheralActive!.services! {
            print("Discovered service \(service.UUID.UUIDString)")
            peripheralActive!.discoverCharacteristics(nil, forService: service)
        }
    }
    
    func peripheral(peripheral: CBPeripheral,
        didDiscoverCharacteristicsForService service: CBService,
        error: NSError?)
    {
        assert(error==nil)
        assert(service.characteristics!.count==2)
        for characteristic in service.characteristics! {
            print("Discovered characteristic \(characteristic.UUID.UUIDString)")
            if characteristic.UUID.UUIDString == UUID_TX_CHARACTERISTICS.UUIDString {
                print("TX Characteristics found")
                charTX = characteristic
            } else {
                assert( characteristic.UUID.UUIDString == UUID_RX_CHARACTERISTICS.UUIDString )
                print("RX Characteristics found")
                charRX = characteristic
                
                peripheralActive!.setNotifyValue(true, forCharacteristic: charRX)
            }
        }
        assert(initialized())
    }
    
    func peripheral(peripheral: CBPeripheral, didUpdateNotificationStateForCharacteristic characteristic: CBCharacteristic, error: NSError?)
    {
        print("Subscribing to Characteristics succeeded")
        assert(error==nil)
    }
    
    func peripheral(peripheral: CBPeripheral, didUpdateValueForCharacteristic characteristic: CBCharacteristic, error: NSError?)
    {
        assert(error==nil)
        let data = charRX.value!
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
    
    var controller : RobotViewController // TODO: Can't set to weak var
    
    var centralmgr : CBCentralManager!
    var peripheralActive : CBPeripheral?
    
    var charRX : CBCharacteristic!
    var charTX : CBCharacteristic!
    
    let UUID_SERVICES = [CBUUID(string:"713D0000-503E-4C75-BA94-3148F18D941E")]
    let UUID_RX_CHARACTERISTICS = CBUUID(string:"713D0002-503E-4C75-BA94-3148F18D941E")
    let UUID_TX_CHARACTERISTICS = CBUUID(string:"713D0003-503E-4C75-BA94-3148F18D941E")
}