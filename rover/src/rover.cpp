#include "Arduino.h"
#include "rover.h"
#include "ahrs.h"

#include "SPI.h"
#include "boards.h"
#include "RBL_nRF8001.h"

struct SMotor {
    const int POWER;
    const int DIR;
    const int CURRENT;
    
    const int ENCODER_IRQ; // Interrupt, not PIN
    const int ENC1;
    const int ENC2;
    
    void setup() {
        pinMode(POWER, OUTPUT);
        pinMode(DIR, OUTPUT);
        pinMode(CURRENT, INPUT);
        
        pinMode(ENC1, INPUT);
        pinMode(ENC2, INPUT);
        
        m_nTicks = 0;
    }
    
    int m_nSpeed;
    void SetSpeed(int nSpeed) {
        m_nSpeed = nSpeed;
        digitalWrite(DIR, m_nSpeed < 0 ? LOW : HIGH);
        analogWrite(POWER, abs(m_nSpeed));
    }
    
    int m_nTicks;
    void onInterrupt() {
        ++m_nTicks;
    }
    
    int Pop() {
        int nTick = m_nTicks;
        m_nTicks = 0;
        return nTick * (m_nSpeed < 0 ? -1 : 1);
    }
};

SMotor g_amotors[] = {
    // See pins.txt
    {4, 33, 35, 0, 32, 34},
    {5, 37, 39, 1, 36, 38},
    {6, 41, 43, 5, 40, 42},
    {7, 45, 47, 4, 44, 46}
};

void OnMotor0Interrupt() { g_amotors[0].onInterrupt(); }
void OnMotor1Interrupt() { g_amotors[1].onInterrupt(); }
void OnMotor2Interrupt() { g_amotors[2].onInterrupt(); }
void OnMotor3Interrupt() { g_amotors[3].onInterrupt(); }

void (*c_afnInterrupts[4])() = {
  OnMotor0Interrupt, OnMotor1Interrupt, OnMotor2Interrupt, OnMotor3Interrupt
};

#define RELAY_PIN 10

void setup()
{
    Serial.begin(57600);
    delay(3000);  //3 seconds delay for enabling to see the start up comments on the serial board
    
    // BLE setup

    ble_set_name("rcontrol2");
    ble_begin();
    
    
    Serial.print("sizeof(SRobotCommand) = ");
    Serial.println(sizeof(SRobotCommand));
    Serial.print("sizeof(SSensorData) = ");
    Serial.println(sizeof(SSensorData));
    
    // Port setup
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);
    
    // Motor setup
    for(int i=0; i<4; ++i) {
      g_amotors[i].setup();
      attachInterrupt(g_amotors[i].ENCODER_IRQ, c_afnInterrupts[i], CHANGE);
    }
   
    // AHRS
    setupAHRS();
    
    Serial.println("BLE Arduino RobotController");
}

void OnConnection() {
    Serial.println("Connected");
    // Active motor controller
    digitalWrite(RELAY_PIN, LOW);
}

void OnDisconnection() {
    Serial.println("Disconnected");
    // Deactivate motor controller
    digitalWrite(RELAY_PIN, HIGH);
}

void HandleCommand(SRobotCommand const& cmd) {
    switch(cmd.m_cmd) {
        case ecmdMOVE:
            // LEFT MOTORS:
            g_amotors[0].SetSpeed(cmd.m_nSpeedLeft);
            g_amotors[2].SetSpeed(cmd.m_nSpeedLeft);
            // RIGHT MOTORS
            g_amotors[1].SetSpeed(cmd.m_nSpeedRight);
            g_amotors[3].SetSpeed(cmd.m_nSpeedRight);
            break;
            
        case ecmdSTOP:
            for(int i=0; i<4; ++i) g_amotors[i].SetSpeed(0);
            break;
            
        default: ;
    }
}

void SendSensorData() {
    SSensorData data = {
        g_nRoll, g_nPitch, g_nYaw,
        g_amotors[0].Pop(),
        g_amotors[1].Pop(),
        g_amotors[2].Pop(),
        g_amotors[3].Pop()
    };
    ble_write_bytes((byte*)&data, sizeof(data));
    ble_do_events();
}

/*
void MeasurementLoop() {
    OnConnection();
    
    int anTicks[80] = {0};
    SRobotCommand cmd = {ecmdMOVE, 255, 255};
    HandleCommand(cmd);
    
    for(int iTick=0; iTick<20; ++iTick) {
        delay(50);
        
        for(int i=0; i<4; ++i) {
            anTicks[iTick * 4 + i] = g_amotors[i].m_cTicks;
        }
    }
    
    SRobotCommand cmdStop = {ecmdSTOP, 0, 0};
    HandleCommand(cmdStop);
    
    Serial.println("Motor encoder counts");
    
    for(int iTick=0; iTick<20; ++iTick) {
        for(int i=0; i<4; ++i) {
            Serial.print(anTicks[iTick * 4 + i]);
            Serial.print(",");
        }
        Serial.println("");
    }
    
    delay(100000);
}
*/

bool g_bConnected = false;
unsigned long g_nLastCommand = 0;
static const unsigned long c_nTIMETOSTOP = 200; // ms

void loop()
{
    bool bSendSensor = updateAHRS();
    
    if(ble_connected()!=g_bConnected) {
        g_bConnected=ble_connected();
        if(g_bConnected) {
            OnConnection();
        } else {
            OnDisconnection();
        }
    }
    
    if(g_bConnected) {
        if(ble_available())
        {
            SRobotCommand cmd;
            char* pcmd = (char*)&cmd;
            char* pcmdEnd = pcmd+sizeof(SRobotCommand);
            for(; pcmd<pcmdEnd && ble_available(); ++pcmd) {
                *pcmd = ble_read();
            }
            if(pcmd==pcmdEnd) {
                // send current sensor data now, in case direction changes
                bSendSensor = true;
                g_nLastCommand = millis();
                HandleCommand(cmd);
            } else {
                Serial.print("Incomplete Command. Read ");
                Serial.print(pcmd - (char*)&cmd);
                Serial.println(" bytes");
            }
            
            ble_do_events();
            return;
        } else if(c_nTIMETOSTOP < millis()-g_nLastCommand) {
            bSendSensor = true;
            SRobotCommand cmdStop = {ecmdSTOP, 0, 0};
            HandleCommand(cmdStop);
        }
        
        if(bSendSensor) SendSensorData();
    }
    
    ble_do_events();
}
