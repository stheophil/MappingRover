#include "Arduino.h"
#include "peripheral_fwd.h"

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
        
        m_cTicks = 0;
    }
    
    int m_cTicks;
    void onInterrupt() {
        ++m_cTicks;
    }
};

SMotor c_amotors[] = {
    // See pins.txt
    {4, 33, 35, 0, 32, 34},
    {5, 37, 39, 1, 36, 38},
    {6, 41, 43, 5, 40, 42},
    {7, 45, 47, 4, 44, 46}
};

void OnMotor0Interrupt() { c_amotors[0].onInterrupt(); }
void OnMotor1Interrupt() { c_amotors[1].onInterrupt(); }
void OnMotor2Interrupt() { c_amotors[2].onInterrupt(); }
void OnMotor3Interrupt() { c_amotors[3].onInterrupt(); }

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

    // Port setup
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);
    
    // Motor setup
    for(int i=0; i<4; ++i) {
      c_amotors[i].setup();
      attachInterrupt(c_amotors[i].ENCODER_IRQ, c_afnInterrupts[i], CHANGE);
    }
   
    Serial.println("BLE Arduino RobotController");
}

enum ECommand {
    ecmdSTOP = 0x1001,
    ecmdMOVE = 0x1010
};

struct SRobotCommand {
    ECommand cmd;
    int arg1;
    int arg2;
};

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
    switch(cmd.cmd) {
        case ecmdMOVE:
            // LEFT MOTORS:
            digitalWrite(c_amotors[0].DIR, cmd.arg1 < 0 ? LOW : HIGH);
            analogWrite(c_amotors[0].POWER, abs(cmd.arg1));
            digitalWrite(c_amotors[2].DIR, cmd.arg1 < 0 ? LOW : HIGH);
            analogWrite(c_amotors[2].POWER, abs(cmd.arg1));
            // RIGHT MOTORS
            digitalWrite(c_amotors[1].DIR, cmd.arg2 < 0 ? LOW : HIGH);
            analogWrite(c_amotors[1].POWER, abs(cmd.arg2));
            digitalWrite(c_amotors[3].DIR, cmd.arg2 < 0 ? LOW : HIGH);
            analogWrite(c_amotors[3].POWER, abs(cmd.arg2));
            break;
            
        case ecmdSTOP:
            for(int i=0; i<4; ++i) analogWrite(c_amotors[i].POWER, 0);
            break;
            
        default: ;
    }
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
            anTicks[iTick * 4 + i] = c_amotors[i].m_cTicks;
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
    if(ble_available())
    {
        SRobotCommand cmd;
        char* pcmd = (char*)&cmd;
        char* pcmdEnd = pcmd+sizeof(SRobotCommand);
        for(; pcmd<pcmdEnd && ble_available(); ++pcmd) {
            *pcmd = ble_read();
        }
        if(pcmd==pcmdEnd) {
            g_nLastCommand = millis();
            HandleCommand(cmd);
        } else {
            Serial.print("Incomplete Command. Read ");
            Serial.print(pcmd - (char*)&cmd);
            Serial.println(" bytes");
        }
        
        ble_do_events();
        return;
    } if(c_nTIMETOSTOP < millis()-g_nLastCommand) {
        SRobotCommand cmdStop = {ecmdSTOP, 0, 0};
        HandleCommand(cmdStop);
    } else if(ble_connected()!=g_bConnected) {
        g_bConnected=ble_connected();
        if(g_bConnected) {
            OnConnection();
        } else {
            OnDisconnection();
        }
    }
    ble_do_events();
}
