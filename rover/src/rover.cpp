#include "Arduino.h"
#include "rover.h"
#include "ahrs.h"

#include "SPI.h"
#include "boards.h"
#include "RBL_nRF8001.h"
#include "PID_v1.h"

// #define PID_TEST
static const int MAX_SPEED = 500; // max encoder ticks per second

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
        m_bReverse = false;
        
        m_nLastCompute = 0;
        m_fSpeed = 0.0;
        m_fTicksPerSecond = 0.0;
        m_fPower = 0.0;
    }
    
    double m_fSpeed; // pid set point == desired ticks per second
    double m_fTicksPerSecond; // pid input
    double m_fPower; // pid output in [0, 255]
    
    unsigned long m_nLastCompute;
    int m_nTicksPID;
    
    bool m_bReverse;
    void SetSpeed(int nSpeed) {
        m_fSpeed = abs(nSpeed);
        m_bReverse = nSpeed < 0;
        digitalWrite(DIR, m_bReverse ? LOW : HIGH);
    }
    
    void Stop(PID& pid) {
        pid.SetMode(MANUAL);
        SetSpeed(0);
        
        m_fPower = 0.0;
        m_nTicksPID = 0;
        m_nLastCompute = 0;
        
        analogWrite(POWER, 0);
        pid.SetMode(AUTOMATIC);
    }
    
    bool ComputePID(PID& pid) {
        unsigned long nNow = millis();
        unsigned long nChange = nNow - m_nLastCompute;
        m_fTicksPerSecond = m_nTicksPID / (nChange / 1000.0);
        if(pid.Compute()) { // is the sample time low enough?
            m_nTicksPID = 0;
            m_nLastCompute = nNow;
            analogWrite(POWER, (int)m_fPower);
            return true;
        }
        return false;
    }
    
    int m_nTicks;
    void onInterrupt() {
        ++m_nTicks;
        ++m_nTicksPID;
    }
    
    int Pop() {
        int nTick = m_nTicks;
        m_nTicks = 0;
        return nTick * (m_bReverse ? -1 : 1);
    }
    
    float Current() const {
        return analogRead(CURRENT) * 5.0 / 1023.0;
    }
};

SMotor g_amotors[] = {
    // See pins.txt
    {4, 33, 35, 0, 32, 34},
    {5, 37, 39, 1, 36, 38},
    {6, 41, 43, 4, 40, 42},
    {7, 45, 47, 5, 44, 46}
};

// PID ctor expects double pointer, can't make it member of SMotor
// TODO: http://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
static const double c_fCoeffP = 0.2;
static const double c_fCoeffI = 0.7;
static const double c_fCoeffD = 0.0;

PID g_apid[] = {
    PID(&g_amotors[0].m_fTicksPerSecond, &g_amotors[0].m_fPower, &g_amotors[0].m_fSpeed, c_fCoeffP, c_fCoeffI, c_fCoeffD, DIRECT),
    PID(&g_amotors[1].m_fTicksPerSecond, &g_amotors[1].m_fPower, &g_amotors[1].m_fSpeed, c_fCoeffP, c_fCoeffI, c_fCoeffD, DIRECT),
    PID(&g_amotors[2].m_fTicksPerSecond, &g_amotors[2].m_fPower, &g_amotors[2].m_fSpeed, c_fCoeffP, c_fCoeffI, c_fCoeffD, DIRECT),
    PID(&g_amotors[3].m_fTicksPerSecond, &g_amotors[3].m_fPower, &g_amotors[3].m_fSpeed, c_fCoeffP, c_fCoeffI, c_fCoeffD, DIRECT)
};

void OnMotor0Interrupt() { g_amotors[0].onInterrupt(); }
void OnMotor1Interrupt() { g_amotors[1].onInterrupt(); }
void OnMotor2Interrupt() { g_amotors[2].onInterrupt(); }
void OnMotor3Interrupt() { g_amotors[3].onInterrupt(); }

void (*c_afnInterrupts[4])() = {
  OnMotor0Interrupt, OnMotor1Interrupt, OnMotor2Interrupt, OnMotor3Interrupt
};

#define RELAY_PIN 10

struct SSonar {
    const int TRIGGER;
    const int ECHO;
    const int ANGLE; // 0: front, 90: left, -90 right
};
SSonar g_asonar[] = {
    { 27, 29, 0 }
};

#define countof(a) (sizeof(a)/sizeof(a[0]))

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
    
    for(int i=0; i<countof(g_asonar); ++i) {
        pinMode(g_asonar[i].TRIGGER, OUTPUT);
        pinMode(g_asonar[i].ECHO, INPUT);
    }
    
    // Motor setup
    for(int i=0; i<countof(g_amotors); ++i) {
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
        {
            bool bReverse = false;
            for(int i=0; i<countof(g_amotors); ++i) {
                int nSpeed = i%2==0 ? cmd.m_nSpeedLeft : cmd.m_nSpeedRight;
                bReverse = bReverse || (nSpeed < 0 != g_amotors[i].m_bReverse);
            }
            if(bReverse) { // stop all motors and reset PID
                Serial.println("STOP Motors");
                for(int i=0; i<countof(g_amotors); ++i) {
                    g_amotors[i].Stop(g_apid[i]);
                }
                delay(200);
            }
                
            // LEFT MOTORS:
            g_amotors[0].SetSpeed(constrain(cmd.m_nSpeedLeft, -MAX_SPEED, MAX_SPEED));
            g_amotors[2].SetSpeed(constrain(cmd.m_nSpeedLeft, -MAX_SPEED, MAX_SPEED));
            // RIGHT MOTORS
            g_amotors[1].SetSpeed(constrain(cmd.m_nSpeedRight, -MAX_SPEED, MAX_SPEED));
            g_amotors[3].SetSpeed(constrain(cmd.m_nSpeedRight, -MAX_SPEED, MAX_SPEED));
            break;
        }
        case ecmdSTOP:
            for(int i=0; i<countof(g_amotors); ++i) g_amotors[i].Stop(g_apid[i]);
            break;
            
        default: ;
    }
}

struct SPerformanceCounter {
    unsigned long m_nStart;
    SPerformanceCounter() : m_nStart(micros()) {}
    unsigned long Stop() {
        unsigned long nEnd = micros();
        return nEnd - m_nStart;
    }
};

int g_iSonar = 0;
void SendSensorData() {
    // TODO: Optimize order in which we accumulate sensor data
    // so sensor data is consistent with each other
    // TODO: Transmit current or make emergency stop if motor current too high
    
    int nDistance; // in cm
    int nAngle;
    {
        digitalWrite(g_asonar[g_iSonar].TRIGGER, HIGH);
        delayMicroseconds(10);
        digitalWrite(g_asonar[g_iSonar].TRIGGER, LOW);
        unsigned long t = pulseIn(g_asonar[g_iSonar].ECHO, 20000);
        nDistance = (int)((t / 58.0) + 0.5);
        nAngle = g_asonar[g_iSonar].ANGLE;
        
        g_iSonar = (g_iSonar + 1) % countof(g_asonar);
    } // ~ 20 ms
    
    SSensorData data = {
        g_nRoll, g_nPitch, g_nYaw,
        nAngle, nDistance,
        g_amotors[0].Pop(),
        g_amotors[1].Pop(),
        g_amotors[2].Pop(),
        g_amotors[3].Pop()
    };
    ble_write_bytes((byte*)&data, sizeof(data));
}

#ifdef PID_TEST
#include "pidtest.h"
#else

bool g_bConnected = false;
unsigned long g_nLastCommand = 0;
static const unsigned long c_nTIMETOSTOP = 200; // ms

void loop()
{
    updateAHRS(); // ~ 4 ms, runs at 50 Hz
    
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
                g_nLastCommand = millis();
                HandleCommand(cmd);
            } else {
                Serial.print("Incomplete Command. Read ");
                Serial.print(pcmd - (char*)&cmd);
                Serial.println(" bytes");
            }
        } else if(c_nTIMETOSTOP < millis()-g_nLastCommand) {
            SRobotCommand cmdStop = {ecmdSTOP, 0, 0};
            HandleCommand(cmdStop);
        }
        for(int i=0; i<countof(g_amotors); ++i) {
            g_amotors[i].ComputePID(g_apid[i]); // effective sample time ~ 130 ms
        }
        SendSensorData(); // ~ 40 ms
    }
    
    ble_do_events();
}
#endif