#include "Arduino.h"
#include "rover.h"
#include "ahrs.h"

#include "SPI.h"
#include "boards.h"
#include "RBL_nRF8001.h"
#include "PID_v1.h"

// #define PID_TEST
// #define CALIBRATE
// #define AHRS_TEST

static const int MAX_SPEED = 500; // max encoder ticks per second

struct SMotor {
    const int POWER;
    const int DIR;
    const int CURRENT;
    
    const int ENCODER_IRQ; // Interrupt, not PIN
    
    void setup() {
        pinMode(POWER, OUTPUT);
        pinMode(DIR, OUTPUT);
        pinMode(CURRENT, INPUT);
        
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
    {4, 33, 35, 0},
    {5, 37, 39, 1},
    {6, 41, 43, 4},
    {7, 45, 47, 5}
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
    const int ECHO;
    const int TRIGGER;
    const int ANGLE; // 0: front, 90: left, -90 right
};

SSonar g_asonar[] = {
    { 32, 34, 90 },
    { 36, 38, 0 },
    { 40, 42, -90 }
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

unsigned long g_nLastCommand = 0; // time in millis() of last command
SRobotCommand g_cmdLastCommand;

static const int c_nMinYaw = (int)(-M_PI * 1000 + 0.5);
static const int c_nMaxYaw = (int)(M_PI * 1000 + 0.5);

int YawDifference(short nYawTarget) {
    int nYawDiff = nYawTarget - g_nYaw;
    if(nYawDiff<c_nMinYaw) nYawDiff+=2*c_nMaxYaw;
    if(c_nMaxYaw<=nYawDiff) nYawDiff-=2*c_nMaxYaw;
    return nYawDiff;
} // < 0 -> turn left

void HandleCommand(SRobotCommand const& cmd) {
    g_cmdLastCommand = cmd;
    g_nLastCommand = millis();
    
    switch(cmd.m_cmd) {
        case ecmdMOVE:
        case ecmdTURN360:
        case ecmdTURN:
        {
            short nSpeedLeft;
            short nSpeedRight;
            if(ecmdMOVE==cmd.m_cmd) {
                nSpeedLeft = cmd.arg.move.m_nSpeedLeft;
                nSpeedRight = cmd.arg.move.m_nSpeedRight;
            } else {
                bool const bTurnLeft = ecmdTURN360==cmd.m_cmd || YawDifference(cmd.arg.turn.m_nYawTarget)<0;
                nSpeedLeft = cmd.arg.turn.m_nSpeed * (bTurnLeft ? -1 : 1);
                nSpeedRight = cmd.arg.turn.m_nSpeed * (bTurnLeft ? 1 : -1);

                if(ecmdTURN360==cmd.m_cmd) {
                    g_cmdLastCommand.arg.turn.m_nYawTarget = g_nYaw;
                    Serial.println("Turn 360");
                } else {
                    Serial.print("Turn ");
                    Serial.print(bTurnLeft ? "left " : "right ");
                    Serial.print(g_nYaw);
                    Serial.print(" -> ");
                    Serial.println(cmd.arg.turn.m_nYawTarget);
                }
            }
            
            bool bReverse = false;
            for(int i=0; i<countof(g_amotors); ++i) {
                int nSpeed = i%2==0 ? nSpeedLeft : nSpeedRight;
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
            g_amotors[0].SetSpeed(constrain(nSpeedLeft, -MAX_SPEED, MAX_SPEED));
            g_amotors[2].SetSpeed(constrain(nSpeedLeft, -MAX_SPEED, MAX_SPEED));
            // RIGHT MOTORS
            g_amotors[1].SetSpeed(constrain(nSpeedRight, -MAX_SPEED, MAX_SPEED));
            g_amotors[3].SetSpeed(constrain(nSpeedRight, -MAX_SPEED, MAX_SPEED));
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
        g_nYaw,
        nAngle, nDistance,
        g_amotors[0].Pop(),
        g_amotors[1].Pop(),
        g_amotors[2].Pop(),
        g_amotors[3].Pop(),
        g_cmdLastCommand.m_cmd
    };
    ble_write_bytes((byte*)&data, sizeof(data));
}

#if defined(PID_TEST)
#include "pidtest.h"
#elif defined(CALIBRATE)
#include "calibrate.h"
#elif defined(AHRS_TEST)
#include "ahrs_test.h"
#else

bool g_bConnected = false;
static const unsigned long c_nTIMETOSTOP = 200; // ms

void loop()
{
    updateAHRS(); // ~ 4 ms, runs at 50 Hz
    
    if(ble_connected()!=g_bConnected) {
        g_bConnected=ble_connected();
        if(g_bConnected) {
            OnConnection();
        } else {
            g_cmdLastCommand = c_rcmdStop;
            OnDisconnection();
        }
    }
    
    if(g_bConnected) {
        if(ecmdTURN360==g_cmdLastCommand.m_cmd || ecmdTURN==g_cmdLastCommand.m_cmd) {
            const int nYawTolerance = 17; // ~ pi/180 * 1000 ie one degree
            int const nYawDiff = YawDifference(g_cmdLastCommand.arg.turn.m_nYawTarget);
            
            // wait a bit before comparing angles when rotating 360
            if(ecmdTURN360==g_cmdLastCommand.m_cmd) {
                if(1500 < millis() - g_nLastCommand
                && nYawDiff >= -nYawTolerance && nYawDiff < 3*nYawTolerance) { // Upper tolerance depends on turn speed!
                    Serial.println("Turned 360 deg. Stopping");
                    HandleCommand(c_rcmdStop);
                }
            } else {
                bool const bTurnLeft = g_amotors[0].m_bReverse;
                if(abs(nYawDiff)<=nYawTolerance) {
                    Serial.println("Turn complete. Stopping");
                    HandleCommand(c_rcmdStop);
                } else if(bTurnLeft != nYawDiff<=0) {
                    Serial.println("Wrong direction. Did yaw measurement change? Try again.");
                    HandleCommand(g_cmdLastCommand);
                }
            }
            
        } else if(ble_available()) {
            SRobotCommand cmd;
            char* pcmd = (char*)&cmd;
            char* pcmdEnd = pcmd+sizeof(SRobotCommand);
            for(; pcmd<pcmdEnd && ble_available(); ++pcmd) {
                *pcmd = ble_read();
            }
            if(pcmd==pcmdEnd) {
                HandleCommand(cmd);
            } else {
                Serial.print("Incomplete Command. Read ");
                Serial.print(pcmd - (char*)&cmd);
                Serial.println(" bytes");
            }
        } else if(c_nTIMETOSTOP < millis()-g_nLastCommand) {
            HandleCommand(c_rcmdStop);
        }
        for(int i=0; i<countof(g_amotors); ++i) {
            g_amotors[i].ComputePID(g_apid[i]); // effective sample time ~ 130 ms
        }
        SendSensorData(); // ~ 40 ms
    }
    
    ble_do_events();
}
#endif