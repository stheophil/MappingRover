#ifndef _rover_h
#define _rover_h

#include <math.h>
#include <assert.h>

enum ECommand { // force enum to be 4 bytes for Swift - AVR GCC interop
    ecmdSTOP = 0x10000000,
    ecmdMOVE = 0x10100000
};

struct SRobotCommand {
    enum ECommand m_cmd;
    short m_nSpeedLeft;     // max is 255, but leave some room for PID
    short m_nSpeedRight;
};

struct SSensorData { // must be < 64 bytes
    short m_nPitch; // angles in 1/1000 radians
    short m_nRoll;
    short m_nYaw;
    short m_nAngle; // sonar sensor angle
    short m_nDistance; // in cm
    short m_anEncoderTicks[4]; // front left, front right, back left, back right
};

double yawToRadians(short nYaw) {
    return -nYaw/1000.0;
}

// Robot configuration

int c_nRobotWidth = 30; // cm
int c_nRobotHeight = 30; // cm

int c_nWheelRadius = 6; // cm

int c_anSonarOffset[] = {6, 7, 2}; // cm for -90, 0, 90 Angle

int sonarOffset(short nAngle) {
    assert(nAngle==0 || abs(nAngle)==90);
    return c_anSonarOffset[nAngle/90 + 1];
}

double c_fSonarMaxDistance = 300.0; // cm = Sonar max distance (Note: depends on mounting height)
double c_fSonarOpeningAngle = M_PI_2 / 6; // 15 degrees = Sensor opening angle (Note: Depends on sensor)
double c_fSonarDistanceTolerance = 5.0; // cm (Note: Need to calibrate, should be in % maybe.)

double encoderTicksToCm(short nTicks) { // Note: Formula depends on wheel encoders
    return nTicks * 6.0 * M_PI * c_nWheelRadius / 1000.0;
}

short c_nMaxTurnSpeed = 200; // in same units as SRobotCommand.m_nSpeed*
short c_nMaxFwdSpeed = 200;

#endif
