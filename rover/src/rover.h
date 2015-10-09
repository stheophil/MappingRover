#ifndef _rover_h
#define _rover_h

#include <math.h>
#include <assert.h>

typedef short ECommand; // force enum to be 2 bytes for Swift - AVR GCC interop
const short ecmdSTOP = 0x0;
const short ecmdMOVE = 1 << 1;
const short ecmdTURN360 = 1 << 2;
const short ecmdTURN = 1 << 3;

struct SMove {
    short m_nSpeedLeft;     // max is 255, but leave some room for PID
    short m_nSpeedRight;
};

struct STurn {
    short m_nSpeed;
    short m_nYawTarget; // iff ecmdTURN
};

struct SRobotCommand {
    ECommand m_cmd;
    union {
        struct SMove move;
        struct STurn turn;
    } arg;
};

// Speeds are given in same units as SRobotCommand.m_nSpeed*
// 100 is a good value so the robot does not turn too fast
// and can be stopped at the right time
const short c_nMaxTurnSpeed = 100;
const short c_nMaxFwdSpeed = 200;

const struct SRobotCommand c_rcmdStop = {ecmdSTOP};
const struct SRobotCommand c_rcmdForward = {ecmdMOVE, c_nMaxFwdSpeed, c_nMaxFwdSpeed};
const struct SRobotCommand c_rcmdTurn360 = {ecmdTURN360, c_nMaxTurnSpeed};

inline struct SRobotCommand RobotCommandTurn(double fYawTarget) {
    struct SRobotCommand rcmd = {
        ecmdTURN,
        c_nMaxTurnSpeed,
        (short)(-fYawTarget*1000+0.5)
    };
    return rcmd;
}

// Manual controls only:
const struct SRobotCommand c_rcmdBackward = {ecmdMOVE, -c_nMaxFwdSpeed, -c_nMaxFwdSpeed};
const struct SRobotCommand c_rcmdTurnLeft = {ecmdMOVE, -c_nMaxFwdSpeed, c_nMaxFwdSpeed};
const struct SRobotCommand c_rcmdTurnRight = {ecmdMOVE, c_nMaxFwdSpeed, -c_nMaxFwdSpeed};

struct SSensorData { // must be < 64 bytes
    short m_nYaw;
    short m_nAngle; // sonar sensor angle
    short m_nDistance; // in cm
    short m_anEncoderTicks[4]; // front left, front right, back left, back right
    ECommand m_ecmdLast; // send last processed command so the controller can check when the turn is completed
};

inline double yawToRadians(short nYaw) {
    return -nYaw/1000.0; // on-board yaw measurement is flipped, i.e. not in counter-clockwise direction
}

// Robot configuration
const int c_nRobotWidth = 30; // cm
const int c_nRobotHeight = 30; // cm

const int c_nWheelRadius = 6; // cm

// Distance from robot center in cm
// for sensor with -90, 0, 90.
// TODO: Wrong offsets? Rotation is CCW.
const int c_anSonarOffset[] = {6, 7, 2};

inline int sonarOffset(short nAngle) {
    assert(nAngle==0 || abs(nAngle)==90);
    return c_anSonarOffset[nAngle/90 + 1];
}

const double c_fSonarMaxDistance = 300.0; // cm = Sonar max distance (Note: depends on mounting height)
const double c_fSonarOpeningAngle = M_PI / 12; // 15 degrees = Sensor opening angle (Note: Depends on sensor)
const double c_fSonarDistanceTolerance = 5.0; // cm (Note: Need to calibrate, should be in % maybe.)

inline double encoderTicksToCm(short nTicks) { // Note: Formula depends on wheel encoders
    return nTicks * 6.0 * M_PI * c_nWheelRadius / 1000.0;
}

#endif
