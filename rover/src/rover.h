#ifndef _rover_h
#define _rover_h

enum ECommand { // force enum to be 4 bytes for Swift - AVR GCC interop
    ecmdSTOP = 0x10000000,
    ecmdMOVE = 0x10100000
};

struct SRobotCommand {
    enum ECommand m_cmd;
    short m_nSpeedLeft;
    short m_nSpeedRight;
};

struct SSensorData { // must be < 64 bytes
    short m_nPitch; // angles in 1/1000 radians
    short m_nRoll;
    short m_nYaw;
    short m_anEncoderTicks[4];
};

#endif
