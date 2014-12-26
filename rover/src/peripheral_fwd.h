#ifndef peripheral_peripheral_h
#define peripheral_peripheral_h

struct SMotor;

struct SRobotCommand;

void OnConnection();
void OnDisconnection();
void HandleCommand(SRobotCommand const& cmd);

#endif
