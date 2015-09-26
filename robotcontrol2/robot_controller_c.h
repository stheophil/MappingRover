#import "../rover/src/rover.h" // Data structures and configuration data shared with Arduino controller
#include <stddef.h>

#ifndef robot_controller_c
#define robot_controller_c

#ifdef __cplusplus
extern "C" {
#endif

// Robot controller C interface used by Swift GUI
struct CRobotController;
struct CRobotController* robot_new_controller();

// Returns new robot pose. The x,y coordinates are in coordinates of the
// internal maps, i.e., they may be scaled.
// The yaw angle is returned in radians.
struct SPose {
    int x;
    int y;
    double fYaw;
};
struct SPose robot_received_sensor_data(struct CRobotController* probot, struct SSensorData data);

// Returns pointer to the current robot maps as bitmaps.
// Returns either the raw map (bEroded = false)
// or the map with erosion filter applied (bEroded = true)
struct SBitmap {
    unsigned char* m_pbImage;
    size_t m_cbBytesPerRow;
    size_t m_nWidth;
    size_t m_nHeight;
    size_t m_nScale; // cm per pixel
};
struct SBitmap robot_get_map(struct CRobotController* probot, bool bEroded);

#ifdef __cplusplus
}
#endif

#endif // robot_controller_c