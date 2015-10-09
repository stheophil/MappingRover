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

// Returns new robot pose. The x,y coordinates are in world coordinates, i.e.,
// not scaled according to occupancy grid resolution.
// The yaw angle is returned in radians.
struct SPose {
    double x;
    double y;
    double fYaw;
};
struct SPose robot_received_sensor_data(struct CRobotController* probot, struct SSensorData data, struct SRobotCommand* prcmd, bool* pbSend);

// Returns pointer to the current robot maps as bitmaps.
// Returns either the raw map (bEroded = false)
// or the map with erosion filter applied (bEroded = true)
struct SBitmap {
    unsigned char* m_pbImage;
    size_t m_cChannels; // No of 8-bit channels, 1 (Greyscale) or 3 (RGB)
    size_t m_cbBytesPerRow;
    size_t m_nWidth;
    size_t m_nHeight;
    size_t m_nScale; // cm per pixel
};
    
enum bitmap_type {
    greyscale, eroded, edges, features
};
    
struct SBitmap robot_get_map(struct CRobotController* probot, enum bitmap_type bm);

#ifdef __cplusplus
}
#endif

#endif // robot_controller_c