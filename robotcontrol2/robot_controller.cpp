#include "robot_controller_c.h"

#include "math.h"
#include "geometry.h"
#include "nonmoveable.h"
#include "occupancy_grid.h"

#include <vector>

namespace rbt {
    struct CRobotController : rbt::nonmoveable {
        CRobotController()
            : m_occgrid(rbt::size<int>(400, 400), /*nScale*/5) {} // = Map of 20m x 20m map
        
        void receivedSensorData(SSensorData const& data) {
            // TODO: Fuse odometry and IMU sensors?
            auto const fYaw = yawToRadians(data.m_nYaw);
            
            auto const ptfPrev = m_vecpairptfPose.empty() ? rbt::point<double>::zero() : m_vecpairptfPose.back().first;
            rbt::point<double> ptf;
            if(rbt::sign(data.m_anEncoderTicks[0]) != rbt::sign(data.m_anEncoderTicks[1])) {
                // turning, position does not change
                ptf = ptfPrev;
            } else {
                ptf = ptfPrev + rbt::size<double>::fromAngleAndDistance(fYaw, encoderTicksToCm(data.m_anEncoderTicks[0]));
            }
            
            m_vecpairptfPose.emplace_back(std::make_pair(ptf, yawToRadians(data.m_nYaw)));
            m_occgrid.update(ptf, fYaw, data.m_nAngle, data.m_nDistance + sonarOffset(data.m_nAngle));
        }
        
        rbt::COccupancyGrid m_occgrid;
        std::vector<std::pair<rbt::point<double>, double>> m_vecpairptfPose;
    };
}
rbt::CRobotController g_robotcontroller;

struct CRobotController* robot_new_controller() {
    return reinterpret_cast<::CRobotController*>(&g_robotcontroller);
}

struct SPose robot_received_sensor_data(struct CRobotController* probot, struct SSensorData data) {
    auto& robotcontroller = *reinterpret_cast<rbt::CRobotController*>(probot);
    robotcontroller.receivedSensorData(data);
    auto const& pairptfPose = robotcontroller.m_vecpairptfPose.back();
    auto ptnGrid = robotcontroller.m_occgrid.toGridCoordinates(pairptfPose.first);
    return { ptnGrid.x, ptnGrid.y, pairptfPose.second };
}

struct SBitmap robot_get_map(struct CRobotController* probot, bool bEroded) {
    auto const& robotcontroller = *reinterpret_cast<rbt::CRobotController*>(probot);
    return robotcontroller.m_occgrid.bitmap(bEroded);
}

//
//func withTimer(fn: () -> Void) {
//    var timebase = mach_timebase_info_data_t()
//    mach_timebase_info(&timebase)
//    
//    let tBegin = mach_absolute_time();
//    
//    fn()
//    
//    let tEnd = mach_absolute_time();
//    let tNanoseconds = (tEnd - tBegin) * UInt64(timebase.numer) / UInt64(timebase.denom)
//    
//    NSLog("\(tNanoseconds / 1000000) ms")
//}
//
