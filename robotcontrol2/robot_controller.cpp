#include "robot_controller_c.h"

#include "math.h"
#include "geometry.h"
#include "nonmoveable.h"
#include <vector>

namespace rbt {
    struct SRobotController : rbt::nonmoveable {
        SRobotController() {}
        
        void receivedSensorData(SSensorData const& data) {
            // TODO: Fuse odometry and IMU sensors?
            auto const fYaw = yawToRadians(data.m_nYaw);
            
            auto const ptPrev = m_vecpairptfPose.empty() ? rbt::point<int>::zero() : m_vecpairptfPose.back().first;
            rbt::point<int> pt;
            if(rbt::sign(data.m_anEncoderTicks[0]) != rbt::sign(data.m_anEncoderTicks[1])) {
                // turning, position does not change
                pt = ptPrev;
            } else {
                pt = ptPrev + rbt::size<int>::fromAngleAndDistance(fYaw, encoderTicksToCm(data.m_anEncoderTicks[0]));
            }
            
            m_vecpairptfPose.emplace_back(std::make_pair(pt, yawToRadians(data.m_nYaw)));
            // m_occgrid.update(pt, fYaw, data.m_nAngle, data.m_nDistance + sonarOffset(data.m_nAngle));
        }
    private:
        std::vector<std::pair<rbt::point<int>, double>> m_vecpairptfPose;
    };
}
rbt::SRobotController g_robotcontroller;

struct SRobotController* robot_new_controller() {
    return reinterpret_cast<::SRobotController*>(&g_robotcontroller);
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
