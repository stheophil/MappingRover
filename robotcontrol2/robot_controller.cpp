#include "robot_controller_c.h"

#include "math.h"
#include "geometry.h"
#include "nonmoveable.h"
#include "occupancy_grid.h"
#include "edge_following_strategy.h"

#include <vector>
#include <chrono>
#include <boost/algorithm/cxx11/all_of.hpp>

namespace rbt {
    struct CRobotController : rbt::nonmoveable {
        CRobotController()
            : m_occgrid(rbt::size<int>(400, 400), /*nScale*/5) // = Map of 20m x 20m map
            , m_bConnected(false)
        {}
        
        boost::optional<SRobotCommand> receivedSensorData(SSensorData const& data) {
            auto const fYaw = yawToRadians(data.m_nYaw); // TODO: Fuse odometry and IMU sensors?
            
            auto const ptfPrev = m_vecpairptfPose.empty() ? rbt::point<double>::zero() : m_vecpairptfPose.back().first;
            rbt::point<double> ptf;
            if(ecmdTURN360==data.m_ecmdLast || ecmdTURN==data.m_ecmdLast) {
                // turning, position does not change
                ptf = ptfPrev;
            } else {
                // assert(boost::algorithm::all_of(
                // data.m_anEncoderTicks, [&](int nTick) { return rbt::sign(data.m_anEncoderTicks[0]) == rbt::sign(nTick); })
                // );
                ptf = ptfPrev + rbt::size<double>::fromAngleAndDistance(fYaw, encoderTicksToCm(data.m_anEncoderTicks[0]));
            }
            
            // Add poses even while we're still ignoring sensor data, so we can return pose in robot_received_sensor_data
            auto const fYawPrev = m_vecpairptfPose.empty() ? fYaw : m_vecpairptfPose.back().second;
            m_vecpairptfPose.emplace_back(std::make_pair(ptf, fYaw));
            
            // wait 10s after connection for sensors before taking measurements seriously
            if(!m_bConnected) {
                m_bConnected = true;
                m_tStart = std::chrono::system_clock::now();
                return boost::none;
            } else {
                std::chrono::duration<double> s(std::chrono::system_clock::now() - m_tStart);
                if(s.count() < 10) {
                    return boost::none;
                }
            }

            m_occgrid.update(ptf,
                             fYaw,
                             data.m_nAngle,
                             data.m_nDistance + sonarOffset(data.m_nAngle)); // TODO: Add sonarOffset to position instead?
            
            return m_edgefollow.update(ptfPrev, ptf, fYawPrev, fYaw, data.m_ecmdLast, m_occgrid);
        }
        
        rbt::COccupancyGrid m_occgrid;
        rbt::CEdgeFollowingStrategy m_edgefollow;
        std::vector<std::pair<rbt::point<double>, double>> m_vecpairptfPose;
        
        bool m_bConnected;
        std::chrono::time_point<std::chrono::system_clock> m_tStart;
    };
}
rbt::CRobotController g_robotcontroller;

struct CRobotController* robot_new_controller() {
    return reinterpret_cast<::CRobotController*>(&g_robotcontroller);
}

struct SPose robot_received_sensor_data(struct CRobotController* probot, struct SSensorData data, struct SRobotCommand* prcmd, bool* pbSend) {
    auto& robotcontroller = *reinterpret_cast<rbt::CRobotController*>(probot);
    auto orcmd = robotcontroller.receivedSensorData(data);
    *pbSend = static_cast<bool>(orcmd);
    if(orcmd) *prcmd = *orcmd;
    
    auto const& pairptfPose = robotcontroller.m_vecpairptfPose.back();
    return { pairptfPose.first.x, pairptfPose.first.y, pairptfPose.second };
}

struct SBitmap robot_get_map(struct CRobotController* probot, bitmap_type bm) {
    auto const& robotcontroller = *reinterpret_cast<rbt::CRobotController*>(probot);

    SBitmap bitmap;
    bitmap.m_pbImage = [&]() {
        switch(bm) {
            case bitmap_type::greyscale: return robotcontroller.m_occgrid.GreyscaleMap().data;
            case bitmap_type::eroded: return robotcontroller.m_occgrid.ErodedMap().data;
            case bitmap_type::edges: return robotcontroller.m_occgrid.ErodedMap().data;
            case bitmap_type::features: return robotcontroller.m_edgefollow.FeatureRGBMap().data;
        }
    }();
    
    bitmap.m_cChannels = (bm==bitmap_type::features ? 3 : 1);
    bitmap.m_cbBytesPerRow = robotcontroller.m_occgrid.m_szn.x * bitmap.m_cChannels * sizeof(std::uint8_t);
    bitmap.m_nWidth = robotcontroller.m_occgrid.m_szn.x;
    bitmap.m_nHeight = robotcontroller.m_occgrid.m_szn.y;
    bitmap.m_nScale = robotcontroller.m_occgrid.m_nScale;
    return bitmap;
}
