//
//  edge_following_strategy.hpp
//  robotcontrol2
//
//  Created by Sebastian Theophil on 29.09.15.
//  Copyright © 2015 Sebastian Theophil. All rights reserved.
//

#ifndef edge_following_strategy_hpp
#define edge_following_strategy_hpp

#include "occupancy_grid.h"
#include <boost/optional.hpp>

namespace rbt {
    struct CEdgeFollowingStrategy {
        CEdgeFollowingStrategy() {}
        boost::optional<SRobotCommand> update(point<double> const& ptfPrev, point<double> const& ptf,
                                              double fYawPrev, double fYaw,
                                              ECommand ecmdLast,
                                              COccupancyGrid const& occgrid);
        
        cv::Mat const& FeatureRGBMap() const { return m_matrgbMapFeatures; }
        
    private:
        void FindNewTarget(point<double> const& ptf, COccupancyGrid const& occgrid, int const nMaxExplorationDistance );
            
        cv::Mat m_matnMapThreshold;
        cv::Mat m_matnMapPathMask;
        
        cv::Mat m_matfMapDistance;

        cv::Mat m_matrgbMapFeatures; // for visualization only
        
        enum class state {
            stopped,
            start_turning,
            turning,
            moving,
            stopped_after_turn
        } m_estate = state::stopped;
        rbt::point<int> m_ptnTarget = rbt::point<int>::invalid();
    };
}
#endif /* edge_following_strategy_hpp */
