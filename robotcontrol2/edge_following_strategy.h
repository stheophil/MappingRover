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

namespace rbt {
    struct CEdgeFollowingStrategy {
        CEdgeFollowingStrategy() {}
        void update(point<double> const& ptfPrev, point<double> const& ptf, double fYaw, COccupancyGrid const& occgrid);
        
        cv::Mat const& FeatureRGBMap() const { return m_matrgbMapFeatures; }
        
    private:
        cv::Mat m_matnMapThreshold;
        cv::Mat m_matnMapPathMask;
        
        cv::Mat m_matnMapDistance;

        cv::Mat m_matrgbMapFeatures; // for visualization only
    };
}
#endif /* edge_following_strategy_hpp */
