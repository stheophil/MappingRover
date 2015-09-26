//
//  occupancy_grid.h
//  robotcontrol2
//
//  Created by Sebastian Theophil on 24.09.15.
//  Copyright © 2015 Sebastian Theophil. All rights reserved.
//

#ifndef occupancy_grid_h
#define occupancy_grid_h

#include "robot_controller_c.h"

#include "nonmoveable.h"
#include "geometry.h"

#include <opencv2/core.hpp>

namespace rbt {
    struct COccupancyGrid : rbt::nonmoveable {
        COccupancyGrid(rbt::size<int> const& szn, int nScale);        
        void update(point<double> const& ptf, double fYaw, int nAngle, int nDistance);
        
        point<int> toGridCoordinates(point<double> const& pt) const;
        point<int> toWorldCoordinates(point<int> const& pt) const;
    
        struct SBitmap bitmap(bool bEroded) const;
        
    private:
        rbt::size<int> m_szn;
        int m_nScale; // cm per pixel
        
        cv::Mat m_matfMapLogOdds;
        cv::Mat m_matnMapGreyscale;
        cv::Mat m_matnMapEroded;
    };
}
#endif /* occupancy_grid_h */
