//
//  occupancy_grid.h
//  robotcontrol2
//
//  Created by Sebastian Theophil on 24.09.15.
//  Copyright © 2015 Sebastian Theophil. All rights reserved.
//

#ifndef occupancy_grid_h
#define occupancy_grid_h

#include "nonmoveable.h"
#include "geometry.h"

namespace rbt {
    struct COccupancyGrid : rbt::nonmoveable {
        COccupancyGrid(rbt::size<int> const& szn, int nScale) : m_szn(szn), m_nScale(nScale) {}
        
        void update(point<int> const& pt, double fYaw, int nAngle, int nDistance) {
        }
        
    private:
        rbt::size<int> m_szn;
        int m_nScale; // cm per pixel
    };
}
#endif /* occupancy_grid_h */
