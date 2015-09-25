//
//  occupancy_grid.cpp
//  robotcontrol2
//
//  Created by Sebastian Theophil on 24.09.15.
//  Copyright © 2015 Sebastian Theophil. All rights reserved.
//

#include "occupancy_grid.h"
#import "../rover/src/rover.h" // Data structures and configuration data shared with Arduino controller

#include <assert.h>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <unordered_map>

#include <boost/range/size.hpp>

namespace rbt {
    
    double angularDistance( double fAngleA, double fAngleB) {
        auto fAngle = fAngleA - fAngleB;
        
        // normalize angle to be between (-pi, +pi]
        while(fAngle < -M_PI) {
            fAngle+=2*M_PI;
        }
        while(M_PI <= fAngle) {
            fAngle-=2*M_PI;
        }
        assert(-M_PI<=fAngle && fAngle<M_PI);
        return fAngle;
    }
        
    struct SArc {
        rbt::point<int> m_ptnCenter;
        double m_fAngleFrom;
        double m_fAngleTo;
        double m_fRadius;
        
        template<class Func>
        void for_each_pixel(Func foreach) const {
            auto szFrom = rbt::size<int>::fromAngleAndDistance(m_fAngleFrom, m_fRadius);
            auto szTo = rbt::size<int>::fromAngleAndDistance(m_fAngleTo, m_fRadius);
            
            assert(angularDistance(m_fAngleFrom, m_fAngleTo)<M_PI/2);
            assert(szFrom.compare(szTo) != 0);
            
            if(szFrom.compare(szTo) < 0) { // ptFrom should be left of ptTo
                std::swap(szFrom, szTo);
            }
            
            auto nQuadrantFrom = szFrom.quadrant();
            auto nQuadrantTo = szTo.quadrant();
            
            assert(nQuadrantFrom==nQuadrantTo || nQuadrantFrom==(nQuadrantTo+1)%4);
            
            auto rectnBound = rbt::rect<int>::bound({rbt::point<int>::zero(),
                                                    rbt::point<int>::zero() + szFrom,
                                                    rbt::point<int>::zero() + szTo});
            if(nQuadrantFrom != nQuadrantTo) {
                auto nRadius = rbt::numeric_cast<int>(m_fRadius);
                switch(nQuadrantFrom) {
                    case 0: rectnBound |= rbt::point<int>(nRadius, 0); break;
                    case 1: rectnBound |= rbt::point<int>(0, nRadius); break;
                    case 2: rectnBound |= rbt::point<int>(-nRadius, 0); break;
                    case 3: rectnBound |= rbt::point<int>(0, -nRadius); break;
                    default: assert(false);
                }
            }
            
            auto fSqrRadius = rbt::sqr(m_fRadius);
            for(int y = rectnBound.top; y <= rectnBound.bottom; ++y) { // both-inclusive
                // Scan bound rect lines in y-direction for interval between vectors szFrom and szTo
                // Since arcs are convex, there is (exactly) one contiguous sequence of pixels
                // that fall into the arc. Therefore, we can stop processing a line once we
                // find the first pixel _not_ in the arc, after we have found pixels in the arc.
                bool bFoundPointsInLine = false;
                for(int x = rectnBound.left; x <= rectnBound.right; ++x) {
                    rbt::size<int> sz(x, y); // still relative to point(0,0), the arc center
                    auto nSqrDistance = sz.SqrAbs();
                    
                    if(nSqrDistance < fSqrRadius
                    && 0<=szFrom.compare(sz)    // this is very strict and does not
                    && 0<=sz.compare(szTo)) {   // count grid cells partially inside arc
                        bFoundPointsInLine = true;
                        
                        foreach(m_ptnCenter + sz, nSqrDistance);
                    } else if(bFoundPointsInLine) {
                        break; // go to next line
                    }
                }
            }
        }
    };
    
    struct SRotatedRect {
        rbt::point<int> m_ptnCenter;
        rbt::size<double> m_szf;
        double m_fAngle;
        
        template<typename Func>
        void for_each_pixel(Func foreach) {
            // Rotate the scaled rectangle.
            // TODO: For numerical precision, it may be better to rotate the rect
            // in world coordinates and then scale.
            rbt::point<int> apt[] = {
                m_ptnCenter - rbt::size<int>((m_szf/2).rotated(m_fAngle)),
                m_ptnCenter + rbt::size<int>((rbt::size<double>(m_szf.x, -m_szf.y)/2).rotated(m_fAngle)),
                m_ptnCenter + rbt::size<int>((m_szf/2).rotated(m_fAngle)),
                m_ptnCenter + rbt::size<int>((rbt::size<double>(-m_szf.x, m_szf.y)/2).rotated(m_fAngle))
            };
            
            auto rasterize = [](rbt::point<int> ptA, rbt::point<int> ptB, auto foreach) {
                if(ptA.x == ptB.x) {
                    // straight vertical line
                    auto nMin = std::min(ptA.y, ptB.y);
                    auto nMax = std::max(ptA.y, ptB.y);
                    
                    for(int y = nMin; y <= nMax; ++y) {
                        foreach(ptA.x, y);
                    }
                } else {
                    if(ptB.x<ptA.x) {
                        std::swap(ptB, ptA);
                    }
                    
                    auto m = rbt::numeric_cast<double>(ptB.y - ptA.y) / (ptB.x - ptA.x);
                    if(std::abs(m)<=1) { // x-step
                        for(int x = ptA.x; x<= ptB.x; ++x) {
                            foreach(x, rbt::numeric_cast<int>(ptA.y + m * (x - ptA.x)));
                        }
                    } else { // y-step
                        if(ptB.y < ptA.y) {
                            std::swap(ptB, ptA);
                        }
                        for(int y = ptA.y; y <= ptB.y; ++y) {
                            foreach(rbt::numeric_cast<int>(ptA.x + (y - ptB.y) / m), y);
                        }
                    }
                }
            };
            
            // TODO: The map could be avoided by sorting the line segments
            std::unordered_map<int, rbt::interval<int>> mapnintvlX;
            for(int i=0; i<boost::size(apt); ++i) {
                rasterize(apt[i], apt[(i+1)%boost::size(apt)], [&](int x, int y) {
                    auto pairitb = mapnintvlX.emplace(y, rbt::interval<int>(x, x));
                    if(!pairitb.second) {
                        pairitb.first->second |= x;
                    }
                });
            }
            boost::for_each(mapnintvlX, [&](auto const& pairnintvlX) {
                for(int x = pairnintvlX.second.begin; x <= pairnintvlX.second.begin; ++x) {
                    foreach(rbt::point<int>(x, pairnintvlX.first));
                }
            });
        }
    };

    
    void COccupancyGrid::update(point<double> const& ptf, double fYaw, int nAngle, int nDistance) {
        assert(nAngle==0 || std::abs(nAngle)==90);
        auto const fAngleSonar = fYaw + M_PI_2 * rbt::sign(nAngle);
        auto const ptnGrid = toGridCoordinates(ptf);
        
        SArc arc{ptnGrid,
                 fAngleSonar - c_fSonarOpeningAngle/2,
                 fAngleSonar + c_fSonarOpeningAngle/2,
                 (nDistance + c_fSonarDistanceTolerance/2)/m_nScale
        };
        
        auto const fSqrMaxDistance = rbt::sqr(c_fSonarMaxDistance/m_nScale);
        auto const fSqrMeasuredDistance = rbt::sqr((nDistance - c_fSonarDistanceTolerance/2)/m_nScale);
        
        arc.for_each_pixel([&](point<int> const& pt, double fSqrDistance) {
            if(fSqrDistance < fSqrMaxDistance) {
                auto const fInverseSensorModel = fSqrDistance < fSqrMeasuredDistance
                    ? -0.5 // free
                    : (100.0 / m_nScale) / std::sqrt(fSqrDistance); // occupied
                // TODO:
                // UpdateGrid(x, y, self.grid[x][y] + fInverseSensorModel) // - prior which is 0
            }
        });
        
        // Clear position of robot itself
        SRotatedRect rectRobot{ptnGrid, rbt::size<double>(c_nRobotWidth, c_nRobotHeight)/m_nScale, fYaw};
        rectRobot.for_each_pixel([&](rbt::point<int> const& pt) {
            // TODO:
            // UpdateGrid(x, y, -100);
        });
        
        // Erode image
        // A pixel p in imageEroded is marked free when the robot centered at p does not occupy an occupied pixel in self.image
        // i.e. the pixel p has the maximum value of the surrounding pixels inside the diameter defined by the robot's size
        // We overestimate robot size by taking robot diagonal
        /*
        let nKernelDiameter = UInt( ceil( sqrt( pow(sizeRobot.width, 2) + pow(sizeRobot.height, 2) ) / scale ) )
        let anKernel = [UInt8](count: Int(nKernelDiameter * nKernelDiameter), repeatedValue: 0)
        var vimgbufInput = vImage_Buffer(data: image.bitmapData, height: UInt(image.pixelsHigh), width: UInt(image.pixelsWide), rowBytes: image.bytesPerRow)
        var vimgbufOutput = vImage_Buffer(data: imageEroded.bitmapData, height: UInt(image.pixelsHigh), width: UInt(image.pixelsWide), rowBytes: image.bytesPerRow)
        
        vImageErode_Planar8( &vimgbufInput, &vimgbufOutput, 0, 0, anKernel, nKernelDiameter, nKernelDiameter, UInt32(kvImageNoFlags) )
        */
    }
    
    point<int> COccupancyGrid::toWorldCoordinates(point<int> const& pt) const {
        return (pt - m_szn/2) * m_nScale;
    }
}
