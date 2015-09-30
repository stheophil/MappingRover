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
#include <opencv2/imgproc.hpp>

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
            // TODO: Approximate arc with lines, use cv::LineIterator instead
            auto szFrom = rbt::size<int>::fromAngleAndDistance(m_fAngleFrom, m_fRadius);
            auto szTo = rbt::size<int>::fromAngleAndDistance(m_fAngleTo, m_fRadius);
            
            assert(angularDistance(m_fAngleFrom, m_fAngleTo)<M_PI/2);
            // This can happen because szFrom and szTo are already cast to int coordinates:
            // assert(szFrom.compare(szTo) != 0);
            
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
            for(int y = rectnBound.bottom; y <= rectnBound.top; ++y) { // both-inclusive
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
            // TODO: Use cv::LineIterator instead
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
                        auto nMin = std::min(ptA.y, ptB.y);
                        auto nMax = std::max(ptA.y, ptB.y);
                        for(int y = nMin; y <= nMax; ++y) {
                            foreach(rbt::numeric_cast<int>(ptA.x + (y - ptA.y) / m), y);
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
                for(int x = pairnintvlX.second.begin; x <= pairnintvlX.second.end; ++x) {
                    foreach(rbt::point<int>(x, pairnintvlX.first));
                }
            });
        }
    };

    COccupancyGrid::COccupancyGrid(rbt::size<int> const& szn, int nScale)
    :   m_szn(szn), m_nScale(nScale),
        m_matfMapLogOdds(m_szn.x, m_szn.y, CV_32FC1, 0.0f),
        m_matnMapGreyscale(m_szn.x, m_szn.y, CV_8UC1, 128),
        m_matnMapEroded(m_szn.x, m_szn.y, CV_8UC1, 128)
    {
        assert(0==szn.x%2 && 0==szn.y%2);
    }

    
    void COccupancyGrid::update(point<double> const& ptf, double fYaw, int nAngle, int nDistance) {
        assert(nAngle==0 || std::abs(nAngle)==90);
        auto const fAngleSonar = fYaw + M_PI_2 * rbt::sign(nAngle);
        auto const ptnGrid = toGridCoordinates(ptf);
        
        auto const fSqrMaxDistance = rbt::sqr(c_fSonarMaxDistance/m_nScale);
        auto const fSqrMeasuredDistance = rbt::sqr((nDistance - c_fSonarDistanceTolerance/2)/m_nScale);
        
        auto UpdateMap = [this](rbt::point<int> const& pt, float fValue) {
            m_matfMapLogOdds.at<float>(pt.y, pt.x) = fValue;
            auto const nColor = rbt::numeric_cast<std::uint8_t>(1.0 / ( 1.0 + std::exp( fValue )) * 255);
            m_matnMapGreyscale.at<std::uint8_t>(pt.y, pt.x) = nColor;
        };
        
        SArc arc{ptnGrid,
            fAngleSonar - c_fSonarOpeningAngle/2,
            fAngleSonar + c_fSonarOpeningAngle/2,
            (nDistance + c_fSonarDistanceTolerance/2)/m_nScale
        };
        arc.for_each_pixel([&](point<int> const& pt, double fSqrDistance) {
            if(fSqrDistance < fSqrMaxDistance) {
                auto const fInverseSensorModel = fSqrDistance < fSqrMeasuredDistance
                    ? -0.5 // free
                    : (100.0 / m_nScale) / std::sqrt(fSqrDistance); // occupied

                UpdateMap(pt, m_matfMapLogOdds.at<float>(pt.y, pt.x) + fInverseSensorModel); // - prior which is 0
            }
        });
        
        // Clear position of robot itself
        SRotatedRect rectRobot{ptnGrid, rbt::size<double>(c_nRobotWidth, c_nRobotHeight)/m_nScale, fYaw};
        rectRobot.for_each_pixel([&](rbt::point<int> const& pt) { UpdateMap(pt, -100); });
        
        // Erode image
        // A pixel p in imageEroded is marked free when the robot centered at p does not occupy an occupied pixel in self.image
        // i.e. the pixel p has the maximum value of the surrounding pixels inside the diameter defined by the robot's size
        // We overestimate robot size by taking robot diagonal        
        static const int s_nKernelDiameter =
            rbt::numeric_cast<int>(std::ceil(std::sqrt(rbt::size<int>(c_nRobotWidth, c_nRobotHeight).SqrAbs()) / m_nScale));
        static const cv::Mat s_matnKernel = cv::Mat(s_nKernelDiameter, s_nKernelDiameter, CV_8UC1, 1);
        cv::erode(m_matnMapGreyscale, m_matnMapEroded, s_matnKernel);
    }
    
    point<int> COccupancyGrid::toGridCoordinates(point<double> const& pt) const {
        return point<int>(pt/m_nScale) + m_szn/2;
    }

    point<int> COccupancyGrid::toWorldCoordinates(point<int> const& pt) const {
        return (pt - m_szn/2) * m_nScale;
    }
}
