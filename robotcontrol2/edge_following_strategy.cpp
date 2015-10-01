//
//  edge_following_strategy.cpp
//  robotcontrol2
//
//  Created by Sebastian Theophil on 29.09.15.
//  Copyright © 2015 Sebastian Theophil. All rights reserved.
//

#include "edge_following_strategy.h"
#include <opencv2/imgproc.hpp>

namespace rbt {
    void CEdgeFollowingStrategy::update(point<double> const& ptfPrev, point<double> const& ptf, double fYaw, COccupancyGrid const& occgrid) {
        // Threshold first, converting eroded map to black & white. Decision to drive to a position is essentially binary.
        // Either we can drive someplace or we can't.
        cv::threshold(occgrid.ErodedMap(), m_matnMapThreshold, /* pixels >= */ 255*0.4, /* are set to */ 255, cv::THRESH_BINARY);
        
        int const nMaxExplorationDistance = 25 /*cm*/ / occgrid.m_nScale; // max pixel distance from eroded obstacle
        auto const ptn = occgrid.toGridCoordinates(ptf);
        auto const ptnPrev = occgrid.toGridCoordinates(ptfPrev);
        
        // Update the mask containing the robot's path
        m_matnMapPathMask.create(m_matnMapThreshold.size(), CV_8UC1);
        
        // Draw a line along path with thickness 3*nMaxExplorationDistance.
        // We try to path obstacles at a distance <= nMaxExplorationDistance.
        // We count points at 1.5*nMaxExplorationDistance as visited to account for errors.
        cv::line(m_matnMapPathMask, ptnPrev, ptn, 1, /*thickness*/ 3*nMaxExplorationDistance);
        
        // Strategy 1: Drive in closely past obstacles to scan them. Sonar sensors are very imprecise at large distances
        
        // Calculate distances to obstacles
        distanceTransform(m_matnMapThreshold, m_matnMapDistance, CV_DIST_L2, CV_DIST_MASK_PRECISE);

        // Ignore visited points in distance map
        m_matnMapDistance.setTo(std::numeric_limits<float>::max(), m_matnMapPathMask);
        
        // Calculate optimal angle to scan obstacles closely
        double const fLookahead = 400;
        point<int> const ptnINVALID(-1, -1);
        
        interval<rbt::point<int>> intvlptnBest;
        double fValueBest = std::numeric_limits<double>::lowest();
        
        for(int i=0; i<360; i++) {
            auto ptnTo = occgrid.toGridCoordinates(rbt::size<double>::fromAngleAndDistance(M_PI*i/180, fLookahead) + ptf);

            cv::LineIterator itpt(m_matnMapDistance, ptn, ptnTo);
            
            interval<rbt::point<int>> intvlnptn(ptnINVALID, ptnINVALID);
            
            for(int i = 0; i < itpt.count; ++i, ++itpt) {
                rbt::point<int> ptnLine(itpt.pos());
                float fDistance = m_matnMapDistance.at<float>(itpt.pos());
                if(ptnINVALID != intvlnptn.begin) {
                    intvlnptn.end = ptnLine;
                    if(fDistance<=0 || nMaxExplorationDistance<fDistance) break;
                } else {
                    if(0<fDistance && fDistance<=nMaxExplorationDistance) {
                        intvlnptn.begin = ptnLine;
                        intvlnptn.end = ptnLine;
                    } else if(fDistance<=0) {
                        break;
                    }
                }
            }
            
            if(ptnINVALID != intvlnptn.begin) {
                auto const fValue = rbt::numeric_cast<double>((intvlnptn.end - intvlnptn.begin).SqrAbs())
                    / std::max((intvlnptn.begin - ptn).SqrAbs(), 1);
                
                if(fValueBest<fValue) {
                    fValueBest = fValue;
                    intvlptnBest = intvlnptn;
                }
            }
        }
        
        // Strategy 2: Find closest uncertain points and drive past them
        
        // Draw recognized features for debugging
        cvtColor(occgrid.ErodedMap(), m_matrgbMapFeatures, CV_GRAY2RGB);
        m_matrgbMapFeatures.setTo(cv::Scalar(0,0,255), m_matnMapPathMask);
        if(std::numeric_limits<double>::lowest()<fValueBest) {
            cv::line(m_matrgbMapFeatures, ptn, intvlptnBest.end, cv::Scalar(255,0,0), /*thickness*/ 1);
        }
    }
}