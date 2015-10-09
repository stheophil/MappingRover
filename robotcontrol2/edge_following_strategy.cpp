//
//  edge_following_strategy.cpp
//  robotcontrol2
//
//  Created by Sebastian Theophil on 29.09.15.
//  Copyright © 2015 Sebastian Theophil. All rights reserved.
//

#include "edge_following_strategy.h"
#include <opencv2/imgproc.hpp>
#include <iostream>

namespace rbt {
    boost::optional<SRobotCommand> CEdgeFollowingStrategy::update(point<double> const& ptfPrev, point<double> const& ptf,
                                                                  double fYawPrev, double fYaw,
                                                                  ECommand ecmdLast,
                                                                  COccupancyGrid const& occgrid) {
        // Threshold first, converting eroded map to black & white. Decision to drive to a position is essentially binary.
        // Either we can drive someplace or we can't.
        cv::threshold(occgrid.ErodedMap(), m_matnMapThreshold, /* pixels >= */ 255*0.4, /* are set to */ 255, cv::THRESH_BINARY);
        
        int const nMaxExplorationDistance = 25 /*cm*/ / occgrid.m_nScale; // max pixel distance from eroded obstacle
        double const fExplorationDistanceTolerance = 1.5;
        auto const ptn = occgrid.toGridCoordinates(ptf);
        auto const ptnPrev = occgrid.toGridCoordinates(ptfPrev);
        
        // Update the mask containing the robot's path
        m_matnMapPathMask.create(m_matnMapThreshold.size(), CV_8UC1);
        
        // Draw a line along path with thickness 3*nMaxExplorationDistance.
        // We try to path obstacles at a distance <= nMaxExplorationDistance.
        // We count points at 1.5*nMaxExplorationDistance as visited to account for errors.
        cv::line(m_matnMapPathMask, ptnPrev, ptn, 1, /*thickness*/ 2*fExplorationDistanceTolerance*nMaxExplorationDistance);
        
        // Draw recognized features for debugging
        cvtColor(occgrid.ErodedMap(), m_matrgbMapFeatures, CV_GRAY2RGB);
        m_matrgbMapFeatures.setTo(cv::Scalar(0,0,255), m_matnMapPathMask);
        if(rbt::point<int>::invalid() != m_ptnTarget) {
            cv::line(m_matrgbMapFeatures, ptn, m_ptnTarget, cv::Scalar(255,0,0), /*thickness*/ 1);
        }
        
        // New control command
        if(rbt::point<int>::invalid() == m_ptnTarget) {
            if(state::stopped == m_estate) { // Make 360 degree turn
                m_estate = state::start_turning;
                std::cout << "Make 360 turn to " << fYaw << std::endl;
                return c_rcmdTurn360;
            } else if(state::start_turning == m_estate) {
                if(ecmdTURN360==ecmdLast) {
                    m_estate = state::turning;
                    std::cout << "Started 360 turn." << std::endl;
                }
            } else if(state::turning == m_estate) {
                if(ecmdSTOP==ecmdLast) {
                    m_estate = state::stopped_after_turn;
                    std::cout << "Stopped 360 turn." << std::endl;
                }
            } else { // else state::stopped_after_turn, try to find new target, else stay still.
                assert(state::stopped_after_turn == m_estate);
                FindNewTarget(ptf, occgrid, nMaxExplorationDistance);
                std::cout << "FindNewTarget after 360 turn." << std::endl;
                if(rbt::point<int>::invalid() == m_ptnTarget) std::cout << "No new target found!" << std::endl;
                return c_rcmdStop;
            }
        } else {
            auto const szMove = m_ptnTarget - ptn;
            auto const fYawTarget = std::atan2(szMove.y, szMove.x);
            
            if(state::stopped == m_estate || state::stopped_after_turn==m_estate) {
                m_estate = state::start_turning;
                std::cout << "Turn to " << fYawTarget << std::endl;
                return RobotCommandTurn(fYawTarget);
            } else if(state::start_turning == m_estate) {
                if(ecmdTURN==ecmdLast) {
                    m_estate = state::turning;
                    std::cout << "Turn started." << std::endl;
                }
            } else if(state::turning == m_estate) {
                // start moving when heading is correct
                if(ecmdSTOP==ecmdLast) {
                    std::cout << "Stopped turn." << std::endl;
                    m_estate = state::moving;
                    return c_rcmdForward;
                }
            } else {
                assert(state::moving == m_estate);
                
                // Don't drive into obstacle
                // TODO: The    robot stops before an obstacle, updates path,
                // the map changes and the robot position in the eroded
                // map turns black, Find best path to safe terrain.
                // Build distance map on original map instead of erosion?
                double const fLookahead = 20; // cm
                cv::LineIterator itpt(m_matnMapThreshold,
                                      ptn,
                                      occgrid.toGridCoordinates(ptf + rbt::size<double>::fromAngleAndDistance(fYaw, fLookahead)));
                for(int i = 0; i < itpt.count; ++i, ++itpt) {
                    if(!m_matnMapThreshold.at<std::uint8_t>(itpt.pos())) {
                        m_ptnTarget = rbt::point<int>::invalid();
                        m_estate = state::stopped;
                        return c_rcmdStop;
                    }
                }
                
                // Reached target
                // Not past closest point to m_ptnTarget on current trajectory
                auto szPath = rbt::size<double>::fromAngleAndDistance(fYaw, 1.0);
                auto szTarget = rbt::size<double>(szMove);
                if(szPath * szTarget<=0) {
                    m_ptnTarget = rbt::point<int>::invalid();
                    m_estate = state::stopped;
                    return c_rcmdStop;
                }
                
                return c_rcmdForward;
            }
        }
        return boost::none;
    }
    
    void CEdgeFollowingStrategy::FindNewTarget(point<double> const& ptf, COccupancyGrid const& occgrid, int const nMaxExplorationDistance ) {
        assert(rbt::point<int>::invalid() == m_ptnTarget);
        // If there is no target, find new target to go to.
        // Strategy 1: Drive in closely past obstacles to scan them. Sonar sensors are very imprecise at large distances
        
        // Calculate distances to obstacles
        distanceTransform(m_matnMapThreshold, m_matfMapDistance, CV_DIST_L2, CV_DIST_MASK_PRECISE);
        
        // Ignore visited points in distance map
        m_matfMapDistance.setTo(std::numeric_limits<float>::max(), m_matnMapPathMask);
        
        // Calculate optimal angle to scan obstacles closely
        double const fLookahead = 400;
        auto const ptn = occgrid.toGridCoordinates(ptf);
        
        interval<rbt::point<int>> intvlptnBest;
        double fValueBest = std::numeric_limits<double>::lowest();
        
        for(int i=0; i<360; i++) {
            auto ptnTo = occgrid.toGridCoordinates(rbt::size<double>::fromAngleAndDistance(M_PI*i/180, fLookahead) + ptf);
            
            cv::LineIterator itpt(m_matfMapDistance, ptn, ptnTo);
            interval<rbt::point<int>> intvlnptn(rbt::point<int>::invalid(), rbt::point<int>::invalid());
            
            for(int i = 0; i < itpt.count; ++i, ++itpt) {
                rbt::point<int> const ptnLine(itpt.pos());
                // The distance map is overwritten with the path taken by the robot
                // Use it only for scoring, not for collision detection.
                float const fDistance = m_matfMapDistance.at<float>(itpt.pos());
                bool const bOccupied = m_matnMapThreshold.at<std::uint8_t>(itpt.pos())==0;
                assert(!bOccupied || fDistance <= 0 || fDistance==std::numeric_limits<float>::max());
                
                if(rbt::point<int>::invalid() != intvlnptn.begin) {
                    intvlnptn.end = ptnLine;
                    if(bOccupied || nMaxExplorationDistance<fDistance) break;
                } else {
                    if(0<fDistance && fDistance<=nMaxExplorationDistance) {
                        intvlnptn.begin = ptnLine;
                        intvlnptn.end = ptnLine;
                    } else if(bOccupied) {
                        break;
                    }
                }
            }
            
            if(rbt::point<int>::invalid() != intvlnptn.begin) {
                auto const fValue = rbt::numeric_cast<double>((intvlnptn.end - intvlnptn.begin).SqrAbs())
                    / std::max((intvlnptn.begin - ptn).SqrAbs(), 1);
                
                if(fValueBest<fValue) {
                    fValueBest = fValue;
                    intvlptnBest = intvlnptn;
                }
            }
        }
        
        if(std::numeric_limits<double>::lowest()<fValueBest) {
            m_ptnTarget = intvlptnBest.end;
        }
        // TODO: Strategy 1 is essentially a local greedy algorithm that follows the next best path
        // Once all local paths are visited, there can still be unexplored parts of the map further
        // away. Find those using Dijkstra?
    }
}