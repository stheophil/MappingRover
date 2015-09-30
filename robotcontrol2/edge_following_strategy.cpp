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
    void CEdgeFollowingStrategy::update(point<double> const& ptf, double fYaw, COccupancyGrid const& occgrid) {
        // Threshold first, converting image to black & white. Decision to drive to a position is essentially binary.
        // Either we can drive someplace or we can't.
        cv::threshold(occgrid.ErodedMap(), m_matnMapEdges, /* pixels >= */ 255*0.4, /* are set to */ 255, cv::THRESH_BINARY);
        
        // Recognize edges in eroded map.
        // Threshold factors determined empirically to work well with greyscale image.
        // They don't matter with thresholded binary image
        cv::Canny(m_matnMapEdges, m_matnMapEdges, 100, 300);

        // Turn edges into lines. The robot must follow these lines to scan the walls.
        std::vector<cv::Vec4i> vecline;
        cv::HoughLinesP(m_matnMapEdges,
                        vecline,
                        1, // resolution of r in px
                        CV_PI/180, // resolution of rho in rad
                        15); // min number of votes for a line, 15-20 seems to work ok

        // Draw recognized features for debugging
        cvtColor(occgrid.GreyscaleMap(), m_matrgbMapFeatures, CV_GRAY2RGB);
        boost::for_each(vecline, [&](cv::Vec4i const& line) {
            cv::line(m_matrgbMapFeatures, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0,0,255), /*thickness*/ 1);
        });
    }
}