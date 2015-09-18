/**
 * This file is part of LSD-SLAM.
 *
 * Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
 * For more information see <http://vision.in.tum.de/lsdslam>
 *
 * LSD-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LSD-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>

#include "util/settings.h"

#include "DepthEstimation/DepthMapPixelHypothesis.h"

namespace lsd_slam {

cv::Vec3b DepthMapPixelHypothesis::getVisualizationColor(int lastFrameID) const {

  if (debugDisplay == 0 || debugDisplay == 1) {
    float id = (debugDisplay == 0) ?
               idepth_smoothed : idepth;

    // depth is not valid
    if (id < 0)
      return cv::Vec3b(255,255,255);

    // rainbow between 0 and 4
    float r = std::abs(0 - id) * 255 / 1.0;
    float g = std::abs(1 - id) * 255 / 1.0;
    float b = std::abs(2 - id) * 255 / 1.0;

    uchar rc = r > 255 ? 255 : r;
    uchar gc = g > 255 ? 255 : g;
    uchar bc = b > 255 ? 255 : b;

    return cv::Vec3b(255 - rc, 255 - gc, 255 - bc);
  }

  // plot validity counter
  if (debugDisplay == 2) {
    float f = validity_counter * (255.0 / (VALIDITY_COUNTER_MAX_VARIABLE + VALIDITY_COUNTER_MAX));
    uchar v = f < 0 ? 0 : (f > 255 ? 255 : f);
    return cv::Vec3b(0, v, v);
  }

  // plot var
  if (debugDisplay == 3 || debugDisplay == 4) {
    float idv = (debugDisplay == 3) ?
                idepth_smoothed : idepth_var;

    float var = - 0.5 * log10(idv);

    var *= 255*0.333;
    if (var > 255) var = 255;
    if (var < 0)
      return cv::Vec3b(0, 0, 255);

    return cv::Vec3b(255 - var, var, 0); // bw
  }

  // plot skip
  if (debugDisplay == 5) {
    float f = (nextStereoFrameMinID - lastFrameID) * (255.0 / 100);
    uchar v = f < 0 ? 0 : (f > 255 ? 255 : f);
    return cv::Vec3b(v, 0, v);
  }

  return cv::Vec3b(255, 255, 255);
}

}
