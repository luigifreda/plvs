/*
Copyright (c) 2015, Helen Oleynikova, ETH Zurich, Switzerland
You can contact the author at <helen dot oleynikova at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef VOLUMETRIC_MAP_BASE_WEIGHING_FUNCTION_H_
#define VOLUMETRIC_MAP_BASE_WEIGHING_FUNCTION_H_

//#include <vector>

namespace volumetric_mapping {

// A base class for weighing functions: each weighing function should take in
// either a point in 3D (PCL), UVD (disparity), or an Eigen point (or
// preferably all 3) and return a weight between 0.0 and 1.0 as confidence
// value for the measurement.
// For example, default behavior would happen with a value of 1.0 for all points
// (and this is the base class implementation).
// In the case of raycasting-built maps, such as octomap, all occupied
// probabilities in the ray of the point will be scaled by this amount.
class PointWeighing {
 public:
  PointWeighing() {}
  virtual ~PointWeighing() {}

  virtual double computeWeightForPoint(double x, double y, double z) const {
    return 1.0;
  }
  virtual double computeWeightForDisparity(unsigned int u, unsigned int v,
                                           double d) const {
    return 1.0;
  }
};

}  // namespace volumetric_mapping

#endif  // VOLUMETRIC_MAP_BASE_WEIGHING_FUNCTION_H_
