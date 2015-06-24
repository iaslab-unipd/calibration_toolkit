/*
 *  Copyright (c) 2015-, Filippo Basso <bassofil@gmail.com>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UNIPD_CALIBRATION_CALIBRATION_ALGORITHMS_PLANE_TO_PLANE_CALIBRATION_H_
#define UNIPD_CALIBRATION_CALIBRATION_ALGORITHMS_PLANE_TO_PLANE_CALIBRATION_H_

#include <calibration_common/base/geometry.h>

namespace unipd
{
namespace calib
{

/**
 * @brief The PlaneToPlaneCalibration class
 * C++ implementation of:
 *    R. Unnikrishnan and M. Hebert, “Fast extrinsic calibration of a laser rangefinder to a camera,” Tech. Rep., 2005.
 */
class PlaneToPlaneCalibration
{
public:

  void
  addPlanePair (const std::pair<Plane3, Plane3> & pair)
  {
    plane_pair_vec_.push_back(pair);
  }

  bool
  canEstimateTransform () const
  {
    return plane_pair_vec_.size() >= 3;
  }

  Transform3
  estimateTransform () const
  {
    return estimateTransform(plane_pair_vec_);
  }

  static Transform3
  estimateTransform (const std::vector<std::pair<Plane3, Plane3>> & plane_pair_vec);

protected:

  std::vector<std::pair<Plane3, Plane3>> plane_pair_vec_;

};

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATION_ALGORITHMS_PLANE_TO_PLANE_CALIBRATION_H_
