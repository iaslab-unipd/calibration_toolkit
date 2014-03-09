/*
 *  Copyright (c) 2013-2014, Filippo Basso <bassofil@dei.unipd.it>
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

#ifndef CALIBRATION_COMMON_ALGORITHMS_PLANE_TO_PLANE_CALIBRATION_H_
#define CALIBRATION_COMMON_ALGORITHMS_PLANE_TO_PLANE_CALIBRATION_H_

#include <vector>
#include <calibration_common/objects/globals.h>

namespace calibration
{

struct PlanePair
{
  PlanePair()
  {
  }

  PlanePair(Plane plane_1,
            Plane plane_2)
    : plane_1_(plane_1),
      plane_2_(plane_2)
  {
  }

  Plane plane_1_;
  Plane plane_2_;
};

/**
 * c++ implementation of [1].
 *
 * References:
 * [1] R. Unnikrishnan and M. Hebert, “Fast extrinsic calibration of a laser rangefinder to a camera,” Tech. Rep., 2005.
 */
class PlaneToPlaneCalibration
{
public:

  void addPair(const PlanePair & pair)
  {
    plane_pair_vec_.push_back(pair);
  }

  void addPair(const Plane & plane_1,
               const Plane & plane_2)
  {
    addPair(PlanePair(plane_1, plane_2));
  }

  int getPairNumber()
  {
    return plane_pair_vec_.size();
  }

  Transform estimateTransform()
  {
    return estimateTransform(plane_pair_vec_);
  }

  static Transform estimateTransform(const std::vector<PlanePair> & plane_pair_vec);

protected:

  std::vector<PlanePair> plane_pair_vec_;

};

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_ALGORITHMS_PLANE_TO_PLANE_CALIBRATION_H_ */
