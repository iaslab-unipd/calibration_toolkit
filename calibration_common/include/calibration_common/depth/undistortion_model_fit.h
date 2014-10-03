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

#ifndef CALIBRATION_COMMON_DEPTH_UNDISTORTION_MODEL_FIT_H_
#define CALIBRATION_COMMON_DEPTH_UNDISTORTION_MODEL_FIT_H_

#include <Eigen/Dense>
#include <calibration_common/depth/undistortion_model.h>
#include <calibration_common/objects/globals.h>

namespace calibration
{

/**
 * @brief The DepthUndistortionModelFit class
 * @param ModelT_
 * @param DepthT_
 * @param ScalarT_
 */
template <typename DepthT_, typename ScalarT_>
  class DepthUndistortionModelFit
  {
  public:

    typedef boost::shared_ptr<DepthUndistortionModelFit> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionModelFit> ConstPtr;

    typedef ScalarT_ Scalar;

    typedef typename DepthTraits<DepthT_>::Point Point;
    typedef typename DepthTraits<DepthT_>::Cloud Cloud;
    typedef typename Types<Scalar>::Plane Plane;

    virtual ~DepthUndistortionModelFit()
    {
      // Do nothing
    }

    /**
     * @brief accumulateCloud
     * @param cloud
     */
    virtual void accumulateCloud(const Cloud & cloud) = 0;

    /**
     * @brief accumulatePoint
     * @param point
     */
    virtual void accumulatePoint(Size1 x_index,
                                 Size1 y_index,
                                 const Point & point) = 0;

    /**
     * @brief addPoint
     * @param point
     * @param plane
     */
    virtual void addPoint(Size1 x_index,
                          Size1 y_index,
                          const Point & point,
                          const Plane & plane) = 0;

    /**
     * @brief addAccumulatedPoints
     * @param plane
     */
    virtual void addAccumulatedPoints(const Plane & plane) = 0;

    /**
     * @brief update
     */
    virtual void update() = 0;

  };

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_DEPTH_UNDISTORTION_MODEL_FIT_H_ */
