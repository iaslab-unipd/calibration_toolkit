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

template <typename Scalar_>
  class DepthUndistortionModelFit
  {
  public:

    typedef boost::shared_ptr<DepthUndistortionModelFit> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionModelFit> ConstPtr;

    typedef std::vector<std::pair<Scalar_, Scalar_> > PointDistorsionBin;

    class AccumulationBin
    {
    public:

      AccumulationBin()
        : sum_(Types_<Scalar_>::Point3::Zero()),
          n_(0)
      {
        // Do nothing
      }

      void reset()
      {
        sum_ = Types_<Scalar_>::Point3::Zero();
        n_ = 0;
      }

      AccumulationBin & operator +=(const typename Types_<Scalar_>::Point3 & point)
      {
        sum_ += point;
        ++n_;
        return *this;
      }

      bool isEmpty()
      {
        return n_ == 0;
      }

      typename Types_<Scalar_>::Point3 average()
      {
        return sum_ / Scalar_(n_);
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

      typename Types_<Scalar_>::Point3 sum_;
      int n_;

    };

  };

template <typename Scalar_>
  class DepthUndistortionModelFitEigen : public DepthUndistortionModelEigen<Scalar_>
  {
  public:

    typedef boost::shared_ptr<DepthUndistortionModelFitEigen> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionModelFitEigen> ConstPtr;

    virtual ~DepthUndistortionModelFitEigen()
    {
      // Do nothing
    }

    virtual void accumulateCloud(const typename Types_<Scalar_>::Point3Matrix & cloud) = 0;

    virtual void accumulateCloud(const typename Types_<Scalar_>::Point3Matrix & cloud,
                                 const std::vector<int> & indices) = 0;

    virtual void accumulatePoint(const typename Types_<Scalar_>::Point3 & point) = 0;

    virtual void addPoint(const typename Types_<Scalar_>::Point3 & point,
                          const typename Types_<Scalar_>::Plane & plane) = 0;

    virtual void addAccumulatedPoints(const typename Types_<Scalar_>::Plane & plane) = 0;

    virtual void update() = 0;

  };

template <typename Scalar_, typename PCLPoint_>
  class DepthUndistortionModelFitPCL : public DepthUndistortionModelPCL<Scalar_, PCLPoint_>
  {
  public:

    typedef boost::shared_ptr<DepthUndistortionModelFitPCL> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionModelFitPCL> ConstPtr;

    virtual ~DepthUndistortionModelFitPCL()
    {
      // Do nothing
    }

    virtual void accumulateCloud(const pcl::PointCloud<PCLPoint_> & cloud) = 0;

    virtual void accumulateCloud(const pcl::PointCloud<PCLPoint_> & cloud,
                                 const std::vector<int> & indices) = 0;

    virtual void accumulatePoint(const PCLPoint_ & point) = 0;

    virtual void addPoint(const PCLPoint_ & point,
                          const typename Types_<Scalar_>::Plane & plane) = 0;

    virtual void addAccumulatedPoints(const typename Types_<Scalar_>::Plane & plane) = 0;

    virtual void update() = 0;

  };

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_DEPTH_UNDISTORTION_MODEL_FIT_H_ */
