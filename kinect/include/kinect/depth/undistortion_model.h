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

#ifndef KINECT_DEPTH_DISTORTION_MODEL_H_
#define KINECT_DEPTH_DISTORTION_MODEL_H_

#include <calibration_common/objects/globals.h>
#include <calibration_common/base/matrix.h>
#include <calibration_common/base/math.h>

#include <calibration_common/depth/undistortion_model.h>

namespace calibration
{

template <typename Scalar_>
  class TwoStepsUndistortionModelEigen : public DepthUndistortionModel<UndTraitsEigen<Scalar_> >
  {
  public:

    typedef boost::shared_ptr<TwoStepsUndistortionModelEigen> Ptr;
    typedef boost::shared_ptr<const TwoStepsUndistortionModelEigen> ConstPtr;

    typedef DepthUndistortionModel<UndTraitsEigen<Scalar_> > Interface;

    virtual ~TwoStepsUndistortionModelEigen()
    {
      // Do nothing
    }

    virtual void undistort(typename Types_<Scalar_>::Point3 & point) const
    {
      assert(local_ and global_);
      local_->undistort(point);
      global_->undistort(point);
    }

    virtual void undistort(typename Types_<Scalar_>::Point3Matrix & cloud) const
    {
      assert(local_ and global_);
      local_->undistort(cloud);
      global_->undistort(cloud);
    }

    virtual typename Interface::Ptr clone() const
    {
      TwoStepsUndistortionModelEigen::Ptr clone = boost::make_shared<TwoStepsUndistortionModelEigen>();
      clone->setLocalUndistortionModel(local_->clone());
      clone->setGlobalUndistortionModel(global_->clone());
      return clone;
    }

    void setLocalUndistortionModel(const typename Interface::ConstPtr & local)
    {
      local_ = local;
    }

    void setGlobalUndistortionModel(const typename Interface::ConstPtr & global)
    {
      global_ = global;
    }

    const typename Interface::ConstPtr & getLocalUndistortionModel() const
    {
      return local_;
    }

    const typename Interface::ConstPtr & getGlobalUndistortionModel() const
    {
      return global_;
    }

  protected:

    typename Interface::ConstPtr global_;
    typename Interface::ConstPtr local_;

  };

template <typename Scalar_, typename PCLPoint_>
  class TwoStepsUndistortionModelPCL : public DepthUndistortionModel<UndTraitsPCL<Scalar_, PCLPoint_> >
  {
  public:

    typedef boost::shared_ptr<TwoStepsUndistortionModelPCL> Ptr;
    typedef boost::shared_ptr<const TwoStepsUndistortionModelPCL> ConstPtr;

    typedef DepthUndistortionModel<UndTraitsPCL<Scalar_, PCLPoint_> > Interface;

    virtual ~TwoStepsUndistortionModelPCL()
    {
      // Do nothing
    }

    virtual void undistort(PCLPoint_ & point) const
    {
      assert(local_ and global_);
      local_->undistort(point);
      global_->undistort(point);
    }

    virtual void undistort(pcl::PointCloud<PCLPoint_> & cloud) const
    {
      assert(local_ and global_);
      local_->undistort(cloud);
      global_->undistort(cloud);
    }

    virtual typename Interface::Ptr clone() const
    {
      TwoStepsUndistortionModelPCL::Ptr clone = boost::make_shared<TwoStepsUndistortionModelPCL>();
      clone->setLocalUndistortionModel(local_->clone());
      clone->setGlobalUndistortionModel(global_->clone());
      return clone;
    }

    void setLocalUndistortionModel(const typename Interface::ConstPtr & local)
    {
      local_ = local;
    }

    void setGlobalUndistortionModel(const typename Interface::ConstPtr & global)
    {
      global_ = global;
    }

    const typename Interface::ConstPtr & getLocalUndistortionModel() const
    {
      return local_;
    }

    const typename Interface::ConstPtr & getGlobalUndistortionModel() const
    {
      return global_;
    }

  protected:

    typename Interface::ConstPtr global_;
    typename Interface::ConstPtr local_;

  };

} /* namespace calibration */
#endif /* KINECT_DEPTH_DISTORTION_MODEL_H_ */
