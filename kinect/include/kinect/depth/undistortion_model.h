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

template <typename Scalar_, typename Model_>
  class TwoStepsUndistortionModel : public DepthUndistortionModel<Scalar_>
  {
  public:

    typedef boost::shared_ptr<TwoStepsUndistortionModel> Ptr;
    typedef boost::shared_ptr<const TwoStepsUndistortionModel> ConstPtr;

    TwoStepsUndistortionModel()
    {
      // Do nothing
    }

    virtual ~TwoStepsUndistortionModel()
    {
      // Do nothing
    }

    void setLocalUndistortionModel(const typename Model_::ConstPtr & local)
    {
      local_ = local;
    }

    void setGlobalUndistortionModel(const typename Model_::ConstPtr & global)
    {
      global_ = global;
    }

    const typename Model_::ConstPtr & getLocalUndistortionModel() const
    {
      return local_;
    }

    const typename Model_::ConstPtr & getGlobalUndistortionModel() const
    {
      return global_;
    }

    virtual Scalar_ * dataPtr()
    {
      return NULL;
    }

    virtual const Scalar_ * dataPtr() const
    {
      return NULL;
    }

  protected:

    typename Model_::ConstPtr global_;
    typename Model_::ConstPtr local_;

  };

template <typename Scalar_>
  class TwoStepsUndistortionModelEigen : public TwoStepsUndistortionModel<Scalar_,
                                           DepthUndistortionModelEigen<Scalar_> >,
                                         public DepthUndistortionModelEigen<Scalar_>
  {
  public:

    typedef boost::shared_ptr<TwoStepsUndistortionModelEigen> Ptr;
    typedef boost::shared_ptr<const TwoStepsUndistortionModelEigen> ConstPtr;

    typedef TwoStepsUndistortionModel<Scalar_, DepthUndistortionModelEigen<Scalar_> > Base;

    TwoStepsUndistortionModelEigen()
      : Base()
    {
      // Do nothing
    }

    virtual ~TwoStepsUndistortionModelEigen()
    {
      // Do nothing
    }

    virtual void undistort(typename Types_<Scalar_>::Point3 & point) const
    {
      assert(Base::local_ and Base::global_);
      Base::local_->undistort(point);
      Base::global_->undistort(point);
    }

    virtual void undistort(typename Types_<Scalar_>::Point3Matrix & cloud) const
    {
      assert(Base::local_ and Base::global_);
      Base::local_->undistort(cloud);
      Base::global_->undistort(cloud);
    }

    virtual typename DepthUndistortionModelEigen<Scalar_>::Ptr clone() const
    {
      TwoStepsUndistortionModelEigen::Ptr clone = boost::make_shared<TwoStepsUndistortionModelEigen>();
      clone->setLocalUndistortionModel(Base::local_->clone());
      clone->setGlobalUndistortionModel(Base::global_->clone());
      return clone;
    }

  };

template <typename Scalar_, typename PCLPoint_>
  class TwoStepsUndistortionModelPCL : public TwoStepsUndistortionModel<Scalar_,
                                         DepthUndistortionModelPCL<Scalar_, PCLPoint_> >,
                                       public DepthUndistortionModelPCL<Scalar_, PCLPoint_>
  {
  public:

    typedef boost::shared_ptr<TwoStepsUndistortionModelPCL> Ptr;
    typedef boost::shared_ptr<const TwoStepsUndistortionModelPCL> ConstPtr;

    typedef TwoStepsUndistortionModel<Scalar_, DepthUndistortionModelPCL<Scalar_, PCLPoint_> > Base;

    TwoStepsUndistortionModelPCL()
      : Base()
    {
      // Do nothing
    }

    virtual ~TwoStepsUndistortionModelPCL()
    {
      // Do nothing
    }

    virtual void undistort(PCLPoint_ & point) const
    {
      assert(Base::local_ and Base::global_);
      Base::local_->undistort(point);
      Base::global_->undistort(point);
    }

    virtual void undistort(pcl::PointCloud<PCLPoint_> & cloud) const
    {
      assert(Base::local_ and Base::global_);
      Base::local_->undistort(cloud);
      Base::global_->undistort(cloud);
    }

    virtual typename DepthUndistortionModelPCL<Scalar_, PCLPoint_>::Ptr clone() const
    {
      TwoStepsUndistortionModelPCL::Ptr clone = boost::make_shared<TwoStepsUndistortionModelPCL>();
      clone->setLocalUndistortionModel(Base::local_->clone());
      clone->getGlobalUndistortionModel(Base::global_->clone());
      return clone;
    }

  };

} /* namespace calibration */
#endif /* KINECT_DEPTH_DISTORTION_MODEL_H_ */
