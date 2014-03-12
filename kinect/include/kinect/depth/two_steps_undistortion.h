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

#ifndef KINECT_DEPTH_TWO_STEPS_UNDISTORTION_H_
#define KINECT_DEPTH_TWO_STEPS_UNDISTORTION_H_

#include <calibration_common/depth/traits.h>
#include <kinect/depth/two_steps_model.h>

namespace calibration
{

template <typename ScalarT_, typename LocalT_, typename GlobalT_>
  class TwoStepsUndistortionImpl_
  {
  public:

    typedef boost::shared_ptr<TwoStepsUndistortionImpl_> Ptr;
    typedef boost::shared_ptr<const TwoStepsUndistortionImpl_> ConstPtr;

    typedef ScalarT_ Scalar;
    typedef typename LocalT_::Model LocalModel;
    typedef typename GlobalT_::Model GlobalModel;

    typedef TwoStepsModel<ScalarT_, LocalModel, GlobalModel> Model;

    TwoStepsUndistortionImpl_()
    {
      // Do nothing
    }

    explicit TwoStepsUndistortionImpl_(const typename Model::ConstPtr & model)
      : model_(model),
        local_(boost::make_shared<LocalT_>(model_->localModel())),
        global_(boost::make_shared<GlobalT_>(model_->globalModel()))
    {
      // Do nothing
    }

    virtual ~TwoStepsUndistortionImpl_()
    {
      // Do nothing
    }

    void setModel(const typename Model::Ptr & model)
    {
      model_ = model;
      local_ = boost::make_shared<LocalT_>(model_->localModel());
      global_ = boost::make_shared<GlobalT_>(model_->globalModel());
    }

    typename Model::Ptr model() const
    {
      return model_;
    }

    virtual void undistort(const typename Types<Scalar>::Point2 & point_sphere,
                           Scalar & z) const
    {
      assert(model_);
      model_->localModel()->undistort(point_sphere, z);
      model_->globalModel()->undistort(point_sphere, z);
    }

    virtual void undistort(size_t x_index,
                           size_t y_index,
                           Scalar & z) const
    {
      assert(model_);
      model_->localModel()->undistort(x_index, y_index, z);
      model_->globalModel()->undistort(x_index, y_index, z);
    }

  protected:

    typename Model::ConstPtr model_;
    typename LocalT_::Ptr local_;
    typename GlobalT_::Ptr global_;

  };

template <typename ScalarT_, typename LocalModelT_, typename GlobalModelT_>
  class DepthUndistortionImpl<TwoStepsModel<ScalarT_, LocalModelT_, GlobalModelT_>, DepthEigen_<ScalarT_> >
    : public TwoStepsUndistortionImpl_<ScalarT_, DepthUndistortionImpl<LocalModelT_, DepthEigen_<ScalarT_> >, DepthUndistortionImpl<GlobalModelT_, DepthEigen_<ScalarT_> > >,
      public DepthUndistortion<DepthEigen_<ScalarT_> >
  {
  public:

    typedef boost::shared_ptr<DepthUndistortionImpl> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionImpl> ConstPtr;

    typedef DepthUndistortionImpl<LocalModelT_, DepthEigen_<ScalarT_> > LocalT;
    typedef DepthUndistortionImpl<GlobalModelT_, DepthEigen_<ScalarT_> > GlobalT;
    typedef TwoStepsUndistortionImpl_<ScalarT_, LocalT, GlobalT> Base;
    typedef typename Base::Model Model;

    typedef DepthUndistortion<DepthEigen_<Scalar> > Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;

    DepthUndistortionImpl()
      : Base()
    {
      // Do nothing
    }

    explicit DepthUndistortionImpl(const typename Model::ConstPtr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~DepthUndistortionImpl()
    {
      // Do nothing
    }

    using Base::undistort;

    virtual void undistort(Point & point) const
    {
      assert(Base::local_ and Base::global_);
      Base::local_->undistort(point);
      Base::global_->undistort(point);
    }

    virtual void undistort(Cloud & cloud) const
    {
      assert(Base::local_ and Base::global_);
      Base::local_->undistort(cloud);
      Base::global_->undistort(cloud);
    }

  };

template <typename ScalarT_, typename PCLPoint_, typename LocalModelT_, typename GlobalModelT_>
  class DepthUndistortionImpl<TwoStepsModel<ScalarT_, LocalModelT_, GlobalModelT_>, DepthPCL_<PCLPoint_> >
    : public TwoStepsUndistortionImpl_<ScalarT_, DepthUndistortionImpl<LocalModelT_, DepthPCL_<PCLPoint_> >, DepthUndistortionImpl<GlobalModelT_, DepthPCL_<PCLPoint_> > >,
      public DepthUndistortion<DepthPCL_<PCLPoint_> >
  {
  public:

    typedef boost::shared_ptr<DepthUndistortionImpl> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionImpl> ConstPtr;

    typedef DepthUndistortionImpl<LocalModelT_, DepthPCL_<PCLPoint_> > LocalT;
    typedef DepthUndistortionImpl<GlobalModelT_, DepthPCL_<PCLPoint_> > GlobalT;
    typedef TwoStepsUndistortionImpl_<ScalarT_, LocalT, GlobalT> Base;
    typedef typename Base::Model Model;

    typedef DepthUndistortion<DepthPCL_<PCLPoint_> > Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;

    DepthUndistortionImpl()
      : Base()
    {
      // Do nothing
    }

    explicit DepthUndistortionImpl(const typename Model::ConstPtr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~DepthUndistortionImpl()
    {
      // Do nothing
    }

    using Base::undistort;

    virtual void undistort(Point & point) const
    {
      assert(Base::local_ and Base::global_);
      Base::local_->undistort(point);
      Base::global_->undistort(point);
    }

    virtual void undistort(Cloud & cloud) const
    {
      assert(Base::local_ and Base::global_);
      Base::local_->undistort(cloud);
      Base::global_->undistort(cloud);
    }

  };

} /* namespace calibration */
#endif /* KINECT_DEPTH_TWO_STEPS_UNDISTORTION_H_ */
