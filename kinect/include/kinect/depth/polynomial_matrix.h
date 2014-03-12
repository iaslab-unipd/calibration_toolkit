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

#ifndef KINECT_DEPTH_POLYNOMIAL_MATRIX_H_
#define KINECT_DEPTH_POLYNOMIAL_MATRIX_H_

#include <kinect/depth/polynomial_matrix_model.h>

namespace calibration
{

template <typename PolynomialT_>
  class PolynomialMatrixImpl_
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixImpl_> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixImpl_> ConstPtr;

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;

    typedef PolynomialMatrixProjectedModel<PolynomialT_> Model;

    PolynomialMatrixImpl_()
      : model_()
    {
      // Do nothing
    }

    explicit PolynomialMatrixImpl_(const typename Model::Ptr & model)
      : model_(model)
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixImpl_()
    {
      // Do nothing
    }

    void setModel(const typename Model::Ptr & model)
    {
      model_ = model;
    }

    typename Model::Ptr model() const
    {
      return model_;
    }

  protected:

    typename Model::Ptr model_;

  };

template <typename PolynomialT_, typename ScalarT_>
  class DepthUndistortionImpl<PolynomialMatrixProjectedModel<PolynomialT_>, DepthEigen_<ScalarT_> > : public PolynomialMatrixImpl_<PolynomialT_>,
                                                                                                      public DepthUndistortion<DepthEigen_<ScalarT_> >
  {
  public:

    typedef boost::shared_ptr<DepthUndistortionImpl> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionImpl> ConstPtr;

    typedef ScalarT_ Scalar;

    typedef PolynomialMatrixImpl_<PolynomialT_> Base;
    typedef typename Base::Model Model;
    typedef typename Model::Point2 Point2;

    typedef DepthUndistortion<DepthEigen_<Scalar> > Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;

    DepthUndistortionImpl()
      : Base()
    {
      // Do nothing
    }

    explicit DepthUndistortionImpl(const typename Model::Ptr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~DepthUndistortionImpl()
    {
      // Do nothing
    }

    virtual void undistort(Point & point) const
    {
      assert(Base::model_);
      Scalar z = point.z();
      Base::model_->undistort(project(point), z);
      point *= z / point.z();
    }

    virtual void undistort(Cloud & cloud) const
    {
      assert(Base::model_);
      for (size_t i = 0; i < cloud.size(); ++i)
      {
        Scalar z = cloud[i].z();
        Base::model_->undistort(project(cloud[i]), z);
        cloud[i] *= z / cloud[i].z();
      }
    }

    static Point2 project(const Point & point)
    {
      return Point2(point.x() / point.z(), point.y() / point.z());
    }

  };

template <typename PolynomialT_, typename PCLPointT_>
  class DepthUndistortionImpl<PolynomialMatrixProjectedModel<PolynomialT_>, DepthPCL_<PCLPointT_> > : public PolynomialMatrixImpl_<PolynomialT_>,
                                                                                                      public DepthUndistortion<DepthPCL_<PCLPointT_> >
  {
  public:

    typedef boost::shared_ptr<DepthUndistortionImpl> Ptr;
    typedef boost::shared_ptr<const DepthUndistortionImpl> ConstPtr;

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;

    typedef PolynomialMatrixImpl_<PolynomialT_> Base;
    typedef typename Base::Model Model;
    typedef typename Model::Point2 Point2;

    typedef DepthUndistortion<DepthPCL_<PCLPointT_> > Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;

    DepthUndistortionImpl()
      : Base()
    {
      // Do nothing
    }

    explicit DepthUndistortionImpl(const typename Model::Ptr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~DepthUndistortionImpl()
    {
      // Do nothing
    }

    virtual void undistort(Point & point) const
    {
      assert(Base::model_);
      if (not pcl::isFinite(point))
        return;

      Scalar z = Scalar(point.z);
      Base::model_->undistort(project(point), z);
      float k = static_cast<float>(z) / point.z;

      point.x *= k;
      point.y *= k;
      point.z *= k;
    }

    virtual void undistort(Cloud & cloud) const
    {
      for (size_t i = 0; i < cloud.size(); ++i)
        undistort(cloud.points[i]);
    }

    static Point2 project(const Point & point)
    {
      return Model::project(Scalar(point.x), Scalar(point.y), Scalar(point.z));
    }

  };

} /* namespace calibration */
#endif /* KINECT_DEPTH_POLYNOMIAL_MATRIX_H_ */
