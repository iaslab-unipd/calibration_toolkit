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

template <typename ModelT_>
  class PolynomialMatrix_
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrix_> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrix_> ConstPtr;

    typedef ModelT_ Model;

    PolynomialMatrix_()
      : model_()
    {
      // Do nothing
    }

    explicit PolynomialMatrix_(const typename Model::Ptr & model)
      : model_(model)
    {
      // Do nothing
    }

    inline void setModel(const typename Model::Ptr & model)
    {
      model_ = model;
    }

    inline typename Model::Ptr model() const
    {
      return model_;
    }

  protected:

    typename Model::Ptr model_;

  };

template <typename ModelT_, typename ScalarT_>
  class PolynomialMatrixEigen : public PolynomialMatrix_<ModelT_>,
                                public DepthUndistortion<DepthEigen_<ScalarT_> >
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixEigen> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixEigen> ConstPtr;

    typedef ScalarT_ Scalar;

    typedef PolynomialMatrix_<ModelT_> Base;
    typedef typename Base::Model Model;

    typedef DepthUndistortion<DepthEigen_<Scalar> > Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;

    PolynomialMatrixEigen()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialMatrixEigen(const typename Model::Ptr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixEigen()
    {
      // Do nothing
    }

    virtual void undistort(Size1 x_index,
                           Size1 y_index,
                           Point & point) const
    {
      assert(Base::model());
      if (point.hasNaN())
        return;

      Scalar z = point.z();
      Base::model()->undistort(x_index, y_index, z);
      point *= z / point.z();
    }

    virtual void undistort(const Size2 & index,
                           Point & point) const
    {
      assert(Base::model());
      if (point.hasNaN())
        return;

      Scalar z = point.z();
      Base::model()->undistort(index, z);
      point *= z / point.z();
    }

    virtual void undistort(Cloud & cloud) const
    {
      assert(Base::model());
      assert((cloud.size() == Base::model()->imageSize()).all());

      for (Size1 i = 0; i < cloud.size().x(); ++i)
      {
        for (Size1 j = 0; j < cloud.size().y(); ++j)
        {
          if (not cloud(i, j).allFinite()/* or std::abs(cloud(i, j).z()) < 0.01*/)
            continue;

          Scalar z = cloud(i, j).z();
          Base::model()->undistort(i, j, z);
          cloud(i, j) /= cloud(i, j).z();
          cloud(i, j) *= z;
        }
      }
    }

  };

template <typename ModelT_, typename ScalarT_, typename PCLPointT_>
  class PolynomialMatrixPCL : public PolynomialMatrix_<ModelT_>,
                              public DepthUndistortion<DepthPCL_<PCLPointT_> >
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixPCL> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixPCL> ConstPtr;

    typedef ScalarT_ Scalar;

    typedef PolynomialMatrix_<ModelT_> Base;
    typedef typename Base::Model Model;

    typedef DepthUndistortion<DepthPCL_<PCLPointT_> > Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;

    PolynomialMatrixPCL()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialMatrixPCL(const typename Model::Ptr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixPCL()
    {
      // Do nothing
    }

    virtual void undistort(Size1 x_index,
                           Size1 y_index,
                           Point & point) const
    {
      assert(Base::model());
      if (not pcl::isFinite(point))
        return;

      Scalar z = Scalar(point.z);
      Base::model()->undistort(x_index, y_index, z);
      float k = static_cast<float>(z) / point.z;

      point.x *= k;
      point.y *= k;
      point.z *= k;
    }

    virtual void undistort(const Size2 & index,
                           Point & point) const
    {
      assert(Base::model());
      if (not pcl::isFinite(point))
        return;

      Scalar z = Scalar(point.z);
      Base::model()->undistort(index, z);
      float k = static_cast<float>(z) / point.z;

      point.x *= k;
      point.y *= k;
      point.z *= k;
    }

    virtual void undistort(Cloud & cloud) const
    {
      assert(Base::model());

      for (Size1 i = 0; i < cloud.width; ++i)
      {
        for (Size1 j = 0; j < cloud.height; ++j)
        {
          if (not pcl::isFinite(cloud(i, j))/* or std::abs(cloud(i, j).z) < 0.01*/)
            continue;

          Scalar z = Scalar(cloud(i, j).z);
          Base::model()->undistort(i, j, z);
          float k = static_cast<float>(z) / cloud(i, j).z;

          cloud(i, j).x *= k;
          cloud(i, j).y *= k;
          cloud(i, j).z *= k;
        }
      }
    }

  };

} /* namespace calibration */
#endif /* KINECT_DEPTH_POLYNOMIAL_MATRIX_H_ */
