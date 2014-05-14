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

#ifndef KINECT_DEPTH_POLYNOMIAL_MATRIX_MODEL_H_
#define KINECT_DEPTH_POLYNOMIAL_MATRIX_MODEL_H_

#include <calibration_common/base/math.h>
#include <calibration_common/base/matrix.h>
#include <calibration_common/depth/undistortion_model.h>

namespace calibration
{

template <typename PolynomialT_>
  class PolynomialMatrixModel;

template <typename PolynomialT_>
  struct ModelTraits<PolynomialMatrixModel<PolynomialT_> >
  {
    typedef PolynomialT_ Poly;
    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;
    typedef EigenMatrix<typename MathTraits<PolynomialT_>::Coefficients> Data;
  };

template <typename PolynomialT_>
  class PolynomialMatrixModel : public DepthUndistortionModel<PolynomialMatrixModel<PolynomialT_> >
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixModel> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixModel> ConstPtr;

    typedef ModelTraits<PolynomialMatrixModel> Traits;
    typedef DepthUndistortionModel<PolynomialMatrixModel> Base;

    typedef typename Traits::Poly Poly;
    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::Data Data;

    typedef typename Data::Element Element;
    typedef typename Data::ConstElement ConstElement;

    PolynomialMatrixModel()
    {
      // Do nothing
    }

    explicit PolynomialMatrixModel(const typename Data::Ptr & data)
      : data_(data)
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixModel()
    {
      // Do nothing
    }

    virtual void setData(const typename Data::Ptr & data)
    {
      data_ = data;
    }

    virtual const typename Data::Ptr & data() const
    {
      return data_;
    }

    Element polynomial(size_t x_index,
                       size_t y_index)
    {
      assert(data_);
      return (*data_)(x_index, y_index);
    }

    const ConstElement polynomial(size_t x_index,
                                  size_t y_index) const
    {
      assert(data_);
      return (*boost::static_pointer_cast<const Data>(data_))(x_index, y_index);
    }

    virtual void undistort(size_t x_index,
                           size_t y_index,
                           Scalar & z) const
    {
      z = Poly::evaluate(polynomial(x_index, y_index), z);
    }

    virtual Scalar * dataPtr()
    {
      assert(data_);
      return data_->container().data();
    }

    virtual const Scalar * dataPtr() const
    {
      assert(data_);
      return data_->container().data();
    }

  protected:

    typename Data::Ptr data_;

  };

template <typename PolynomialT_>
  class PolynomialMatrixProjectedModel;

template <typename PolynomialT_>
  struct ModelTraits<PolynomialMatrixProjectedModel<PolynomialT_> > : public ModelTraits<PolynomialMatrixModel<PolynomialT_> >
  {
    typedef ModelTraits<PolynomialMatrixModel<PolynomialT_> > Base;
    typedef typename Base::Poly Poly;
    typedef typename Base::Scalar Scalar;
    typedef typename Base::Data Data;
  };

template <typename PolynomialT_>
  class PolynomialMatrixProjectedModel : public PolynomialMatrixModel<PolynomialT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixProjectedModel> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixProjectedModel> ConstPtr;

    typedef PolynomialMatrixModel<PolynomialT_> Base;

    typedef typename Base::Scalar Scalar;
    typedef typename Base::Data Data;
    typedef typename Base::Poly Poly;
    typedef typename Base::Element Element;
    typedef typename Base::ConstElement ConstElement;

    typedef typename Types<Scalar>::Point2 Point2;
    typedef Eigen::Array<Scalar, 2, 1> Array2;

    PolynomialMatrixProjectedModel()
      : Base(),
        bin_size_(0.0, 0.0)
    {
      // Do nothing
    }

    explicit PolynomialMatrixProjectedModel(const typename Data::Ptr & data)
      : Base(data),
        bin_size_(0.0, 0.0)
    {
      // Do nothing
    }

    PolynomialMatrixProjectedModel(const Base & other)
      : Base(other),
        zero_(other.zero_),
        bin_size_(other.bin_size_),
        fov_(other.fov_)
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixProjectedModel()
    {
      // Do nothing
    }

    void setFieldOfView(Scalar x,
                        Scalar y)
    {
      assert(Base::data_);
      fov_ = Array2(x, y);
      zero_ = Point2(-std::tan(x / 2), -std::tan(y / 2));
      bin_size_ = -2 * zero_.array() / Array2(Base::data_->xSize(), Base::data_->ySize());
    }

    Scalar fieldOfViewX() const
    {
      return fov_.x();
    }

    Scalar fieldOfViewY() const
    {
      return fov_.y();
    }

    void getIndex(const Point2 & point_proj,
                  size_t & x_index,
                  size_t & y_index) const
    {
      assert(Base::data_);
      assert(bin_size_.x() > 0 and bin_size_.y() > 0);
      Point2 diff = point_proj - zero_;
      x_index = diff.x() < 0 ? 0 : size_t(std::min(Base::data_->xSize() - 1.0, std::floor(diff.x() / bin_size_.x())));
      y_index = diff.y() < 0 ? 0 : size_t(std::min(Base::data_->ySize() - 1.0, std::floor(diff.y() / bin_size_.y())));
    }

    using Base::polynomial;

    Element polynomial(const Point2 & point_proj)
    {
      size_t x_index, y_index;
      getIndex(point_proj, x_index, y_index);
      return Base::polynomial(x_index, y_index);
    }

    const ConstElement polynomial(const Point2 & point_proj) const
    {
      size_t x_index, y_index;
      getIndex(point_proj, x_index, y_index);
      return Base::polynomial(x_index, y_index);
    }

    using Base::undistort;

//    TODO create two different classes.
//    virtual void undistort(const Point2 & point_proj,
//                           Scalar & z) const
//    {
//      z = Poly::evaluate(polynomial(point_proj), z);
//    }

    virtual void undistort(const Point2 & point_proj,
                           Scalar & z) const
    {
      assert(Base::data_);
      assert(bin_size_.x() > 0 and bin_size_.y() > 0);
      Point2 diff = point_proj - zero_ - 0.5 * bin_size_.matrix();

      size_t x_index[2];
      size_t y_index[2];
      Scalar x_weight[2] = {0.5, 0.5};
      Scalar y_weight[2] = {0.5, 0.5};

      Scalar dx = diff.x() / bin_size_.x();

      if (diff.x() < 0)
        x_index[0] = x_index[1] = 0;
      else if (dx > Base::data_->xSize() - 1)
        x_index[0] = x_index[1] = Base::data_->xSize() - 1;
      else
      {
        x_index[0] = size_t(std::floor(dx));
        x_index[1] = x_index[0] + 1;
        x_weight[1] = dx - x_index[0];
        x_weight[0] = 1.0 - x_weight[1];
      }

      Scalar dy = diff.y() / bin_size_.y();

      if (diff.y() < 0)
        y_index[0] = y_index[1] = 0;
      else if (dy > Base::data_->ySize() - 1)
        y_index[0] = y_index[1] = Base::data_->ySize() - 1;
      else
      {
        y_index[0] = size_t(std::floor(dy));
        y_index[1] = y_index[0] + 1;
        y_weight[1] = dy - y_index[0];
        y_weight[0] = 1.0 - y_weight[1];
      }

      Scalar tmp_z = 0.0;
      for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
          tmp_z += x_weight[i] * y_weight[j] * Poly::evaluate(Base::polynomial(x_index[i], y_index[j]), z);

      z = tmp_z;

    }

    static Point2 project(Scalar x,
                          Scalar y,
                          Scalar z)
    {
      return Point2(x / z, y / z);
    }

  protected:

    Point2 zero_;
    Array2 bin_size_;
    Array2 fov_;

  };

} /* namespace calibration */
#endif /* KINECT_DEPTH_POLYNOMIAL_MATRIX_MODEL_H_ */
