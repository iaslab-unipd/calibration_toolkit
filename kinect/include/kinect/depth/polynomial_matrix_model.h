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
  class PolynomialMatrixModel_;

template <typename PolynomialT_>
  struct ModelTraits<PolynomialMatrixModel_<PolynomialT_> >
  {
    typedef PolynomialT_ Poly;
    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;
    typedef EigenMatrix<typename MathTraits<PolynomialT_>::Coefficients> Data;
  };

template <typename PolynomialT_>
  class PolynomialMatrixModel_
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixModel_> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixModel_> ConstPtr;

    typedef ModelTraits<PolynomialMatrixModel_> Traits;
//    typedef DepthUndistortionModel<PolynomialSimpleMatrixModel> Base;

    typedef typename Traits::Poly Poly;
    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::Data Data;

    typedef typename Data::Element Element;
    typedef typename Data::ConstElement ConstElement;

    PolynomialMatrixModel_(const Size2 & image_size)
      : image_size_(image_size),
        bin_size_(1, 1)
    {
      // Do nothing
    }

    inline const Size2 & imageSize() const
    {
      return image_size_;
    }

    inline virtual void setMatrix(const typename Data::Ptr & matrix,
                                  const Size2 & bin_size)
    {
      matrix_ = matrix;
      bin_size_ = bin_size;
    }

    inline virtual void setMatrix(const typename Data::Ptr & matrix)
    {
      matrix_ = matrix;
      bin_size_ = image_size_ / matrix->size();
    }

    inline const typename Data::Ptr & matrix() const
    {
      return matrix_;
    }

    inline const Size2 & binSize() const
    {
      return bin_size_;
    }

    inline Element polynomial(Size1 x_index,
                              Size1 y_index)
    {
      assert(matrix_);
      return (*matrix_)(x_index, y_index);
    }

    inline const ConstElement polynomial(Size1 x_index,
                                         Size1 y_index) const
    {
      assert(matrix_);
      return (*boost::static_pointer_cast<const Data>(matrix_))(x_index, y_index);
    }

    inline Element polynomial(const Size2 & index)
    {
      assert(matrix_);
      return (*matrix_)(index);
    }

    inline const ConstElement polynomial(const Size2 & index) const
    {
      assert(matrix_);
      return (*boost::static_pointer_cast<const Data>(matrix_))(index);
    }

    inline typename Data::ConstPtr data() const
    {
      return typename Data::ConstPtr(matrix_);
    }

    inline virtual Scalar * dataPtr()
    {
      assert(matrix_);
      return matrix_->container().data();
    }

    inline virtual const Scalar * dataPtr() const
    {
      assert(matrix_);
      return matrix_->container().data();
    }

  protected:

    typename Data::Ptr matrix_;
    Size2 image_size_;
    Size2 bin_size_;

  };

template <typename PolynomialT_>
  class PolynomialMatrixSimpleModel;

template <typename PolynomialT_>
  struct ModelTraits<PolynomialMatrixSimpleModel<PolynomialT_> > : public ModelTraits<PolynomialMatrixModel_<PolynomialT_> >
  {
    typedef ModelTraits<PolynomialMatrixModel_<PolynomialT_> > Base;
    typedef typename Base::Poly Poly;
    typedef typename Base::Scalar Scalar;
    typedef typename Base::Data Data;
  };

template <typename PolynomialT_>
  class PolynomialMatrixSimpleModel : public PolynomialMatrixModel_<PolynomialT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixSimpleModel> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixSimpleModel> ConstPtr;

    typedef ModelTraits<PolynomialMatrixSimpleModel> Traits;
    typedef PolynomialMatrixModel_<PolynomialT_> Base;

    typedef typename Traits::Poly Poly;
    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::Data Data;

    typedef typename Data::Element Element;
    typedef typename Data::ConstElement ConstElement;

    PolynomialMatrixSimpleModel(const Size2 & image_size)
      : Base(image_size)
    {
      // Do nothing
    }

    inline typename Data::Ptr createMatrix(const Size2 & bin_size)
    {
      assert((bin_size <= Base::image_size_).all() and (bin_size > Size2(0, 0)).all());
      return boost::make_shared<Data>(Base::image_size_ / bin_size);
    }

    inline Size2 matrixIndex(Size1 x_index,
                             Size1 y_index) const
    {
      assert(x_index < Base::image_size_.x() and x_index >= 0 and y_index < Base::image_size_.y() and y_index >= 0);
      return Size2(x_index / Base::bin_size_.x(), y_index / Base::bin_size_.y());
    }

    inline Size2 matrixIndex(const Size2 & index) const
    {
      assert((index < Base::image_size_).all() and (index >= Size2(0, 0)).all());
      return index / Base::bin_size_;
    }

    inline void undistort(Size1 x_index,
                          Size1 y_index,
                          Scalar & depth) const
    {
      depth = Poly::evaluate(Base::polynomial(matrixIndex(x_index, y_index)), depth);
    }

    inline void undistort(const Size2 & index,
                          Scalar & depth) const
    {
      depth = Poly::evaluate(Base::polynomial(matrixIndex(index)), depth);
    }

  };

template <typename PolynomialT_>
  class PolynomialMatrixSmoothModel;

template <typename PolynomialT_>
  struct ModelTraits<PolynomialMatrixSmoothModel<PolynomialT_> > : public ModelTraits<PolynomialMatrixModel_<PolynomialT_> >
  {
    typedef ModelTraits<PolynomialMatrixModel_<PolynomialT_> > Base;
    typedef typename Base::Poly Poly;
    typedef typename Base::Scalar Scalar;
    typedef typename Base::Data Data;
  };

/**
 * @brief The PolynomialSmoothMatrixModel class
 * @todo Use a Lookup Table
 */
template <typename PolynomialT_>
  class PolynomialMatrixSmoothModel : public PolynomialMatrixModel_<PolynomialT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixSmoothModel> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixSmoothModel> ConstPtr;

    typedef ModelTraits<PolynomialMatrixSmoothModel> Traits;
    typedef PolynomialMatrixModel_<PolynomialT_> Base;

    typedef typename Traits::Poly Poly;
    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::Data Data;

    typedef typename Data::Element Element;
    typedef typename Data::ConstElement ConstElement;

    PolynomialMatrixSmoothModel(const Size2 & image_size)
      : Base(image_size)
    {
      // Do nothing
    }

    inline typename Data::Ptr createMatrix(const Size2 & bin_size)
    {
      assert((bin_size <= Base::image_size_).all() and (bin_size > Size2(0, 0)).all());
      assert(bin_size.x() % 2 == 0 and bin_size.y() % 2 == 0);
      return boost::make_shared<Data>(Base::image_size_ / bin_size + Size2(1, 1));
    }

    inline virtual void setMatrix(const typename Data::Ptr & matrix,
                                  const Size2 & bin_size)
    {
      Base::setMatrix(matrix, bin_size);
      assert(bin_size.x() % 2 == 0 and bin_size.y() % 2 == 0);
    }

    inline virtual void setMatrix(const typename Data::Ptr & matrix)
    {
      Base::setMatrix(matrix, Base::image_size_ / (matrix->size() - Size2(1, 1)));
      assert((Base::bin_size_.x() % 2 == 0 and Base::bin_size_.y() % 2 == 0) or (Base::bin_size_ == Size2(1, 1)).all());
    }

    inline Size2 matrixIndex(Size1 x_index,
                             Size1 y_index) const
    {
      assert(x_index < Base::image_size_.x() and x_index >= 0 and y_index < Base::image_size_.y() and y_index >= 0);
      return Size2((x_index + Base::bin_size_.x() / 2) / Base::bin_size_.x(),
                   (y_index + Base::bin_size_.y() / 2) / Base::bin_size_.y());
    }

    inline Size2 matrixIndex(const Size2 & index) const
    {
      assert((index < Base::image_size_).all() and (index >= Size2(0, 0)).all());
      return (index + Base::bin_size_ / 2) / Base::bin_size_;
    }

    void undistort(size_t x_index,
                   size_t y_index,
                   Scalar & depth) const
    {
      if (not (x_index < Base::image_size_.x() and x_index >= 0 and y_index < Base::image_size_.y() and y_index >= 0))
        std::cout << x_index << " " << y_index << std::endl;


      assert(x_index < Base::image_size_.x() and x_index >= 0 and y_index < Base::image_size_.y() and y_index >= 0);

      Size2 x_bin, y_bin;
      Scalar x_weight[2] = {Scalar(0.5), Scalar(0.5)};
      Scalar y_weight[2] = {Scalar(0.5), Scalar(0.5)};

      x_bin[0] = x_index / Base::bin_size_.x();
      x_bin[1] = (x_index + Base::bin_size_.x() / 2) / Base::bin_size_.x();
      if (x_bin[0] == x_bin[1])
        x_bin[1] = x_bin[0] + 1;

      x_weight[1] = static_cast<Scalar>(x_index - x_bin[0] * Base::bin_size_.x()) / Base::bin_size_.x();
      x_weight[0] = Scalar(1.0) - x_weight[1];

      y_bin[0] = y_index / Base::bin_size_.y();
      y_bin[1] = (y_index + Base::bin_size_.y() / 2) / Base::bin_size_.y();
      if (y_bin[0] == y_bin[1])
        y_bin[1] = y_bin[0] + 1;

      y_weight[1] = static_cast<Scalar>(y_index - y_bin[0] * Base::bin_size_.y()) / Base::bin_size_.y();
      y_weight[0] = Scalar(1.0) - y_weight[1];

      Scalar tmp_depth = 0.0;
      for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
          tmp_depth += x_weight[i] * y_weight[j] * Poly::evaluate(Base::polynomial(x_bin[i], y_bin[j]), depth);

      depth = tmp_depth;
    }

    void undistort(const Size2 & index,
                   Scalar & depth) const
    {
      assert((index < Base::image_size_).all() and (index >= Size2(0, 0)).all());

      Eigen::Array<Size1, 2, 2> bin;
      Eigen::Array<Scalar, 2, 2> weight;

      bin.col(0) = index / Base::bin_size_;
      bin.col(1) = matrixIndex(index);

      if (bin(0, 0) == bin(0, 1))
        bin(0, 1) = bin(0, 0) + 1;

      if (bin(1, 0) == bin(1, 1))
        bin(1, 1) = bin(1, 0) + 1;

      weight.col(1) = ((index - bin.col(0) * Base::bin_size_).template cast<Scalar>() / Base::bin_size_.template cast<Scalar>());
      weight.col(0) = Eigen::Array<Scalar, 2, 1>(Scalar(1.0), Scalar(1.0)) - weight.col(1);

      Scalar tmp_depth = 0.0;
      for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
          tmp_depth += weight(0, i) * weight(1, j) * Poly::evaluate(Base::polynomial(bin(0, i), bin(1, j)), depth);

      depth = tmp_depth;
    }

  };

//template <typename PolynomialT_>
//  class PolynomialMatrixProjectedModel;

//template <typename PolynomialT_>
//  struct ModelTraits<PolynomialMatrixProjectedModel<PolynomialT_> > : public ModelTraits<PolynomialMatrixModel<PolynomialT_> >
//  {
//    typedef ModelTraits<PolynomialMatrixModel<PolynomialT_> > Base;
//    typedef typename Base::Poly Poly;
//    typedef typename Base::Scalar Scalar;
//    typedef typename Base::Data Data;
//  };

//template <typename PolynomialT_>
//  class PolynomialMatrixProjectedModel : public PolynomialMatrixModel<PolynomialT_>
//  {
//  public:

//    typedef boost::shared_ptr<PolynomialMatrixProjectedModel> Ptr;
//    typedef boost::shared_ptr<const PolynomialMatrixProjectedModel> ConstPtr;

//    typedef PolynomialMatrixModel<PolynomialT_> Base;

//    typedef typename Base::Scalar Scalar;
//    typedef typename Base::Data Data;
//    typedef typename Base::Poly Poly;
//    typedef typename Base::Element Element;
//    typedef typename Base::ConstElement ConstElement;

//    typedef typename Types<Scalar>::Point2 Point2;
//    typedef Eigen::Array<Scalar, 2, 1> Array2;

//    PolynomialMatrixProjectedModel()
//      : Base(),
//        bin_size_(0.0, 0.0)
//    {
//      // Do nothing
//    }

//    explicit PolynomialMatrixProjectedModel(const typename Data::Ptr & matrix)
//      : Base(matrix),
//        bin_size_(0.0, 0.0)
//    {
//      // Do nothing
//    }

//    PolynomialMatrixProjectedModel(const Base & other)
//      : Base(other),
//        zero_(other.zero_),
//        bin_size_(other.bin_size_),
//        fov_(other.fov_)
//    {
//      // Do nothing
//    }

//    virtual ~PolynomialMatrixProjectedModel()
//    {
//      // Do nothing
//    }

//    void setFieldOfView(Scalar x,
//                        Scalar y)
//    {
//      assert(Base::matrix_);
//      fov_ = Array2(x, y);
//      zero_ = Point2(-std::tan(x / 2), -std::tan(y / 2));
//      bin_size_ = -2 * zero_.array() / Array2(Base::matrix_->size().x(), Base::matrix_->size().y());
//    }

//    Scalar fieldOfViewX() const
//    {
//      return fov_.x();
//    }

//    Scalar fieldOfViewY() const
//    {
//      return fov_.y();
//    }

//    void getIndex(const Point2 & point_proj,
//                  size_t & x_index,
//                  size_t & y_index) const
//    {
//      assert(Base::matrix_);
//      assert(bin_size_.x() > 0 and bin_size_.y() > 0);
//      Point2 diff = point_proj - zero_;
//      x_index = diff.x() < 0 ? 0 : size_t(std::min(Base::matrix_->size().x() - 1.0, std::floor(diff.x() / bin_size_.x())));
//      y_index = diff.y() < 0 ? 0 : size_t(std::min(Base::matrix_->size().y() - 1.0, std::floor(diff.y() / bin_size_.y())));
//    }

//    using Base::polynomial;

//    Element polynomial(const Point2 & point_proj)
//    {
//      size_t x_index, y_index;
//      getIndex(point_proj, x_index, y_index);
//      return Base::polynomial(x_index, y_index);
//    }

//    const ConstElement polynomial(const Point2 & point_proj) const
//    {
//      size_t x_index, y_index;
//      getIndex(point_proj, x_index, y_index);
//      return Base::polynomial(x_index, y_index);
//    }

//    virtual void undistort(const Point2 & point_proj,
//                           Scalar & z) const
//    {
//      assert(Base::matrix_);
//      assert(bin_size_.x() > 0 and bin_size_.y() > 0);
//      Point2 diff = point_proj - zero_ - 0.5 * bin_size_.matrix();

//      size_t x_index[2];
//      size_t y_index[2];
//      Scalar x_weight[2] = {0.5, 0.5};
//      Scalar y_weight[2] = {0.5, 0.5};

//      Scalar dx = diff.x() / bin_size_.x();

//      if (diff.x() < 0)
//        x_index[0] = x_index[1] = 0;
//      else if (dx > Base::matrix_->size().x() - 1)
//        x_index[0] = x_index[1] = Base::matrix_->size().x() - 1;
//      else
//      {
//        x_index[0] = size_t(std::floor(dx));
//        x_index[1] = x_index[0] + 1;
//        x_weight[1] = dx - x_index[0];
//        x_weight[0] = 1.0 - x_weight[1];
//      }

//      Scalar dy = diff.y() / bin_size_.y();

//      if (diff.y() < 0)
//        y_index[0] = y_index[1] = 0;
//      else if (dy > Base::matrix_->size().y() - 1)
//        y_index[0] = y_index[1] = Base::matrix_->size().y() - 1;
//      else
//      {
//        y_index[0] = size_t(std::floor(dy));
//        y_index[1] = y_index[0] + 1;
//        y_weight[1] = dy - y_index[0];
//        y_weight[0] = 1.0 - y_weight[1];
//      }

//      Scalar tmp_z = 0.0;
//      for (int i = 0; i < 2; ++i)
//        for (int j = 0; j < 2; ++j)
//          tmp_z += x_weight[i] * y_weight[j] * Poly::evaluate(Base::polynomial(x_index[i], y_index[j]), z);

//      z = tmp_z;

//    }

//    static Point2 project(Scalar x,
//                          Scalar y,
//                          Scalar z)
//    {
//      return Point2(x / z, y / z);
//    }

//  protected:

//    Point2 zero_;
//    Array2 bin_size_;
//    Array2 fov_;

//  };

} /* namespace calibration */
#endif /* KINECT_DEPTH_POLYNOMIAL_MATRIX_MODEL_H_ */
