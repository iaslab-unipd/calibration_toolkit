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

#ifndef CALIBRATION_COMMON_BASE_POINT_MATRIX_H_
#define CALIBRATION_COMMON_BASE_POINT_MATRIX_H_

#include <boost/smart_ptr/shared_ptr.hpp>
#include <Eigen/Geometry>

namespace calibration
{

template <typename ScalarT_, int Dimension_, int XSize_ = Eigen::Dynamic, int YSize_ = Eigen::Dynamic>
  class PointMatrix
  {
  public:

    static const int Size = (XSize_ == Eigen::Dynamic || YSize_ == Eigen::Dynamic) ? Eigen::Dynamic : XSize_ * YSize_;
    static const int Options = (Dimension_ > 1) ? Eigen::ColMajor : Eigen::RowMajor;

    typedef boost::shared_ptr<PointMatrix> Ptr;
    typedef boost::shared_ptr<const PointMatrix> ConstPtr;

    typedef Eigen::Matrix<ScalarT_, Dimension_, Size, Options, Dimension_> RawMatrix;
    typedef Eigen::Transform<ScalarT_, 3, Eigen::Affine> Transform;

    typedef typename RawMatrix::ColXpr Element;
    typedef typename RawMatrix::ConstColXpr ConstElement;

    PointMatrix()
      : x_size_(XSize_ == Eigen::Dynamic ? 0 : XSize_),
        y_size_(YSize_ == Eigen::Dynamic ? 0 : YSize_)
    {
      //EIGEN_STATIC_ASSERT_FIXED_SIZE(Data);
    }

    PointMatrix(size_t x_size,
                size_t y_size)
      : points_(Dimension_, x_size * y_size),
        x_size_(x_size),
        y_size_(y_size)
    {
      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
    }

    template <typename OtherDerived>
      explicit PointMatrix(const Eigen::DenseBase<OtherDerived> & value)
        : x_size_(XSize_),
          y_size_(YSize_)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(RawMatrix);
        points_.colwise() = value;
      }

    template <typename OtherDerived>
      PointMatrix(size_t x_size,
                  size_t y_size,
                  const Eigen::DenseBase<OtherDerived> & value)
        : points_(Dimension_, x_size * y_size),
          x_size_(x_size),
          y_size_(y_size)
      {
        assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
        assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
        points_.colwise() = value;
      }

    explicit PointMatrix(const RawMatrix & points)
      : points_(points),
        x_size_(XSize_),
        y_size_(YSize_)
    {
      EIGEN_STATIC_ASSERT_FIXED_SIZE(RawMatrix);
    }

    PointMatrix(size_t x_size,
                size_t y_size,
                const RawMatrix & points)
      : points_(points),
        x_size_(x_size),
        y_size_(y_size)
    {
      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
      assert(x_size * y_size == points.cols());
    }

    size_t size() const
    {
      return size_t(points_.cols());
    }

    size_t xSize() const
    {
      return x_size_;
    }

    size_t ySize() const
    {
      return y_size_;
    }

    void setData(const RawMatrix & points)
    {
      assert(size() == points.cols());
      points_ = points;
    }

    void resize(size_t x_size,
                size_t y_size)
    {
      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
      points_.conservativeResize(Dimension_, x_size * y_size);
      x_size_ = x_size;
      y_size_ = y_size;
    }

    void reshape(size_t x_size,
                 size_t y_size)
    {
      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
      assert(x_size * y_size == size());
      x_size_ = x_size;
      y_size_ = y_size;
    }

    Element operator [](size_t index)
    {
      return points_.col(index);
    }

    const ConstElement operator [](size_t index) const
    {
      return points_.col(index);
    }

    const ConstElement at(size_t x_index,
                          size_t y_index) const
    {
      assert(y_index >= 0 && y_index < y_size_);
      assert(x_index >= 0 && x_index < x_size_);
      return operator ()(x_index, y_index);
    }

    Element at(size_t x_index,
               size_t y_index)
    {
      assert(y_index >= 0 && y_index < y_size_);
      assert(x_index >= 0 && x_index < x_size_);
      return operator ()(x_index, y_index);
    }

    const ConstElement operator ()(size_t x_index,
                                   size_t y_index) const
    {
      return operator [](y_index * x_size_ + x_index);
    }

    Element operator ()(size_t x_index,
                        size_t y_index)
    {
      return operator [](y_index * x_size_ + x_index);
    }

    void transform(const Eigen::Transform<ScalarT_, 3, Eigen::Affine> & transform)
    {
      EIGEN_STATIC_ASSERT(Dimension_ == 3, THIS_METHOD_IS_ONLY_FOR_OBJECTS_OF_A_SPECIFIC_SIZE);
      points_ = transform * points_;
    }

    RawMatrix & matrix()
    {
      return points_;
    }

    const RawMatrix & matrix() const
    {
      return points_;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    RawMatrix points_;

    size_t x_size_;
    size_t y_size_;

  };

template <typename ScalarT_, int Dimension_, int XSize_, int YSize_>
  const int PointMatrix<ScalarT_, Dimension_, XSize_, YSize_>::Size;

template <typename ScalarT_, int Dimension_, int XSize_, int YSize_>
  const int PointMatrix<ScalarT_, Dimension_, XSize_, YSize_>::Options;

} /* namespace calibration */

#endif /* CALIBRATION_COMMON_BASE_POINT_MATRIX_H_ */
