/*
 *  Copyright (C) 2013 - Filippo Basso <bassofil@dei.unipd.it>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
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

    explicit PointMatrix(size_t x_size,
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
