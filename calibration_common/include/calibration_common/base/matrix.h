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

#ifndef CALIBRATION_COMMON_BASE_MATRIX_H_
#define CALIBRATION_COMMON_BASE_MATRIX_H_

#include <vector>
#include <Eigen/Core>
#include <boost/smart_ptr/shared_ptr.hpp>

namespace calibration
{

template <typename T, int XSize = Eigen::Dynamic, int YSize = Eigen::Dynamic, typename AllocatorT = std::allocator<T> >
  class Matrix
  {
  public:

    static const int Size = (XSize == Eigen::Dynamic || YSize == Eigen::Dynamic) ? Eigen::Dynamic : XSize * YSize;

    typedef std::vector<T, AllocatorT> DataMatrix;

    typedef boost::shared_ptr<Matrix> Ptr;
    typedef boost::shared_ptr<const Matrix> ConstPtr;

    typedef T & Element;
    typedef const T & ConstElement;

    Matrix()
      : matrix_(XSize * YSize),
        x_size_(XSize),
        y_size_(YSize)
    {
      EIGEN_STATIC_ASSERT(Size != Eigen::Dynamic, YOU_CALLED_A_FIXED_SIZE_METHOD_ON_A_DYNAMIC_SIZE_MATRIX_OR_VECTOR);
    }

    Matrix(size_t x_size,
           size_t y_size)
      : matrix_(x_size * y_size),
        x_size_(x_size),
        y_size_(y_size)
    {
      EIGEN_STATIC_ASSERT(Size == Eigen::Dynamic, YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
      assert(XSize == Eigen::Dynamic || x_size == XSize);
      assert(YSize == Eigen::Dynamic || y_size == YSize);
    }

    Matrix(const T & value)
      : matrix_(XSize * YSize, value),
        x_size_(XSize),
        y_size_(YSize)
    {
      EIGEN_STATIC_ASSERT(Size != Eigen::Dynamic, YOU_CALLED_A_FIXED_SIZE_METHOD_ON_A_DYNAMIC_SIZE_MATRIX_OR_VECTOR);
    }

    Matrix(size_t x_size,
           size_t y_size,
           const T & value)
      : matrix_(x_size * y_size, value),
        x_size_(x_size),
        y_size_(y_size)
    {
      EIGEN_STATIC_ASSERT(Size == Eigen::Dynamic, YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
      assert(XSize == Eigen::Dynamic || x_size == XSize);
      assert(YSize == Eigen::Dynamic || y_size == YSize);
    }

    explicit Matrix(const DataMatrix & data)
      : matrix_(data),
        x_size_(XSize),
        y_size_(YSize)
    {
      EIGEN_STATIC_ASSERT(Size != Eigen::Dynamic, YOU_CALLED_A_FIXED_SIZE_METHOD_ON_A_DYNAMIC_SIZE_MATRIX_OR_VECTOR); // TODO also for Dynamic size?
      assert(data.size() == Size);
    }

    size_t size() const
    {
      return matrix_.size();
    }

    size_t xSize() const
    {
      return x_size_;
    }

    size_t ySize() const
    {
      return y_size_;
    }

    void reshape(size_t x_size,
                 size_t y_size)
    {
      EIGEN_STATIC_ASSERT(XSize == Eigen::Dynamic && YSize == Eigen::Dynamic,
                          YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
      assert(x_size * y_size == matrix_.size());
      x_size_ = x_size;
      y_size_ = y_size;
    }

    void setData(const DataMatrix & data)
    {
      assert(xSize() * ySize() == data.size());
      matrix_ = data;
    }

    const ConstElement operator [](size_t index) const
    {
      return matrix_[index];
    }

    Element operator [](size_t index)
    {
      return matrix_[index];
    }

    const ConstElement operator ()(size_t x_index,
                                   size_t y_index) const
    {
      return matrix_[y_index * x_size_ + x_index];
    }

    Element operator ()(size_t x_index,
                        size_t y_index)
    {
      return matrix_[y_index * x_size_ + x_index];
    }

    const ConstElement at(size_t x_index,
                          size_t y_index) const
    {
      assert(y_index >= 0 and y_index < y_size_);
      assert(x_index >= 0 and x_index < x_size_);
      return operator ()(x_index, y_index);
    }

    Element at(size_t x_index,
               size_t y_index)
    {
      assert(y_index >= 0 and y_index < y_size_);
      assert(x_index >= 0 and x_index < x_size_);
      return operator ()(x_index, y_index);
    }

    DataMatrix & matrix()
    {
      return matrix_;
    }

    const DataMatrix & matrix() const
    {
      return matrix_;
    }

  private:

    DataMatrix matrix_;

    size_t x_size_;
    size_t y_size_;

  };

#ifndef MAX
  #define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

template <typename EigenT, int XSize = Eigen::Dynamic, int YSize = Eigen::Dynamic>
  class EigenMatrix
  {
  public:

    typedef boost::shared_ptr<EigenMatrix> Ptr;
    typedef boost::shared_ptr<const EigenMatrix> ConstPtr;

    typedef typename EigenT::Scalar T;

    static const int TSize = MAX(EigenT::RowsAtCompileTime, EigenT::ColsAtCompileTime);
    static const int Size = (XSize == Eigen::Dynamic || YSize == Eigen::Dynamic) ? Eigen::Dynamic : XSize * YSize;

    typedef Eigen::Array<T, TSize, Size> DataMatrix;

    typedef typename DataMatrix::ColXpr Element;
    typedef const typename DataMatrix::ConstColXpr ConstElement;

    EigenMatrix()
      : x_size_(XSize),
        y_size_(YSize)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(DataMatrix);
    }

    EigenMatrix(size_t x_size,
                size_t y_size)
      : matrix_(TSize, x_size * y_size),
        x_size_(x_size),
        y_size_(y_size)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT);
      EIGEN_STATIC_ASSERT_DYNAMIC_SIZE(DataMatrix);
      assert(XSize == Eigen::Dynamic || x_size == XSize);
      assert(YSize == Eigen::Dynamic || y_size == YSize);
    }

    EigenMatrix(const EigenT & value)
      : matrix_(DataMatrix::Zero()),
        x_size_(XSize),
        y_size_(YSize)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(DataMatrix);
      matrix_.colwise() += value;
    }

    EigenMatrix(size_t x_size,
                size_t y_size,
                const EigenT & value)
      : matrix_(DataMatrix::Zero(TSize, x_size * y_size)),
        x_size_(x_size),
        y_size_(y_size)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT);
      EIGEN_STATIC_ASSERT_DYNAMIC_SIZE(DataMatrix);
      assert(XSize == Eigen::Dynamic || x_size == XSize);
      assert(YSize == Eigen::Dynamic || y_size == YSize);
      matrix_.colwise() += value;
    }

    explicit EigenMatrix(const DataMatrix & data)
      : matrix_(data),
        x_size_(XSize),
        y_size_(YSize)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(DataMatrix); // TODO also for Dynamic size?
      assert(data.size() == Size);
    }

    size_t size() const
    {
      return matrix_.size();
    }

    size_t xSize() const
    {
      return x_size_;
    }

    size_t ySize() const
    {
      return y_size_;
    }

    void reshape(size_t x_size,
                 size_t y_size)
    {
      EIGEN_STATIC_ASSERT(XSize == Eigen::Dynamic and YSize == Eigen::Dynamic,
                          YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
      assert(x_size * y_size == matrix_.size());
      x_size_ = x_size;
      y_size_ = y_size;
    }

    void setData(const DataMatrix & data)
    {
      assert(xSize() * ySize() == data.size());
      matrix_ = data;
    }

    const ConstElement operator [](size_t index) const
    {
      return matrix_.col(index);
    }

    Element operator [](size_t index)
    {
      return matrix_.col(index);
    }

    const ConstElement operator ()(size_t x_index,
                                   size_t y_index) const
    {
      return matrix_.col(y_index * x_size_ + x_index);
    }

    Element operator ()(size_t x_index,
                        size_t y_index)
    {
      return matrix_.col(y_index * x_size_ + x_index);
    }

    const ConstElement at(size_t x_index,
                          size_t y_index) const
    {
      assert(y_index >= 0 and y_index < y_size_);
      assert(x_index >= 0 and x_index < x_size_);
      return operator ()(x_index, y_index);
    }

    Element at(size_t x_index,
               size_t y_index)
    {
      assert(y_index >= 0 and y_index < y_size_);
      assert(x_index >= 0 and x_index < x_size_);
      return operator ()(x_index, y_index);
    }

    DataMatrix & matrix()
    {
      return matrix_;
    }

    const DataMatrix & matrix() const
    {
      return matrix_;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    DataMatrix matrix_;

    size_t x_size_;
    size_t y_size_;

  };

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_BASE_MATRIX_H_ */
