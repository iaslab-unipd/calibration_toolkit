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

#ifndef CALIBRATION_COMMON_BASE_MATRIX_H_
#define CALIBRATION_COMMON_BASE_MATRIX_H_

#include <vector>
#include <Eigen/Core>
#include <boost/smart_ptr/shared_ptr.hpp>

namespace calibration
{

template <typename T_, int XSize_ = Eigen::Dynamic, int YSize_ = Eigen::Dynamic, typename AllocatorT_ = std::allocator<T_> >
  class Matrix
  {
  public:

    static const int Size = (XSize_ == Eigen::Dynamic || YSize_ == Eigen::Dynamic) ? Eigen::Dynamic : XSize_ * YSize_;

    typedef std::vector<T_, AllocatorT_> DataMatrix;

    typedef boost::shared_ptr<Matrix> Ptr;
    typedef boost::shared_ptr<const Matrix> ConstPtr;

    typedef T_ & Element;
    typedef const T_ & ConstElement;

    Matrix()
      : matrix_(XSize_ * YSize_),
        x_size_(XSize_),
        y_size_(YSize_)
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
      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
    }

    Matrix(const T_ & value)
      : matrix_(XSize_ * YSize_, value),
        x_size_(XSize_),
        y_size_(YSize_)
    {
      EIGEN_STATIC_ASSERT(Size != Eigen::Dynamic, YOU_CALLED_A_FIXED_SIZE_METHOD_ON_A_DYNAMIC_SIZE_MATRIX_OR_VECTOR);
    }

    Matrix(size_t x_size,
           size_t y_size,
           const T_ & value)
      : matrix_(x_size * y_size, value),
        x_size_(x_size),
        y_size_(y_size)
    {
      EIGEN_STATIC_ASSERT(Size == Eigen::Dynamic, YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
    }

    explicit Matrix(const DataMatrix & data)
      : matrix_(data),
        x_size_(XSize_),
        y_size_(YSize_)
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
      EIGEN_STATIC_ASSERT(XSize_ == Eigen::Dynamic && YSize_ == Eigen::Dynamic,
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

template <typename EigenT_, int XSize_ = Eigen::Dynamic, int YSize_ = Eigen::Dynamic, bool UseArray_ = true>
  class EigenMatrix;

template <typename EigenMatrixT_, bool UseArray_>
  struct ContainerTraits
  {

  };

template <typename EigenMatrixT_>
  struct ContainerTraits<EigenMatrixT_, true>
  {
    typedef Eigen::Array<typename EigenMatrixT_::Scalar, EigenMatrixT_::TSize, EigenMatrixT_::Size, Eigen::ColMajor> Container;
  };

template <typename EigenMatrixT_>
  struct ContainerTraits<EigenMatrixT_, false>
  {
    typedef Eigen::Matrix<typename EigenMatrixT_::Scalar, EigenMatrixT_::TSize, EigenMatrixT_::Size, Eigen::ColMajor> Container;
  };

template <typename EigenT_, int XSize_, int YSize_, bool UseArray_>
  class EigenMatrix
  {
  public:

    typedef boost::shared_ptr<EigenMatrix> Ptr;
    typedef boost::shared_ptr<const EigenMatrix> ConstPtr;

    typedef typename EigenT_::Scalar Scalar;

    static const int TSize = MAX(EigenT_::RowsAtCompileTime, EigenT_::ColsAtCompileTime);
    static const int Size = (XSize_ == Eigen::Dynamic || YSize_ == Eigen::Dynamic) ? Eigen::Dynamic : XSize_ * YSize_;

    typedef typename ContainerTraits<EigenMatrix, UseArray_>::Container Container;

    typedef typename Container::ColXpr Element;
    typedef const typename Container::ConstColXpr ConstElement;

    EigenMatrix()
      : x_size_(XSize_),
        y_size_(YSize_)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(Container);
    }

    EigenMatrix(size_t x_size,
                size_t y_size)
      : matrix_(TSize, x_size * y_size),
        x_size_(x_size),
        y_size_(y_size)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
      EIGEN_STATIC_ASSERT_DYNAMIC_SIZE(Container);
      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
    }

    EigenMatrix(const EigenT_ & value)
      : matrix_(Container::Zero()),
        x_size_(XSize_),
        y_size_(YSize_)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(Container);
      matrix_.colwise() += value;
    }

    EigenMatrix(size_t x_size,
                size_t y_size,
                const EigenT_ & value)
      : matrix_(Container::Zero(TSize, x_size * y_size)),
        x_size_(x_size),
        y_size_(y_size)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
      EIGEN_STATIC_ASSERT_DYNAMIC_SIZE(Container);
      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
      matrix_.colwise() += value;
    }

    explicit EigenMatrix(const Container & data)
      : matrix_(data),
        x_size_(XSize_),
        y_size_(YSize_)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(Container); // TODO also for Dynamic size?
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
      EIGEN_STATIC_ASSERT(XSize_ == Eigen::Dynamic and YSize_ == Eigen::Dynamic,
                          YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
      assert(x_size * y_size == matrix_.size());
      x_size_ = x_size;
      y_size_ = y_size;
    }

    void setData(const Container & data)
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

    Container & matrix()
    {
      return matrix_;
    }

    const Container & matrix() const
    {
      return matrix_;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    Container matrix_;

    size_t x_size_;
    size_t y_size_;

  };

template <typename T_, int XSize_, int YSize_, typename AllocatorT_>
  const int Matrix<T_, XSize_, YSize_, AllocatorT_>::Size;

template <typename EigenT_, int XSize_, int YSize_, bool UseArray_>
  const int EigenMatrix<EigenT_, XSize_, YSize_, UseArray_>::TSize;

template <typename EigenT_, int XSize_, int YSize_, bool UseArray_>
  const int EigenMatrix<EigenT_, XSize_, YSize_, UseArray_>::Size;

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_BASE_MATRIX_H_ */
