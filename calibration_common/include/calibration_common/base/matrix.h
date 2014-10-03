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

// TODO move away?
typedef size_t Size1;
typedef Eigen::Array<Size1, 2, 1> Size2;
typedef Eigen::Array<Size1, 3, 1> Size3;

/**
 * @addtogroup Base
 * @{
 */

/**
 * @brief A 2D matrix class
 * @param T_ The object type.
 * @param XSize_ The first dimension size. Can be @c Eigen::Dynamic for resizable matrices.
 * @param YSize_ The second dimension size. Can be @c Eigen::Dynamic for resizable matrices.
 * @param AllocatorT_ The @c std::vector allocator.
 */
template <typename T_, int XSize_ = Eigen::Dynamic, int YSize_ = Eigen::Dynamic, typename AllocatorT_ = std::allocator<T_> >
  class Matrix
  {
  public:

    static const int Size = (XSize_ == Eigen::Dynamic || YSize_ == Eigen::Dynamic) ? Eigen::Dynamic : XSize_ * YSize_; ///< The container size


    typedef std::vector<T_, AllocatorT_> Container; ///< The data container

    typedef boost::shared_ptr<Matrix> Ptr;
    typedef boost::shared_ptr<const Matrix> ConstPtr;

    typedef T_ & Element;
    typedef const T_ & ConstElement;

    /**
     * @brief Creates a @c YSize_ @f$\times@f$ @c XSize_ matrix. Elements are created with the default constructor.
     * @warning Fixed-size only.
     */
    Matrix()
      : container_(XSize_ * YSize_),
        size_(XSize_, YSize_)
    {
      EIGEN_STATIC_ASSERT(Size != Eigen::Dynamic, YOU_CALLED_A_FIXED_SIZE_METHOD_ON_A_DYNAMIC_SIZE_MATRIX_OR_VECTOR);
    }

    /**
     * @brief Creates a @c y_size @f$\times@f$ @c x_size matrix. Elements are created with the default constructor.
     * @param size The two-dimensional size
     * @warning Dynamic-size only.
     */
    explicit Matrix(const Size2 & size)
      : container_(size.x() * size.y()),
        size_(size)
    {
      EIGEN_STATIC_ASSERT(Size == Eigen::Dynamic, YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
      assert(XSize_ == Eigen::Dynamic || size.x() == XSize_);
      assert(YSize_ == Eigen::Dynamic || size.y() == YSize_);
    }

    /**
     * @brief Creates a @c YSize_ @f$\times@f$ @c XSize_ matrix. Elements are copies of @c value.
     * @param value The element to insert in the matrix.
     * @warning Fixed-size only.
     */
    explicit Matrix(const T_ & value)
      : container_(XSize_ * YSize_, value),
        size_(XSize_, YSize_)
    {
      EIGEN_STATIC_ASSERT(Size != Eigen::Dynamic, YOU_CALLED_A_FIXED_SIZE_METHOD_ON_A_DYNAMIC_SIZE_MATRIX_OR_VECTOR);
    }

    /**
     * @brief Creates a @c size.y() @f$\times@f$ @c size.x() matrix. Elements are copies of @c value.
     * @param size The two-dimensional size.
     * @param value The element to insert in the matrix.
     * @warning Dynamic-size only.
     */
    Matrix(const Size2 & size,
           const T_ & value)
      : container_(size.x() * size.y(), value),
        size_(size)
    {
      EIGEN_STATIC_ASSERT(Size == Eigen::Dynamic, YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
      assert(XSize_ == Eigen::Dynamic || size.x() == XSize_);
      assert(YSize_ == Eigen::Dynamic || size.y() == YSize_);
    }

    /**
     * @brief Creates a @c YSize_ @f$\times@f$ @c XSize_ matrix. Elements are those in @c data.
     * @param data The data to be copied.
     * @warning Fixed-size only.
     */
    explicit Matrix(const Container & data)
      : container_(data),
        size_(XSize_, YSize_)
    {
      EIGEN_STATIC_ASSERT(Size != Eigen::Dynamic, YOU_CALLED_A_FIXED_SIZE_METHOD_ON_A_DYNAMIC_SIZE_MATRIX_OR_VECTOR); // TODO also for Dynamic size?
      assert(data.size() == Size);
    }

    /**
     * @brief Returns the actual size of the container.
     * YSize_ @f$\times@f$ @c XSize_ if the matrix is of a fixed size. Otherwise the returned size is the one passed to the constructor.
     * @return The actual size.
     */
    inline const Size2 & size() const
    {
      return size_;
    }

    /**
     * @brief elements
     * @return
     */
    inline Size1 elements() const
    {
      return size_.prod();
    }

    /**
     * @brief reshape
     * @param size
     */
    inline void reshape(const Size2 & size)
    {
      EIGEN_STATIC_ASSERT(XSize_ == Eigen::Dynamic && YSize_ == Eigen::Dynamic,
                          YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
      assert(size.x() * size.y() == container_.size());
      size_ = size;
    }

    /**
     * @brief setData
     * @param data
     */
    inline void setData(const Container & data)
    {
      assert(size_.x() * size_.y() == data.size());
      container_ = data;
    }

    /**
     * @brief operator []
     * @param index
     * @return
     */
    inline const ConstElement operator [](Size1 index) const
    {
      return container_[index];
    }

    /**
     * @brief operator []
     * @param index
     * @return
     */
    inline Element operator [](Size1 index)
    {
      return container_[index];
    }

    /**
     * @brief operator ()
     * @param index
     * @return
     */
    inline const ConstElement operator ()(const Size2 & index) const
    {
      return operator ()(index.x(), index.y());
    }

    /**
     * @brief operator ()
     * @param index
     * @return
     */
    inline Element operator ()(const Size2 & index)
    {
      return operator ()(index.x(), index.y());
    }

    /**
     * @brief at
     * @param index
     * @return
     */
    inline const ConstElement at(const Size2 & index) const
    {
      assert((index < size_).all() and (index >= Size2(0, 0)).all());
      return at(index.x(), index.y());
    }

    /**
     * @brief at
     * @param index
     * @return
     */
    inline Element at(const Size2 & index)
    {
      assert((index < size_).all() and (index >= Size2(0, 0)).all());
      return at(index.x(), index.y());
    }

    /**
     * @brief operator ()
     * @param x_index
     * @param y_index
     * @return
     */
    inline const ConstElement operator ()(Size1 x_index,
                                          Size1 y_index) const
    {
      return container_[y_index * size_.x() + x_index];
    }

    /**
     * @brief operator ()
     * @param x_index
     * @param y_index
     * @return
     */
    inline Element operator ()(Size1 x_index,
                               Size1 y_index)
    {
      return container_[y_index * size_.x() + x_index];
    }

    /**
     * @brief at
     * @param x_index
     * @param y_index
     * @return
     */
    inline const ConstElement at(Size1 x_index,
                                 Size1 y_index) const
    {
      assert(y_index >= 0 and y_index < size_.y());
      assert(x_index >= 0 and x_index < size_.x());
      return operator ()(x_index, y_index);
    }

    /**
     * @brief at
     * @param x_index
     * @param y_index
     * @return
     */
    inline Element at(Size1 x_index,
                      Size1 y_index)
    {
      assert(y_index >= 0 and y_index < size_.y());
      assert(x_index >= 0 and x_index < size_.x());
      return operator ()(x_index, y_index);
    }

    /**
     * @brief matrix
     * @return
     */
    inline Container & container()
    {
      return container_;
    }

    /**
     * @brief matrix
     * @return
     */
    inline const Container & container() const
    {
      return container_;
    }

  private:

    Container container_;
    Size2 size_;

  };

template <typename T_, int XSize_, int YSize_, typename AllocatorT_>
  const int Matrix<T_, XSize_, YSize_, AllocatorT_>::Size;

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_BASE_MATRIX_H_ */
