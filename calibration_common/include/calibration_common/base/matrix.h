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

/**
 * @brief The Matrix class
 * @param T_
 * @param XSize_
 * @param YSize_
 * @param AllocatorT_
 */
template <typename T_, int XSize_ = Eigen::Dynamic, int YSize_ = Eigen::Dynamic, typename AllocatorT_ = std::allocator<T_> >
  class Matrix
  {
  public:

    static const int Size = (XSize_ == Eigen::Dynamic || YSize_ == Eigen::Dynamic) ? Eigen::Dynamic : XSize_ * YSize_;

    typedef std::vector<T_, AllocatorT_> Container;

    typedef boost::shared_ptr<Matrix> Ptr;
    typedef boost::shared_ptr<const Matrix> ConstPtr;

    typedef T_ & Element;
    typedef const T_ & ConstElement;

    /**
     * @brief Matrix
     */
    Matrix()
      : container_(XSize_ * YSize_),
        x_size_(XSize_),
        y_size_(YSize_)
    {
      EIGEN_STATIC_ASSERT(Size != Eigen::Dynamic, YOU_CALLED_A_FIXED_SIZE_METHOD_ON_A_DYNAMIC_SIZE_MATRIX_OR_VECTOR);
    }

    /**
     * @brief Matrix
     * @param x_size
     * @param y_size
     */
    Matrix(size_t x_size,
           size_t y_size)
      : container_(x_size * y_size),
        x_size_(x_size),
        y_size_(y_size)
    {
      EIGEN_STATIC_ASSERT(Size == Eigen::Dynamic, YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
    }

    /**
     * @brief Matrix
     * @param value
     */
    Matrix(const T_ & value)
      : container_(XSize_ * YSize_, value),
        x_size_(XSize_),
        y_size_(YSize_)
    {
      EIGEN_STATIC_ASSERT(Size != Eigen::Dynamic, YOU_CALLED_A_FIXED_SIZE_METHOD_ON_A_DYNAMIC_SIZE_MATRIX_OR_VECTOR);
    }

    /**
     * @brief Matrix
     * @param x_size
     * @param y_size
     * @param value
     */
    Matrix(size_t x_size,
           size_t y_size,
           const T_ & value)
      : container_(x_size * y_size, value),
        x_size_(x_size),
        y_size_(y_size)
    {
      EIGEN_STATIC_ASSERT(Size == Eigen::Dynamic, YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
    }

    /**
     * @brief Matrix
     * @param data
     */
    explicit Matrix(const Container & data)
      : container_(data),
        x_size_(XSize_),
        y_size_(YSize_)
    {
      EIGEN_STATIC_ASSERT(Size != Eigen::Dynamic, YOU_CALLED_A_FIXED_SIZE_METHOD_ON_A_DYNAMIC_SIZE_MATRIX_OR_VECTOR); // TODO also for Dynamic size?
      assert(data.size() == Size);
    }

    /**
     * @brief size
     * @return
     */
    inline size_t size() const
    {
      return container_.size();
    }

    /**
     * @brief xSize
     * @return
     */
    inline size_t xSize() const
    {
      return x_size_;
    }

    /**
     * @brief ySize
     * @return
     */
    inline size_t ySize() const
    {
      return y_size_;
    }

    /**
     * @brief reshape
     * @param x_size
     * @param y_size
     */
    inline void reshape(size_t x_size,
                        size_t y_size)
    {
      EIGEN_STATIC_ASSERT(XSize_ == Eigen::Dynamic && YSize_ == Eigen::Dynamic,
                          YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
      assert(x_size * y_size == container_.size());
      x_size_ = x_size;
      y_size_ = y_size;
    }

    /**
     * @brief setData
     * @param data
     */
    inline void setData(const Container & data)
    {
      assert(xSize() * ySize() == data.size());
      container_ = data;
    }

    /**
     * @brief operator []
     * @param index
     * @return
     */
    inline const ConstElement operator [](size_t index) const
    {
      return container_[index];
    }

    /**
     * @brief operator []
     * @param index
     * @return
     */
    inline Element operator [](size_t index)
    {
      return container_[index];
    }

    /**
     * @brief operator ()
     * @param x_index
     * @param y_index
     * @return
     */
    inline const ConstElement operator ()(size_t x_index,
                                   size_t y_index) const
    {
      return container_[y_index * x_size_ + x_index];
    }

    /**
     * @brief operator ()
     * @param x_index
     * @param y_index
     * @return
     */
    inline Element operator ()(size_t x_index,
                        size_t y_index)
    {
      return container_[y_index * x_size_ + x_index];
    }

    /**
     * @brief at
     * @param x_index
     * @param y_index
     * @return
     */
    inline const ConstElement at(size_t x_index,
                                 size_t y_index) const
    {
      assert(y_index >= 0 and y_index < y_size_);
      assert(x_index >= 0 and x_index < x_size_);
      return operator ()(x_index, y_index);
    }

    /**
     * @brief at
     * @param x_index
     * @param y_index
     * @return
     */
    inline Element at(size_t x_index,
                      size_t y_index)
    {
      assert(y_index >= 0 and y_index < y_size_);
      assert(x_index >= 0 and x_index < x_size_);
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

    size_t x_size_;
    size_t y_size_;

  };

template <typename T_, int XSize_, int YSize_, typename AllocatorT_>
  const int Matrix<T_, XSize_, YSize_, AllocatorT_>::Size;

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_BASE_MATRIX_H_ */
