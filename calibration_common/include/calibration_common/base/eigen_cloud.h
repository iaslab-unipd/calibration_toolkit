/*
 *  Copyright (c) 2015-, Filippo Basso <bassofil@gmail.com>
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

#ifndef UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_EIGEN_CLOUD_H_
#define UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_EIGEN_CLOUD_H_

#include <Eigen/Dense>
#include <calibration_common/base/geometry.h>

namespace unipd
{
namespace calib
{

template <typename ScalarT_, int Dimension_, int XSize_ = Eigen::Dynamic, int YSize_ = Eigen::Dynamic, bool UseArray_ = true>
  class EigenCloud;

template <typename EigenMatrixT_, bool UseArray_>
  struct ContainerTraits {};

template <typename EigenCloudT_>
  struct ContainerTraits<EigenCloudT_, true>
  {
    using Container = Eigen::Array<typename EigenCloudT_::Scalar, EigenCloudT_::Dimension, EigenCloudT_::Size>;
  };

template <typename EigenCloudT_>
  struct ContainerTraits<EigenCloudT_, false>
  {
    using Container = Eigen::Matrix<typename EigenCloudT_::Scalar, EigenCloudT_::Dimension, EigenCloudT_::Size>;
  };

template <typename ScalarT_, int Dimension_, int XSize_, int YSize_, bool UseArray_>
  struct Dimension<EigenCloud<ScalarT_, Dimension_, XSize_, YSize_, UseArray_>>
  {
    static const int value = Dimension_;
  };

template <typename ScalarT_, int Dimension_, int XSize_, int YSize_, bool UseArray_>
  class EigenCloud
  {
  public:

    using Scalar = ScalarT_;

    enum : int
    {
      XSize = XSize_,
      YSize = YSize_,
      Size = ((XSize == Eigen::Dynamic || YSize == Eigen::Dynamic) ? Eigen::Dynamic : XSize * YSize),
      Dimension = Dimension_
    };

    using Container = typename ContainerTraits<EigenCloud, UseArray_>::Container;

    using Element = typename Container::ColXpr;
    using ConstElement = const typename Container::ConstColXpr;

    EigenCloud ()
      : size_(Size2{XSize_ == Eigen::Dynamic ? 0 : XSize_, YSize_ == Eigen::Dynamic ? 0 : YSize_}),
        dimension_(Dimension_)
    {
      //EIGEN_STATIC_ASSERT_FIXED_SIZE(Container);
    }

    EigenCloud (const EigenCloud & other) = default;

    EigenCloud (EigenCloud && other)
      : size_(std::move(other.size_)),
        dimension_(other.dimension_),
        container_(std::move(other.container_))
    {
      other.dimension_ = 0;
    }

    EigenCloud &
    operator = (const EigenCloud & other) = default;

    EigenCloud &
    operator = (EigenCloud && other)
    {
      size_ = std::move(other.size_);
      dimension_ = other.dimension_;
      container_ = std::move(other.container_);
      other.dimension_ = 0;
      return *this;
    }

    explicit
    EigenCloud (Size1 dimension)
      : size_(Size2{XSize_, YSize_}),
        dimension_(dimension)
    {
      EIGEN_STATIC_ASSERT(Size != Eigen::Dynamic, THIS_METHOD_IS_ONLY_FOR_FIXED_SIZE);
      EIGEN_STATIC_ASSERT(Dimension == Eigen::Dynamic, THIS_METHOD_IS_ONLY_FOR_DYNAMIC_SIZE);
    }

    explicit
    EigenCloud (const Size2 & size)
      : size_(size),
        dimension_(Dimension_),
        container_(Dimension_, size.x * size.y)
    {
      EIGEN_STATIC_ASSERT(Size == Eigen::Dynamic, THIS_METHOD_IS_ONLY_FOR_FIXED_SIZE);
      EIGEN_STATIC_ASSERT(Dimension != Eigen::Dynamic, THIS_METHOD_IS_ONLY_FOR_FIXED_SIZE);
      assert(XSize == Eigen::Dynamic || size.x == XSize);
      assert(YSize == Eigen::Dynamic || size.y == YSize);
    }

    EigenCloud (Size1 dimension,
                const Size2 & size)
      : size_(size),
        dimension_(dimension),
        container_(dimension, size.x * size.y)
    {
      EIGEN_STATIC_ASSERT(Size == Eigen::Dynamic, THIS_METHOD_IS_ONLY_FOR_FIXED_SIZE);
      EIGEN_STATIC_ASSERT(Dimension == Eigen::Dynamic, THIS_METHOD_IS_ONLY_FOR_FIXED_SIZE);
      assert(XSize == Eigen::Dynamic || size.x == XSize);
      assert(YSize == Eigen::Dynamic || size.y == YSize);
    }

    template <typename Derived>
      explicit
      EigenCloud (const Eigen::MatrixBase<Derived> & value)
        : size_(Size2{XSize_, YSize_}),
          dimension_(Dimension_),
          container_(Container::Zero())
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Container);
        container_.colwise() += value;
      }

    template <typename Derived>
      EigenCloud (const Size2 & size,
                  const Eigen::MatrixBase<Derived> & value)
        : size_(size),
          dimension_(Dimension_),
          container_(Container::Zero(Dimension_, size.x * size.y))
      {
        EIGEN_STATIC_ASSERT(Size == Eigen::Dynamic, THIS_METHOD_IS_ONLY_FOR_FIXED_SIZE);
        assert(XSize == Eigen::Dynamic || size.x == XSize);
        assert(YSize == Eigen::Dynamic || size.y == YSize);
        container_.colwise() += value;
      }

    explicit
    EigenCloud (const Container & data)
      : size_(Size2{static_cast<Size1>(data.cols()), 1}),
        dimension_(data.rows()),
        container_(data)
    {
      // Do nothing
    }

//    explicit
//    EigenCloud (Container && data)
//      : size_(Size2{static_cast<Size1>(data.cols()), 1}),
//        dimension_(data.rows()),
//        container_(data)
//    {
//      // Do nothing
//    }

    EigenCloud (const Container & data,
                const Size2 & size)
      : size_(size),
        dimension_(data.rows()),
        container_(data)
    {
      assert(reduce(size) == data.cols());
    }

//    EigenCloud (Container && data,
//                const Size2 & size)
//      : size_(size),
//        dimension_(data.rows()),
//        container_(data)
//    {
//      assert(reduce(size) == data.cols());
//    }

//    inline bool
//    isOrganized () const
//    {
//      return size_.y() > 1;
//    }

    template <typename OtherScalarT_>
      EigenCloud<OtherScalarT_, Dimension_, XSize_, YSize_, UseArray_>
      cast () const
      {
        return EigenCloud<OtherScalarT_, Dimension_, XSize_, YSize_, UseArray_>(container_.cast<OtherScalarT_>(), size_);
      }

    /**
     * @brief size
     * @return
     */
    inline const Size2 &
    size () const
    {
      return size_;
    }

    /**
     * @brief resize
     * @param size
     */
    inline void
    resize (const Size2 & size)
    {
      assert(XSize == Eigen::Dynamic || size.x == XSize);
      assert(YSize == Eigen::Dynamic || size.y == YSize);
      container_.conservativeResize(Dimension, size.x * size.y);
      size_ = size;
    }

//    inline void
//    reshape (const Size2 & size)
//    {
//      EIGEN_STATIC_ASSERT(XSize == Eigen::Dynamic and YSize == Eigen::Dynamic,
//                          YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
//      assert(size.x() * size.y() == container_.size());
//      size_ = size;
//    }

    inline Size1
    elements () const
    {
      return reduce(size_);
    }

    inline const ConstElement
    operator [] (Size1 index) const
    {
      return container_.col(index);
    }

    inline Element
    operator [] (Size1 index)
    {
      return container_.col(index);
    }

    inline const ConstElement
    operator () (const Size2 & index) const
    {
      return operator ()(index.x, index.y);
    }

    inline Element
    operator () (const Size2 & index)
    {
      return operator ()(index.x, index.y);
    }

    inline const ConstElement
    at (const Size2 & index) const
    {
      return at(index.x, index.y);
    }

    inline Element
    at (const Size2 & index)
    {
      return at(index.x, index.y);
    }

    inline const ConstElement
    operator () (Size1 x_index, Size1 y_index) const
    {
      assert(x_index >= 0 and x_index < size_.x);
      assert(y_index >= 0 and y_index < size_.y);
      return container_.col(y_index * size_.x + x_index);
    }

    inline Element
    operator () (Size1 x_index, Size1 y_index)
    {
      assert(x_index >= 0 and x_index < size_.x);
      assert(y_index >= 0 and y_index < size_.y);
      return container_.col(y_index * size_.x + x_index);
    }

    inline const ConstElement
    at (Size1 x_index, Size1 y_index) const
    {
      assert(x_index >= 0 and x_index < size_.x);
      assert(y_index >= 0 and y_index < size_.y);
      return operator ()(x_index, y_index);
    }

    inline Element
    at (Size1 x_index, Size1 y_index)
    {
      assert(x_index >= 0 and x_index < size_.x);
      assert(y_index >= 0 and y_index < size_.y);
      return operator ()(x_index, y_index);
    }

    inline Container &
    container ()
    {
      return container_;
    }

    inline const Container &
    container () const
    {
      return container_;
    }

    inline void
    transform (const Transform3_<ScalarT_> & transform)
    {
      assert(dimension_ == 3);
      container_ = transform * container_;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:

    Size2 size_;
    Size1 dimension_;
    Container container_;

  };

template <typename ScalarT_>
  using Cloud2_ = EigenCloud<ScalarT_, 2, Eigen::Dynamic, Eigen::Dynamic, false>;

template <typename ScalarT_>
  using Cloud3_ = EigenCloud<ScalarT_, 3, Eigen::Dynamic, Eigen::Dynamic, false>;

using Cloud2 = Cloud2_<Scalar>;
using Cloud3 = Cloud3_<Scalar>;

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_EIGEN_CLOUD_H_
