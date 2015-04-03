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

#include <calibration_common/base/matrix.h>

namespace calibration
{

/**
 * @brief The EigenMatrix class
 * @param EigenT_
 * @param XSize_
 * @param YSize_
 * @param UseArray_
 */
template <typename EigenT_, int XSize_ = Eigen::Dynamic, int YSize_ = Eigen::Dynamic, bool UseArray_ = true>
  class EigenMatrix;

/**
 * @brief The ContainerTraits struct
 */
template <typename EigenMatrixT_, bool UseArray_>
  struct ContainerTraits
  {

  };

template <typename EigenMatrixT_>
  struct ContainerTraits<EigenMatrixT_, true>
  {
    typedef Eigen::Array<typename EigenMatrixT_::Scalar, EigenMatrixT_::TSize, EigenMatrixT_::Size, (EigenMatrixT_::TSize > 1 ? Eigen::ColMajor : Eigen::RowMajor)> Container;
  };

template <typename EigenMatrixT_>
  struct ContainerTraits<EigenMatrixT_, false>
  {
    typedef Eigen::Matrix<typename EigenMatrixT_::Scalar, EigenMatrixT_::TSize, EigenMatrixT_::Size, (EigenMatrixT_::TSize > 1 ? Eigen::ColMajor : Eigen::RowMajor)> Container;
  };

#define COMPUTE_SIZE(X, Y) ((X == Eigen::Dynamic || Y == Eigen::Dynamic) ? Eigen::Dynamic : X * Y)

template <typename EigenT_, int XSize_, int YSize_, bool UseArray_>
  class EigenMatrix
  {
  public:

    typedef boost::shared_ptr<EigenMatrix> Ptr;
    typedef boost::shared_ptr<const EigenMatrix> ConstPtr;

    typedef typename EigenT_::Scalar Scalar;

    static const int TSize = COMPUTE_SIZE(EigenT_::RowsAtCompileTime, EigenT_::ColsAtCompileTime);
    static const int Size = COMPUTE_SIZE(XSize_, YSize_);

    typedef typename ContainerTraits<EigenMatrix, UseArray_>::Container Container;

    typedef typename Container::ColXpr Element;
    typedef const typename Container::ConstColXpr ConstElement;

    /**
     * @brief EigenMatrix
     */
    EigenMatrix()
      : size_(XSize_ == Eigen::Dynamic ? 0 : XSize_, YSize_ == Eigen::Dynamic ? 0 : YSize_),
        t_size_(TSize)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
    }

//    /**
//     * @brief EigenMatrix
//     * @param t_size
//     */
//    EigenMatrix(Size1 t_size)
//      : container_(t_size, Size), //TODO XXX!
//        size_(XSize_ == Eigen::Dynamic ? 0 : XSize_, YSize_ == Eigen::Dynamic ? 0 : YSize_),
//        t_size_(t_size)
//    {
//      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
//      assert(TSize == Eigen::Dynamic || t_size == TSize);
//    }

    /**
     * @brief EigenMatrix
     * @param size
     */
    explicit EigenMatrix(const Size2 & size)
      : container_(TSize, size.x() * size.y()),
        size_(size)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
      EIGEN_STATIC_ASSERT_DYNAMIC_SIZE(Container);
      assert(XSize_ == Eigen::Dynamic || size.x() == XSize_);
      assert(YSize_ == Eigen::Dynamic || size.y() == YSize_);
    }

//    /**
//     * @brief EigenMatrix
//     * @param t_size
//     * @param size
//     */
//    explicit EigenMatrix(Size1 t_size, const Size2 & size) //TODO XXX!
//      : container_(t_size, size.x() * size.y()),
//        size_(size)
//    {
//      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
//      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
//      EIGEN_STATIC_ASSERT_DYNAMIC_SIZE(Container);
//      assert(XSize_ == Eigen::Dynamic || size.x() == XSize_);
//      assert(YSize_ == Eigen::Dynamic || size.y() == YSize_);
//    }

    /**
     * @brief EigenMatrix
     * @param value
     */
    explicit EigenMatrix(const EigenT_ & value)
      : container_(Container::Zero()),
        size_(XSize_, YSize_)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(Container);
      container_.colwise() += value;
    }

    /**
     * @brief EigenMatrix
     * @param size
     * @param value
     */
    EigenMatrix(const Size2 & size,
                const EigenT_ & value)
      : container_(Container::Zero(TSize, size.x() * size.y())),
        size_(size)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
      EIGEN_STATIC_ASSERT_DYNAMIC_SIZE(Container);
      assert(XSize_ == Eigen::Dynamic || size.x() == XSize_);
      assert(YSize_ == Eigen::Dynamic || size.y() == YSize_);
      container_.colwise() += value;
    }

    /**
     * @brief isOrganized
     * @return
     */
    bool isOrganized() const
    {
      return size_.y() > 1;
    }

    /**
     * @brief size
     * @return
     */
    inline const Size2 & size() const
    {
      return size_;
    }

    /**
     * @brief resize
     * @param size
     */
    inline void resize(const Size2 & size)
    {
      assert(XSize_ == Eigen::Dynamic || size.x() == XSize_);
      assert(YSize_ == Eigen::Dynamic || size.y() == YSize_);
      container_.conservativeResize(TSize,  size.x() * size.y());
      size_ = size;
    }

    /**
     * @brief reshape
     * @param size
     */
    inline void reshape(const Size2 & size)
    {
      EIGEN_STATIC_ASSERT(XSize_ == Eigen::Dynamic and YSize_ == Eigen::Dynamic,
                          YOU_CALLED_A_DYNAMIC_SIZE_METHOD_ON_A_FIXED_SIZE_MATRIX_OR_VECTOR);
      assert(size.x() * size.y() == container_.size());
      size_ = size;
    }

    /**
     * @brief elements
     * @return
     */
    inline Size1 elements() const
    {
      return size_.prod();
    }
    
//    /**
//     * @brief setData
//     * @param data
//     */
//    inline void setData(const Container & data)
//    {
//      assert(size_.x() * size_.y() == data.size());
//      container_ = data;
//    }

    /**
     * @brief operator []
     * @param index
     * @return
     */
    inline const ConstElement operator [](Size1 index) const
    {
      return container_.col(index);
    }

    /**
     * @brief operator []
     * @param index
     * @return
     */
    inline Element operator [](Size1 index)
    {
      return container_.col(index);
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
     * @param x_index
     * @param y_index
     * @return
     */
    inline const ConstElement at(const Size2 & index) const
    {
      return at(index.x(), index.y());
    }

    /**
     * @brief at
     * @param x_index
     * @param y_index
     * @return
     */
    inline Element at(const Size2 & index)
    {
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
      assert(x_index >= 0 and x_index < size_.x());
      assert(y_index >= 0 and y_index < size_.y());
      return container_.col(y_index * size_.x() + x_index);
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
      assert(x_index >= 0 and x_index < size_.x());
      assert(y_index >= 0 and y_index < size_.y());
      return container_.col(y_index * size_.x() + x_index);
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
      assert(x_index >= 0 and x_index < size_.x());
      assert(y_index >= 0 and y_index < size_.y());
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
      assert(x_index >= 0 and x_index < size_.x());
      assert(y_index >= 0 and y_index < size_.y());
      return operator ()(x_index, y_index);
    }

    /**
     * @brief container
     * @return
     */
    inline Container & container()
    {
      return container_;
    }

    /**
     * @brief container
     * @return
     */
    inline const Container & container() const
    {
      return container_;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:

    Container container_;
    Size2 size_;
    Size1 t_size_;

  };

template <typename EigenT_, int XSize_, int YSize_, bool UseArray_>
  const int EigenMatrix<EigenT_, XSize_, YSize_, UseArray_>::TSize;

template <typename EigenT_, int XSize_, int YSize_, bool UseArray_>
  const int EigenMatrix<EigenT_, XSize_, YSize_, UseArray_>::Size;

template <typename ScalarT_, int Dimension_, int XSize_ = Eigen::Dynamic, int YSize_ = Eigen::Dynamic>
  class PointMatrix : public EigenMatrix<Eigen::Matrix<ScalarT_, Dimension_, 1>, XSize_, YSize_, false>
  {
  public:

    typedef boost::shared_ptr<PointMatrix> Ptr;
    typedef boost::shared_ptr<const PointMatrix> ConstPtr;

    typedef Eigen::Matrix<ScalarT_, Dimension_, 1> Point;
    typedef EigenMatrix<Point, XSize_, YSize_, false> Base;

    typedef typename Base::Container Container;
    typedef Eigen::Transform<ScalarT_, 3, Eigen::Affine> Transform;

    /**
     * @brief PointMatrix
     */
    PointMatrix()
      : Base()
    {
      // Do nothing
    }

    /**
     * @brief PointMatrix
     * @param x_size
     * @param y_size
     */
    explicit PointMatrix(const Size2 & size)
      : Base(size)
    {
      // Do nothing
    }

    /**
     * @brief PointMatrix
     * @param value
     */
    template <typename OtherDerived>
      explicit PointMatrix(const Eigen::DenseBase<OtherDerived> & value)
        : Base(Point(value))
      {
        // Do nothing
      }

    /**
     * @brief PointMatrix
     * @param size
     * @param value
     */
    template <typename OtherDerived>
      PointMatrix(const Size2 & size,
                  const Eigen::DenseBase<OtherDerived> & value)
        : Base(size, Point(value))
      {
        // Do nothing
      }

    /**
     * @brief PointMatrix
     * @param other
     * @param indices
     */
    PointMatrix(const PointMatrix & other,
                const std::vector<int> & indices)
      : Base(Size2(1, indices.size()))
    {
      for (Size1 i = 0; i < indices.size(); ++i)
        Base::operator [](i) = other[indices[i]];
    }

//    /**
//     * @brief PointMatrix
//     * @param container
//     */
//    explicit PointMatrix(const Container & container)
//      : Base(container)
//    {
//      // Do nothing
//    }

    /**
     * @brief transform
     * @param transform
     */
    inline void transform(const Eigen::Transform<ScalarT_, 3, Eigen::Affine> & transform)
    {
      EIGEN_STATIC_ASSERT(Dimension_ == 3, THIS_METHOD_IS_ONLY_FOR_OBJECTS_OF_A_SPECIFIC_SIZE);
      Base::container_ = transform * Base::container_;
    }

  };

} /* namespace calibration */

#endif /* CALIBRATION_COMMON_BASE_POINT_MATRIX_H_ */
