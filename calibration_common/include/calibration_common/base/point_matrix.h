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


#ifndef MAX
#define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

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

    /**
     * @brief EigenMatrix
     */
    EigenMatrix()
      : x_size_(XSize_ == Eigen::Dynamic ? 0 : XSize_),
        y_size_(YSize_ == Eigen::Dynamic ? 0 : YSize_)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
//      EIGEN_STATIC_ASSERT_FIXED_SIZE(Container);
    }

    /**
     * @brief EigenMatrix
     * @param x_size
     * @param y_size
     */
    EigenMatrix(size_t x_size,
                size_t y_size)
      : container_(TSize, x_size * y_size),
        x_size_(x_size),
        y_size_(y_size)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
      EIGEN_STATIC_ASSERT_DYNAMIC_SIZE(Container);
      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
    }

    /**
     * @brief EigenMatrix
     * @param value
     */
    explicit EigenMatrix(const EigenT_ & value)
      : container_(Container::Zero()),
        x_size_(XSize_),
        y_size_(YSize_)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(Container);
      container_.colwise() += value;
    }

    /**
     * @brief EigenMatrix
     * @param x_size
     * @param y_size
     * @param value
     */
    EigenMatrix(size_t x_size,
                size_t y_size,
                const EigenT_ & value)
      : container_(Container::Zero(TSize, x_size * y_size)),
        x_size_(x_size),
        y_size_(y_size)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
      EIGEN_STATIC_ASSERT_DYNAMIC_SIZE(Container);
      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
      container_.colwise() += value;
    }

    /**
     * @brief EigenMatrix
     * @param data
     */
    explicit EigenMatrix(const Container & container)
      : container_(container),
        x_size_(XSize_),
        y_size_(YSize_)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(EigenT_);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(Container); // TODO also for Dynamic size?
      assert(container.size() == Size);
    }

    /**
     * @brief size
     * @return
     */
    inline size_t size() const
    {
      return container_.cols();
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
     * @brief resize
     * @param x_size
     * @param y_size
     */
    void resize(size_t x_size,
                size_t y_size)
    {
      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
      container_.conservativeResize(TSize, x_size * y_size);
      x_size_ = x_size;
      y_size_ = y_size;
    }

    /**
     * @brief reshape
     * @param x_size
     * @param y_size
     */
    inline void reshape(size_t x_size,
                        size_t y_size)
    {
      EIGEN_STATIC_ASSERT(XSize_ == Eigen::Dynamic and YSize_ == Eigen::Dynamic,
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
      return container_.col(index);
    }

    /**
     * @brief operator []
     * @param index
     * @return
     */
    inline Element operator [](size_t index)
    {
      return container_.col(index);
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
      return container_.col(y_index * x_size_ + x_index);
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
      return container_.col(y_index * x_size_ + x_index);
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

    size_t x_size_;
    size_t y_size_;

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
    PointMatrix(size_t x_size,
                size_t y_size)
      : Base(x_size, y_size)
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
     * @param x_size
     * @param y_size
     * @param value
     */
    template <typename OtherDerived>
      PointMatrix(size_t x_size,
                  size_t y_size,
                  const Eigen::DenseBase<OtherDerived> & value)
        : Base(Point(value), x_size, y_size)
      {
        // Do nothing
      }

    /**
     * @brief PointMatrix
     * @param container
     */
    explicit PointMatrix(const Container & container)
      : Base(container)
    {
      // Do nothing
    }

    /**
     * @brief PointMatrix
     * @param x_size
     * @param y_size
     * @param container
     */
    PointMatrix(size_t x_size,
                size_t y_size,
                const Container & container)
      : Base(x_size, y_size, container)
    {
      // Do nothing
    }

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

//template <typename ScalarT_, int Dimension_, int XSize_ = Eigen::Dynamic, int YSize_ = Eigen::Dynamic>
//  class PointMatrix
//  {
//  public:

//    static const int Size = (XSize_ == Eigen::Dynamic || YSize_ == Eigen::Dynamic) ? Eigen::Dynamic : XSize_ * YSize_;
//    static const int Options = (Dimension_ > 1) ? Eigen::ColMajor : Eigen::RowMajor;

//    typedef boost::shared_ptr<PointMatrix> Ptr;
//    typedef boost::shared_ptr<const PointMatrix> ConstPtr;

//    typedef Eigen::Matrix<ScalarT_, Dimension_, Size, Options, Dimension_> Container;
//    typedef Eigen::Transform<ScalarT_, 3, Eigen::Affine> Transform;

//    typedef typename Container::ColXpr Element;
//    typedef typename Container::ConstColXpr ConstElement;

//    PointMatrix()
//      : x_size_(XSize_ == Eigen::Dynamic ? 0 : XSize_),
//        y_size_(YSize_ == Eigen::Dynamic ? 0 : YSize_)
//    {
//      //EIGEN_STATIC_ASSERT_FIXED_SIZE(Data);
//    }

//    PointMatrix(size_t x_size,
//                size_t y_size)
//      : container_(Dimension_, x_size * y_size),
//        x_size_(x_size),
//        y_size_(y_size)
//    {
//      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
//      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
//    }

//    template <typename OtherDerived>
//      explicit PointMatrix(const Eigen::DenseBase<OtherDerived> & value)
//        : x_size_(XSize_),
//          y_size_(YSize_)
//      {
//        EIGEN_STATIC_ASSERT_FIXED_SIZE(Container);
//        container_.colwise() = value;
//      }

//    template <typename OtherDerived>
//      PointMatrix(size_t x_size,
//                  size_t y_size,
//                  const Eigen::DenseBase<OtherDerived> & value)
//        : container_(Dimension_, x_size * y_size),
//          x_size_(x_size),
//          y_size_(y_size)
//      {
//        assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
//        assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
//        container_.colwise() = value;
//      }

//    explicit PointMatrix(const Container & points)
//      : container_(points),
//        x_size_(XSize_),
//        y_size_(YSize_)
//    {
//      EIGEN_STATIC_ASSERT_FIXED_SIZE(Container);
//    }

//    PointMatrix(size_t x_size,
//                size_t y_size,
//                const Container & points)
//      : container_(points),
//        x_size_(x_size),
//        y_size_(y_size)
//    {
//      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
//      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
//      assert(x_size * y_size == points.cols());
//    }

//    size_t size() const
//    {
//      return size_t(container_.cols());
//    }

//    size_t xSize() const
//    {
//      return x_size_;
//    }

//    size_t ySize() const
//    {
//      return y_size_;
//    }

//    void setData(const Container & points)
//    {
//      assert(size() == points.cols());
//      container_ = points;
//    }

//    void resize(size_t x_size,
//                size_t y_size)
//    {
//      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
//      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
//      container_.conservativeResize(Dimension_, x_size * y_size);
//      x_size_ = x_size;
//      y_size_ = y_size;
//    }

//    void reshape(size_t x_size,
//                 size_t y_size)
//    {
//      assert(XSize_ == Eigen::Dynamic || x_size == XSize_);
//      assert(YSize_ == Eigen::Dynamic || y_size == YSize_);
//      assert(x_size * y_size == size());
//      x_size_ = x_size;
//      y_size_ = y_size;
//    }

//    Element operator [](size_t index)
//    {
//      return container_.col(index);
//    }

//    const ConstElement operator [](size_t index) const
//    {
//      return container_.col(index);
//    }

//    const ConstElement at(size_t x_index,
//                          size_t y_index) const
//    {
//      assert(y_index >= 0 && y_index < y_size_);
//      assert(x_index >= 0 && x_index < x_size_);
//      return operator ()(x_index, y_index);
//    }

//    Element at(size_t x_index,
//               size_t y_index)
//    {
//      assert(y_index >= 0 && y_index < y_size_);
//      assert(x_index >= 0 && x_index < x_size_);
//      return operator ()(x_index, y_index);
//    }

//    const ConstElement operator ()(size_t x_index,
//                                   size_t y_index) const
//    {
//      return operator [](y_index * x_size_ + x_index);
//    }

//    Element operator ()(size_t x_index,
//                        size_t y_index)
//    {
//      return operator [](y_index * x_size_ + x_index);
//    }

//    void transform(const Eigen::Transform<ScalarT_, 3, Eigen::Affine> & transform)
//    {
//      EIGEN_STATIC_ASSERT(Dimension_ == 3, THIS_METHOD_IS_ONLY_FOR_OBJECTS_OF_A_SPECIFIC_SIZE);
//      container_ = transform * container_;
//    }

//    Container & container()
//    {
//      return container_;
//    }

//    const Container & container() const
//    {
//      return container_;
//    }

//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//  private:

//    Container container_;

//    size_t x_size_;
//    size_t y_size_;

//  };

//template <typename ScalarT_, int Dimension_, int XSize_, int YSize_>
//  const int PointMatrix<ScalarT_, Dimension_, XSize_, YSize_>::Size;

//template <typename ScalarT_, int Dimension_, int XSize_, int YSize_>
//  const int PointMatrix<ScalarT_, Dimension_, XSize_, YSize_>::Options;

} /* namespace calibration */

#endif /* CALIBRATION_COMMON_BASE_POINT_MATRIX_H_ */
