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
    typedef PolynomialT_ PolynomialT;
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

    typedef typename Traits::PolynomialT PolynomialT;
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

    virtual void setMatrix(const typename Data::Ptr & matrix) = 0;
    virtual typename Data::Ptr createMatrix(const Size2 & bin_size) = 0;
    virtual typename Data::Ptr createMatrix(const Size2 & bin_size,
                                            const typename MathTraits<PolynomialT_>::Coefficients & value) = 0;

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

    inline virtual void setMatrix(const typename Data::Ptr & matrix,
                                  const Size2 & bin_size)
    {
      matrix_ = matrix;
      bin_size_ = bin_size;
    }

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
    typedef typename Base::PolynomialT PolynomialT;
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

    typedef typename Traits::PolynomialT PolynomialT;
    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::Data Data;

    typedef typename Data::Element Element;
    typedef typename Data::ConstElement ConstElement;

    PolynomialMatrixSimpleModel(const Size2 & image_size)
      : Base(image_size)
    {
      // Do nothing
    }

    inline virtual void setMatrix(const typename Data::Ptr & matrix)
    {
      Base::setMatrix(matrix, Base::image_size_ / matrix->size());
    }

    inline virtual typename Data::Ptr createMatrix(const Size2 & bin_size)
    {
      assert((bin_size <= Base::image_size_).all() and (bin_size > Size2(0, 0)).all());
      return boost::make_shared<Data>(Size2(Base::image_size_ / bin_size));
    }

    inline virtual typename Data::Ptr createMatrix(const Size2 & bin_size,
                                                   const typename MathTraits<PolynomialT_>::Coefficients & value)
    {
      assert((bin_size <= Base::image_size_).all() and (bin_size > Size2(0, 0)).all());
      return boost::make_shared<Data>(Size2(Base::image_size_ / bin_size), value);
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
      depth = PolynomialT::evaluate(Base::polynomial(matrixIndex(x_index, y_index)), depth);
    }

    inline void undistort(const Size2 & index,
                          Scalar & depth) const
    {
      depth = PolynomialT::evaluate(Base::polynomial(matrixIndex(index)), depth);
    }

  };

template <typename PolynomialT_>
  class PolynomialMatrixSmoothModel;

template <typename PolynomialT_>
  struct ModelTraits<PolynomialMatrixSmoothModel<PolynomialT_> > : public ModelTraits<PolynomialMatrixModel_<PolynomialT_> >
  {
    typedef ModelTraits<PolynomialMatrixModel_<PolynomialT_> > Base;
    typedef typename Base::PolynomialT PolynomialT;
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

    typedef typename Traits::PolynomialT PolynomialT;
    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::Data Data;

    typedef typename Data::Element Element;
    typedef typename Data::ConstElement ConstElement;

    struct LookupTableData
    {
      LookupTableData(Size2 index, Scalar weight) : index_(index), weight_(weight) {}
      Size2 index_;
      Scalar weight_;
    };
    typedef Matrix<std::vector<LookupTableData> > LookupTable;

    PolynomialMatrixSmoothModel(const Size2 & image_size)
      : Base(image_size),
        lookup_table_(image_size)
    {
      // Do nothing
    }

    void createLookupTable()
    {

      for (Size1 x_index = 0; x_index < lookup_table_.size().x(); ++x_index)
      {
        for (Size1 y_index = 0; y_index < lookup_table_.size().y(); ++y_index)
        {
          std::vector<LookupTableData> & lt_data = lookup_table_(x_index, y_index);
          lt_data.clear();

          Size2 index(x_index, y_index);
          Eigen::Array<Size1, 2, 2> bin;
          Eigen::Array<Scalar, 2, 2> weight;

          bin.col(0) = index / Base::bin_size_;
          bin.col(1) = bin.col(0) + Size2(1, 1);

          weight.col(1) = ((index - bin.col(0) * Base::bin_size_).template cast<Scalar>() / Base::bin_size_.template cast<Scalar>());
          weight.col(0) = Eigen::Array<Scalar, 2, 1>(Scalar(1.0), Scalar(1.0)) - weight.col(1);

          for (Size1 i = 0; i < 2; ++i)
            for (Size1 j = 0; j < 2; ++j)
              //if (weight(0, i) * weight(1, j) > 0)
                lt_data.push_back(LookupTableData(Size2(bin(0, i), bin(1, j)), weight(0, i) * weight(1, j)));

        }
      }
    }

    inline virtual typename Data::Ptr createMatrix(const Size2 & bin_size)
    {
      assert((bin_size <= Base::image_size_).all() and (bin_size > Size2(0, 0)).all());
      assert(bin_size.x() % 2 == 0 and bin_size.y() % 2 == 0);
      return boost::make_shared<Data>(Size2(Base::image_size_ / bin_size + Size2(1, 1)));
    }

    inline virtual typename Data::Ptr createMatrix(const Size2 & bin_size,
                                                   const typename MathTraits<PolynomialT_>::Coefficients & value)
    {
      assert((bin_size <= Base::image_size_).all() and (bin_size > Size2(0, 0)).all());
      assert(bin_size.x() % 2 == 0 and bin_size.y() % 2 == 0);
      return boost::make_shared<Data>(Size2(Base::image_size_ / bin_size + Size2(1, 1)), value);
    }

    inline virtual void setMatrix(const typename Data::Ptr & matrix)
    {
      Base::setMatrix(matrix, Base::image_size_ / (matrix->size() - Size2(1, 1)));
      assert((Base::bin_size_.x() % 2 == 0 and Base::bin_size_.y() % 2 == 0) or (Base::bin_size_ == Size2(1, 1)).all());
      createLookupTable();
    }

    inline const std::vector<LookupTableData> & lookupTable(Size1 x_index,
                                                            Size1 y_index) const
    {
      return lookup_table_.at(x_index, y_index);
    }

    inline const std::vector<LookupTableData> & lookupTable(Size2 index) const
    {
      return lookup_table_.at(index);
    }

    inline void undistort(size_t x_index,
                          size_t y_index,
                          Scalar & depth) const
    {
      undistort_(lookup_table_.at(x_index, y_index), depth);
    }

    inline void undistort(const Size2 & index,
                          Scalar & depth) const
    {
      undistort_(lookup_table_.at(index), depth);
    }

    inline void undistort_(const std::vector<LookupTableData> & lt_data,
                           Scalar & depth) const
    {
      Scalar tmp_depth = 0.0;
      Scalar weight_sum = 0.0;
      for (Size1 i = 0; i < lt_data.size(); ++i)
      {
        weight_sum += lt_data[i].weight_;
        tmp_depth += lt_data[i].weight_ * PolynomialT::evaluate(Base::polynomial(lt_data[i].index_), depth);
      }
      depth = tmp_depth / weight_sum;
    }

  protected:

    LookupTable lookup_table_;

  };

} /* namespace calibration */
#endif /* KINECT_DEPTH_POLYNOMIAL_MATRIX_MODEL_H_ */
