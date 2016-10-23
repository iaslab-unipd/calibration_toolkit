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

#ifndef KINECT_DEPTH_POLYNOMIAL_MATRIX_FIT_H_
#define KINECT_DEPTH_POLYNOMIAL_MATRIX_FIT_H_

#include <Eigen/Dense>
#include <pcl/common/point_tests.h>
#include <calibration_common/ceres/polynomial_fit.h>
#include <calibration_common/depth/undistortion_model_fit.h>
#include <kinect/depth/polynomial_matrix.h>

namespace calibration
{

template <typename PolynomialT_>
  class PolynomialMatrixSimpleModelFit_
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixSimpleModelFit_> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixSimpleModelFit_> ConstPtr;

    typedef PolynomialMatrixSimpleModel<PolynomialT_> ModelT;

    typedef typename ModelTraits<ModelT>::PolynomialT PolynomialT;
    typedef typename ModelTraits<ModelT>::Scalar Scalar;

    static const int Size = MathTraits<PolynomialT_>::Size;
    static const int MinDegree = MathTraits<PolynomialT_>::MinDegree;
    static const int Degree = MathTraits<PolynomialT_>::Degree;

    typedef typename Types<Scalar>::Plane Plane;

    typedef std::vector<std::pair<Scalar, Scalar> > DataBin;

    /**
     * @brief The AccumulationBin class
     */
    class AccumulationBin
    {
    public:

      typedef typename Types<Scalar>::Point3 Point;

      /**
       * @brief AccumulationBin
       */
      AccumulationBin()
        : sum_(Point::Zero()),
          n_(0)
      {
        // Do nothing
      }

      /**
       * @brief reset
       */
      void reset()
      {
        sum_ = Point::Zero();
        n_ = 0;
      }

      /**
       * @brief operator +=
       * @param point
       * @return
       */
      AccumulationBin & operator +=(const Point & point)
      {
        sum_ += point;
        ++n_;
        return *this;
      }

      /**
       * @brief isEmpty
       * @return
       */
      bool isEmpty()
      {
        return n_ == 0;
      }

      /**
       * @brief average
       * @return
       */
      Point average()
      {
        return sum_ / Scalar(n_);
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

      Point sum_;
      int n_;

    };

    PolynomialMatrixSimpleModelFit_()
    {
      // Do nothing
    }

    explicit PolynomialMatrixSimpleModelFit_(const typename ModelT::Ptr & model)
      : model_(model),
        data_bins_(model->matrix()->size()),
        accumulation_bins_(model->matrix()->size())
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixSimpleModelFit_()
    {
      // Do nothing
    }

    void setModel(const typename ModelT::Ptr & model)
    {
      model_ = model;
      data_bins_ = Matrix<DataBin>(model->matrix()->size());
      accumulation_bins_ = Matrix<AccumulationBin>(model->matrix()->size());
    }

    const typename ModelT::Ptr & model() const
    {
      return model_;
    }

    const DataBin & getSamples(Size1 x_index,
                               Size1 y_index) const
    {
      return data_bins_(x_index, y_index);
    }

    virtual void addAccumulatedPoints(const Plane & plane);

    virtual void update();

  protected:

    Matrix<DataBin> data_bins_;
    Matrix<AccumulationBin> accumulation_bins_;

    typename ModelT::Ptr model_;

  };

template <typename PolynomialT_, typename ScalarT_>
  class PolynomialMatrixSimpleModelFitEigen : public PolynomialMatrixSimpleModelFit_<PolynomialT_>,
                                              virtual public DepthUndistortionModelFit<DepthEigen_<ScalarT_>, ScalarT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixSimpleModelFitEigen> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixSimpleModelFitEigen> ConstPtr;

    typedef ScalarT_ Scalar;

    typedef PolynomialMatrixSimpleModelFit_<PolynomialT_> Base;
    typedef typename Base::ModelT ModelT;
    typedef typename Base::DataBin DataBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    typedef DepthUndistortionModelFit<DepthEigen_<Scalar>, Scalar> Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;
    typedef typename Interface::Plane Plane;

    PolynomialMatrixSimpleModelFitEigen()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialMatrixSimpleModelFitEigen(const typename ModelT::Ptr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixSimpleModelFitEigen()
    {
      // Do nothing
    }

    inline virtual void accumulateCloud(const Cloud & cloud)
    {
      assert(cloud.isOrganized());
      for (Size1 i = 0; i < cloud.size().x(); ++i)
        for (Size1 j = 0; j < cloud.size().y(); ++j)
          accumulatePoint(i, j, cloud(i, j));
    }

    virtual void accumulateCloud(const Cloud & cloud,
                                 const std::vector<int> & indices)
    {
      assert(cloud.isOrganized());
      for (Size1 i = 0; i < indices.size(); ++i)
        accumulatePoint(indices[i] % cloud.size().x(), indices[i] / cloud.size().x(), cloud[indices[i]]);
    }

    inline virtual void accumulatePoint(Size1 x_index,
                                        Size1 y_index,
                                        const Point & point)
    {
      assert(Base::model());
      if (point.hasNaN())
        return;

      Size2 index = Base::model()->matrixIndex(x_index, y_index);
      Base::accumulation_bins_.at(index) += point;
    }

    virtual void addPoint(Size1 x_index,
                          Size1 y_index,
                          const Point & point,
                          const Plane & plane);

    inline virtual void addAccumulatedPoints(const Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    inline virtual void update()
    {
      Base::update();
    }

  };

template <typename PolynomialT_, typename ScalarT_, typename PCLPointT_>
  class PolynomialMatrixSimpleModelFitPCL : public PolynomialMatrixSimpleModelFit_<PolynomialT_>,
                                            virtual public DepthUndistortionModelFit<DepthPCL_<PCLPointT_>, ScalarT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixSimpleModelFitPCL> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixSimpleModelFitPCL> ConstPtr;

    typedef ScalarT_ Scalar;

    typedef PolynomialMatrixSimpleModelFit_<PolynomialT_> Base;
    typedef typename Base::ModelT ModelT;
    typedef typename Base::DataBin DataBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    typedef DepthUndistortionModelFit<DepthPCL_<PCLPointT_>, Scalar> Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;
    typedef typename Interface::Plane Plane;

    PolynomialMatrixSimpleModelFitPCL()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialMatrixSimpleModelFitPCL(const typename ModelT::Ptr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixSimpleModelFitPCL()
    {
      // Do nothing
    }

    inline virtual void accumulateCloud(const Cloud & cloud)
    {
      for (Size1 i = 0; i < cloud.width; ++i)
        for (Size1 j = 0; j < cloud.height; ++j)
          accumulatePoint(i, j, cloud(i, j));
    }

    inline virtual void accumulateCloud(const Cloud & cloud,
                                        const std::vector<int> & indices)
    {
      assert(cloud.isOrganized());
      for (Size1 i = 0; i < indices.size(); ++i)
        accumulatePoint(indices[i] % cloud.width, indices[i] / cloud.width, cloud.points[indices[i]]);
    }

    inline virtual void accumulatePoint(Size1 x_index,
                                        Size1 y_index,
                                        const Point & point)
    {
      assert(Base::model());
      if (not pcl::isFinite(point))
        return;

      Size2 index = Base::model()->matrixIndex(x_index, y_index);
      Base::accumulation_bins_.at(index) += typename Types<Scalar>::Point3(point.x, point.y, point.z);
    }

    virtual void addPoint(Size1 x_index,
                          Size1 y_index,
                          const Point & point,
                          const Plane & plane);

    inline virtual void addAccumulatedPoints(const Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    inline virtual void update()
    {
      Base::update();
    }

  };


template <typename PolynomialT_>
  class PolynomialMatrixSmoothModelFit_
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixSmoothModelFit_> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixSmoothModelFit_> ConstPtr;

    typedef PolynomialMatrixSmoothModel<PolynomialT_> ModelT;

    typedef typename ModelTraits<ModelT>::PolynomialT PolynomialT;
    typedef typename ModelTraits<ModelT>::Scalar Scalar;

    static const int Size = MathTraits<PolynomialT_>::Size;
    static const int MinDegree = MathTraits<PolynomialT_>::MinDegree;
    static const int Degree = MathTraits<PolynomialT_>::Degree;

    typedef typename Types<Scalar>::Plane Plane;

    struct Data
    {
      Data(Scalar x, Scalar y, Scalar weight) : x_(x), y_(y), weight_(weight) {}
      Scalar x_;
      Scalar y_;
      Scalar weight_;
    };
    typedef std::vector<Data> DataBin;

    struct AccumulationBinData
    {
      typedef typename Types<Scalar>::Point3 Point;
      AccumulationBinData(Point point, Scalar weight) : point_(point), weight_(weight) {}
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Point point_;
      Scalar weight_;
    };
    typedef std::vector<AccumulationBinData> AccumulationBin;

    PolynomialMatrixSmoothModelFit_()
    {
      // Do nothing
    }

    explicit PolynomialMatrixSmoothModelFit_(const typename ModelT::Ptr & model)
      : model_(model),
        data_bins_(model->matrix()->size()),
        accumulation_bins_(model->matrix()->size())
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixSmoothModelFit_()
    {
      // Do nothing
    }

    void setModel(const typename ModelT::Ptr & model)
    {
      model_ = model;
      data_bins_ = Matrix<DataBin>(model->matrix()->size());
      accumulation_bins_ = Matrix<AccumulationBin>(model->matrix()->size());
    }

    const typename ModelT::Ptr & model() const
    {
      return model_;
    }

    const DataBin & getSamples(Size1 x_index,
                               Size1 y_index) const
    {
      return data_bins_(x_index, y_index);
    }

    virtual void addAccumulatedPoints(const Plane & plane);

    virtual void update();

    virtual void reset()
    {
      data_bins_ = Matrix<DataBin>(model_->matrix()->size());
      accumulation_bins_ = Matrix<AccumulationBin>(model_->matrix()->size());
    }

    void setDepthErrorFunction(const Polynomial<Scalar, 2> & depth_error_function)
    {
      depth_error_function_ = depth_error_function;
    }

  protected:

    Matrix<DataBin> data_bins_;
    Matrix<AccumulationBin> accumulation_bins_;

    typename ModelT::Ptr model_;
    Polynomial<Scalar, 2> depth_error_function_;

  };

template <typename PolynomialT_, typename ScalarT_>
  class PolynomialMatrixSmoothModelFitEigen : public PolynomialMatrixSmoothModelFit_<PolynomialT_>,
                                              virtual public DepthUndistortionModelFit<DepthEigen_<ScalarT_>, ScalarT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixSmoothModelFitEigen> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixSmoothModelFitEigen> ConstPtr;

    typedef ScalarT_ Scalar;

    typedef PolynomialMatrixSmoothModelFit_<PolynomialT_> Base;
    typedef typename Base::ModelT ModelT;
    typedef typename Base::Data Data;
    typedef typename Base::DataBin DataBin;
    typedef typename Base::AccumulationBinData AccumulationBinData;
    typedef typename Base::AccumulationBin AccumulationBin;

    typedef DepthUndistortionModelFit<DepthEigen_<Scalar>, Scalar> Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;
    typedef typename Interface::Plane Plane;

    PolynomialMatrixSmoothModelFitEigen()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialMatrixSmoothModelFitEigen(const typename ModelT::Ptr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixSmoothModelFitEigen()
    {
      // Do nothing
    }

    inline virtual void accumulateCloud(const Cloud & cloud)
    {
      assert(cloud.isOrganized());
      for (Size1 i = 0; i < cloud.size().x(); ++i)
        for (Size1 j = 0; j < cloud.size().y(); ++j)
          accumulatePoint(i, j, cloud(i, j));
    }

    virtual void accumulateCloud(const Cloud & cloud,
                                 const std::vector<int> & indices)
    {
      assert(cloud.isOrganized());
      for (Size1 i = 0; i < indices.size(); ++i)
        accumulatePoint(indices[i] % cloud.size().x(), indices[i] / cloud.size().x(), cloud[indices[i]]);
    }

    inline virtual void accumulatePoint(Size1 x_index,
                                        Size1 y_index,
                                        const Point & point)
    {
      assert(Base::model());
      if (point.hasNaN())
        return;

      const std::vector<typename ModelT::LookupTableData> & lt_data = Base::model()->lookupTable(x_index, y_index);
//      Size2 index = Base::model()->matrixIndex(x_index, y_index);
//      Base::accumulation_bins_.at(index).push_back(Data(point, weight));
      for (Size1 i = 0; i < lt_data.size(); ++i)
        Base::accumulation_bins_.at(lt_data[i].index_).push_back(AccumulationBinData(point, lt_data[i].weight_));

    }

    virtual void addPoint(Size1 x_index,
                          Size1 y_index,
                          const Point & point,
                          const Plane & plane);

    inline virtual void addAccumulatedPoints(const Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    inline virtual void update()
    {
      Base::update();
    }

  };

template <typename PolynomialT_, typename ScalarT_, typename PCLPointT_>
  class PolynomialMatrixSmoothModelFitPCL : public PolynomialMatrixSmoothModelFit_<PolynomialT_>,
                                            virtual public DepthUndistortionModelFit<DepthPCL_<PCLPointT_>, ScalarT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixSmoothModelFitPCL> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixSmoothModelFitPCL> ConstPtr;

    typedef ScalarT_ Scalar;

    typedef PolynomialMatrixSmoothModelFit_<PolynomialT_> Base;
    typedef typename Base::ModelT ModelT;
    typedef typename Base::Data Data;
    typedef typename Base::DataBin DataBin;
    typedef typename Base::AccumulationBinData AccumulationBinData;
    typedef typename Base::AccumulationBin AccumulationBin;

    typedef DepthUndistortionModelFit<DepthPCL_<PCLPointT_>, Scalar> Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;
    typedef typename Interface::Plane Plane;

    PolynomialMatrixSmoothModelFitPCL()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialMatrixSmoothModelFitPCL(const typename ModelT::Ptr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixSmoothModelFitPCL()
    {
      // Do nothing
    }

    inline virtual void accumulateCloud(const Cloud & cloud)
    {
      for (Size1 i = 0; i < cloud.width; ++i)
        for (Size1 j = 0; j < cloud.height; ++j)
          accumulatePoint(i, j, cloud(i, j));
    }

    inline virtual void accumulateCloud(const Cloud & cloud,
                                        const std::vector<int> & indices)
    {
      assert(cloud.isOrganized());
      for (Size1 i = 0; i < indices.size(); ++i)
        accumulatePoint(indices[i] % cloud.width, indices[i] / cloud.width, cloud.points[indices[i]]);
    }

    inline virtual void accumulatePoint(Size1 x_index,
                                        Size1 y_index,
                                        const Point & point)
    {
      assert(Base::model());
      if (not pcl::isFinite(point))
        return;

      const std::vector<typename ModelT::LookupTableData> & lt_data = Base::model()->lookupTable(x_index, y_index);
//      Size2 index = Base::model()->matrixIndex(x_index, y_index);
//      Base::accumulation_bins_.at(index) += typename Types<Scalar>::Point3(point.x, point.y, point.z);
      typename Types<Scalar>::Point3 eigen_point(point.x, point.y, point.z);
      for (Size1 i = 0; i < lt_data.size(); ++i)
        Base::accumulation_bins_.at(lt_data[i].index_).push_back(AccumulationBinData(eigen_point, lt_data[i].weight_));
    }

    virtual void addPoint(Size1 x_index,
                          Size1 y_index,
                          const Point & point,
                          const Plane & plane);

    inline virtual void addAccumulatedPoints(const Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    inline virtual void update()
    {
      Base::update();
    }

  };


} /* namespace calibration */

#include <impl/kinect/depth/polynomial_matrix_fit.hpp>

#endif /* KINECT_DEPTH_POLYNOMIAL_MATRIX_FIT_H_ */
