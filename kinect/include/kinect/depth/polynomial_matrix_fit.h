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

template <typename ModelT_>
  class PolynomialMatrixModelFit_
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixModelFit_> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixModelFit_> ConstPtr;

    typedef typename ModelTraits<ModelT_>::Poly Poly;
    typedef typename ModelTraits<ModelT_>::Scalar Scalar;

    static const int Size = MathTraits<Poly>::Size;
    static const int MinDegree = MathTraits<Poly>::MinDegree;
    static const int Degree = MathTraits<Poly>::Degree;

    typedef typename Types<Scalar>::Plane Plane;

    typedef std::vector<std::pair<Scalar, Scalar> > PointDistorsionBin;

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

    PolynomialMatrixModelFit_()
    {
      // Do nothing
    }

    explicit PolynomialMatrixModelFit_(const typename ModelT_::Ptr & model)
      : model_(model),
        distorsion_bins_(model->matrix()->size()),
        accumulation_bins_(model->matrix()->size())
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixModelFit_()
    {
      // Do nothing
    }

    void setModel(const typename ModelT_::Ptr & model)
    {
      model_ = model;
      distorsion_bins_ = Matrix<PointDistorsionBin>(model->matrix()->size());
      accumulation_bins_ = Matrix<AccumulationBin>(model->matrix()->size());
    }

    const typename ModelT_::Ptr & model() const
    {
      return model_;
    }

    const PointDistorsionBin & getSamples(size_t x_index,
                                          size_t y_index) const
    {
      return distorsion_bins_(x_index, y_index);
    }

    virtual void addAccumulatedPoints(const Plane & plane);

    virtual void update();

  protected:

    Matrix<PointDistorsionBin> distorsion_bins_;
    Matrix<AccumulationBin> accumulation_bins_;

    typename ModelT_::Ptr model_;

  };

template <typename ModelT_, typename ScalarT_>
  class PolynomialMatrixModelFitEigen : public PolynomialMatrixModelFit_<ModelT_>,
                                        virtual public DepthUndistortionModelFit<ModelT_, DepthEigen_<ScalarT_>, ScalarT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixModelFitEigen> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixModelFitEigen> ConstPtr;

    typedef ScalarT_ Scalar;
    typedef ModelT_ Model;

    typedef PolynomialMatrixModelFit_<Model> Base;
    typedef typename Base::PointDistorsionBin PointDistorsionBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    typedef DepthUndistortionModelFit<Model, DepthEigen_<Scalar>, Scalar> Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;
    typedef typename Interface::Plane Plane;

    PolynomialMatrixModelFitEigen()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialMatrixModelFitEigen(const typename Model::Ptr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixModelFitEigen()
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

//    virtual void accumulateCloud(const Cloud & cloud,
//                                 const std::vector<int> & indices)
//    {
//      assert(cloud.isOrganized());
//      for (Size1 i = 0; i < indices.size(); ++i)
//        accumulatePoint(indices[i] % cloud.width, indices[i] / cloud.width, cloud.points[indices[i]]);
//    }

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

template <typename ModelT_, typename ScalarT_, typename PCLPointT_>
  class PolynomialMatrixModelFitPCL : public PolynomialMatrixModelFit_<ModelT_>,
                                      virtual public DepthUndistortionModelFit<ModelT_, DepthPCL_<PCLPointT_>, ScalarT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialMatrixModelFitPCL> Ptr;
    typedef boost::shared_ptr<const PolynomialMatrixModelFitPCL> ConstPtr;

    typedef ScalarT_ Scalar;
    typedef ModelT_ Model;

    typedef PolynomialMatrixModelFit_<Model> Base;
    typedef typename Base::PointDistorsionBin PointDistorsionBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    typedef DepthUndistortionModelFit<Model, DepthPCL_<PCLPointT_>, Scalar> Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;
    typedef typename Interface::Plane Plane;

    PolynomialMatrixModelFitPCL()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialMatrixModelFitPCL(const typename Model::Ptr & model)
      : Base(model)
    {
      // Do nothing
    }

    virtual ~PolynomialMatrixModelFitPCL()
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

} /* namespace calibration */

#include <impl/kinect/depth/polynomial_matrix_fit.hpp>

#endif /* KINECT_DEPTH_POLYNOMIAL_MATRIX_FIT_H_ */
