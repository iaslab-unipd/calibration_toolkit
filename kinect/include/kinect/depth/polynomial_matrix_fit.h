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

template <typename UndistortionT_>
  class PolynomialUndistortionMatrixFit_ : public DepthUndistortionModelFitImpl<UndistortionT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrixFit_> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrixFit_> ConstPtr;

    typedef DepthUndistortionModelFitImpl<UndistortionT_> Base;

    typedef typename UndistortionT_::Model::Poly Poly;
    typedef typename UndistortionT_::Model::Scalar Scalar;

    static const int Size = MathTraits<Poly>::Size;
    static const int MinDegree = MathTraits<Poly>::MinDegree;
    static const int Degree = MathTraits<Poly>::Degree;

    typedef typename Types<Scalar>::Plane Plane;
    typedef typename Base::PointDistorsionBin PointDistorsionBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    PolynomialUndistortionMatrixFit_()
    {
      // Do nothing
    }

    explicit PolynomialUndistortionMatrixFit_(const typename UndistortionT_::Ptr & model_impl)
      : model_impl_(model_impl),
        distorsion_bins_(model_impl->model()->data()->xSize(), model_impl->model()->data()->ySize()),
        accumulation_bins_(model_impl->model()->data()->xSize(), model_impl->model()->data()->ySize())
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionMatrixFit_()
    {
      // Do nothing
    }

    virtual void setModelImpl(const typename UndistortionT_::Ptr & model_impl)
    {
      model_impl_ = model_impl;
      distorsion_bins_ = Matrix<typename Base::PointDistorsionBin>(model_impl->model()->data()->xSize(),
                                                                   model_impl->model()->data()->ySize());
      accumulation_bins_ = Matrix<typename Base::AccumulationBin>(model_impl->model()->data()->xSize(),
                                                                  model_impl->model()->data()->ySize());
    }

    virtual const typename UndistortionT_::Ptr & modelImpl() const
    {
      return model_impl_;
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

    typename UndistortionT_::Ptr model_impl_;

  };

template <typename PolynomialT_, typename ScalarT_ = typename MathTraits<PolynomialT_>::Scalar>
  class PolynomialUndistortionMatrixFitEigen : public PolynomialUndistortionMatrixFit_<DepthUndistortionImpl<PolynomialMatrixProjectedModel<PolynomialT_>, DepthEigen_<ScalarT_> > >,
                                               public DepthUndistortionModelFit<DepthEigen_<ScalarT_> >
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrixFitEigen> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrixFitEigen> ConstPtr;

    typedef typename MathTraits<PolynomialT_>::Scalar Scalar;

    typedef DepthUndistortionImpl<PolynomialMatrixProjectedModel<PolynomialT_>, DepthEigen_<ScalarT_> > UndistortionModel;

    typedef PolynomialUndistortionMatrixFit_<UndistortionModel> Base;
    typedef typename Base::PointDistorsionBin PointDistorsionBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    typedef DepthUndistortionModelFit<DepthEigen_<Scalar> > Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;
    typedef typename Interface::Plane Plane;

    PolynomialUndistortionMatrixFitEigen()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialUndistortionMatrixFitEigen(const typename UndistortionModel::Ptr & model)
      : Base(model),
        model_(model)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionMatrixFitEigen()
    {
      // Do nothing
    }

    virtual void setModel(const typename UndistortionModel::Ptr & model)
    {
      Base::setModelImpl(model);
      model_ = model;
    }

    virtual const typename DepthUndistortion<DepthEigen_<Scalar> >::Ptr & model() const
    {
      return model_;
    }

    virtual void accumulateCloud(const Cloud & cloud)
    {
      for (size_t i = 0; i < cloud.size(); ++i)
        accumulatePoint(cloud[i]);
    }

    virtual void accumulateCloud(const Cloud & cloud,
                                 const std::vector<int> & indices)
    {
      for (size_t i = 0; i < indices.size(); ++i)
        accumulatePoint(cloud[indices[i]]);
    }

    virtual void accumulatePoint(const Point & point)
    {
      assert(Base::model_impl_);
      size_t x_index, y_index;
      Base::model_impl_->getIndex(UndistortionModel::toSphericalCoordinates(point), x_index, y_index);
      Base::accumulation_bins_(x_index, y_index) += point;
    }

    virtual void addPoint(const Point & point,
                          const Plane & plane);

    virtual void addAccumulatedPoints(const Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    virtual void update()
    {
      Base::update();
    }

  protected:

    typename DepthUndistortion<DepthEigen_<Scalar> >::Ptr model_;

  };

template <typename PolynomialT_, typename PCLPointT_, typename ScalarT_ = typename MathTraits<PolynomialT_>::Scalar>
  class PolynomialUndistortionMatrixFitPCL : public PolynomialUndistortionMatrixFit_<DepthUndistortionImpl<PolynomialMatrixProjectedModel<PolynomialT_>, DepthPCL_<PCLPointT_> > >,
                                             public DepthUndistortionModelFit<DepthPCL_<PCLPointT_>, ScalarT_>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrixFitPCL> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrixFitPCL> ConstPtr;

    typedef ScalarT_ Scalar;

    typedef DepthUndistortionImpl<PolynomialMatrixProjectedModel<PolynomialT_>, DepthPCL_<PCLPointT_> > UndistortionModel;

    typedef PolynomialUndistortionMatrixFit_<UndistortionModel> Base;
    typedef typename Base::PointDistorsionBin PointDistorsionBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    typedef DepthUndistortionModelFit<DepthPCL_<PCLPointT_>, Scalar> Interface;
    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;
    typedef typename Interface::Plane Plane;

    PolynomialUndistortionMatrixFitPCL()
      : Base()
    {
      // Do nothing
    }

    explicit PolynomialUndistortionMatrixFitPCL(const typename UndistortionModel::Ptr & model)
      : Base(model),
        model_(model)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionMatrixFitPCL()
    {
      // Do nothing
    }

    virtual void setModel(const typename UndistortionModel::Ptr & model)
    {
      Base::setModelImpl(model);
      model_ = model;
    }

    virtual const typename DepthUndistortion<DepthPCL_<PCLPointT_> >::Ptr & model() const
    {
      return model_;
    }

    virtual void accumulateCloud(const Cloud & cloud)
    {
      for (size_t i = 0; i < cloud.points.size(); ++i)
        accumulatePoint(cloud.points[i]);
    }

    virtual void accumulateCloud(const Cloud & cloud,
                                 const std::vector<int> & indices)
    {
      for (size_t i = 0; i < indices.size(); ++i)
        accumulatePoint(cloud.points[indices[i]]);
    }

    virtual void accumulatePoint(const Point & point)
    {
      assert(Base::model_impl_);
      if (not pcl::isFinite(point))
        return;

      size_t x_index, y_index;
      Base::model_impl_->model()->getIndex(UndistortionModel::project(point), x_index, y_index);
      Base::accumulation_bins_.at(x_index, y_index) += typename Types<Scalar>::Point3(point.x, point.y, point.z);
    }

    virtual void addPoint(const Point & point,
                          const Plane & plane);

    virtual void addAccumulatedPoints(const Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    virtual void update()
    {
      Base::update();
    }

  protected:

    typename DepthUndistortion<DepthPCL_<Point> >::Ptr model_;

  };

} /* namespace calibration */

#include <impl/kinect/depth/polynomial_matrix_fit.hpp>

#endif /* KINECT_DEPTH_POLYNOMIAL_MATRIX_FIT_H_ */
