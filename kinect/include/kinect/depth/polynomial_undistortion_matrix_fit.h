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

#ifndef KINECT_DEPTH_POLYNOMIAL_UNDISTORTION_MATRIX_FIT_H_
#define KINECT_DEPTH_POLYNOMIAL_UNDISTORTION_MATRIX_FIT_H_

#include <Eigen/Dense>
#include <pcl/common/point_tests.h>
#include <calibration_common/ceres/polynomial_fit.h>
#include <calibration_common/base/traits.h>
#include <calibration_common/depth/undistortion_model_fit.h>
#include <kinect/depth/polynomial_undistortion_matrix.h>

namespace calibration
{

template <typename Polynomial_, typename Matrix_>
  class PolynomialUndistortionMatrixFit : public DepthUndistortionModelFit<typename Traits<Polynomial_>::Scalar>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrixFit> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrixFit> ConstPtr;

    typedef typename Traits<Polynomial_>::Scalar Scalar;
    typedef DepthUndistortionModelFit<Scalar> Base;

    typedef typename Base::PointDistorsionBin PointDistorsionBin;
    typedef typename Base::AccumulationBin AccumulationBin;

    static const int Size = Traits<Polynomial_>::Size;
    static const int MinDegree = Traits<Polynomial_>::MinDegree;
    static const int Degree = Traits<Polynomial_>::Degree;

    PolynomialUndistortionMatrixFit(size_t x_size,
                                    size_t y_size,
                                    Scalar x_fov,
                                    Scalar y_fov)
      : matrix_(boost::make_shared<Matrix_>(x_size, y_size, x_fov, y_fov)),
        distorsion_bins_(x_size, y_size),
        accumulation_bins_(x_size, y_size)
    {
      // Do nothing
    }

    PolynomialUndistortionMatrixFit(const typename Matrix_::Ptr & matrix)
      : matrix_(matrix),
        distorsion_bins_(matrix->data()->xSize(), matrix->data()->ySize()),
        accumulation_bins_(matrix->data()->xSize(), matrix->data()->ySize())
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionMatrixFit()
    {
      // Do nothing
    }

    void setMatrix(const typename Matrix_::Ptr & matrix)
    {
      matrix_ = matrix;
      distorsion_bins_ = Matrix<typename Base::PointDistorsionBin>(matrix->data()->xSize(), matrix->data()->ySize());
      accumulation_bins_ = Matrix<typename Base::AccumulationBin>(matrix->data()->xSize(), matrix->data()->ySize());
    }

    virtual void addAccumulatedPoints(const typename Types_<Scalar>::Plane & plane);

    virtual void update();

  protected:

    Matrix<PointDistorsionBin> distorsion_bins_;
    Matrix<AccumulationBin> accumulation_bins_;

    typename Matrix_::Ptr matrix_;

  };

template <typename Polynomial_>
  class PolynomialUndistortionMatrixFitEigen : public PolynomialUndistortionMatrixFit<Polynomial_,
                                                 PolynomialUndistortionMatrixEigen<Polynomial_> >,
                                               public DepthUndistortionModelFitEigen<
                                                 typename Traits<Polynomial_>::Scalar>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrixFitEigen> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrixFitEigen> ConstPtr;

    typedef typename Traits<Polynomial_>::Scalar Scalar;
    typedef PolynomialUndistortionMatrixEigen<Polynomial_> UndistortionMatrix;
    typedef PolynomialUndistortionMatrixFit<Polynomial_, UndistortionMatrix> Base;

    PolynomialUndistortionMatrixFitEigen(size_t x_size,
                                         size_t y_size,
                                         Scalar x_fov,
                                         Scalar y_fov)
      : Base(x_size, y_size, x_fov, y_fov)
    {
      // Do nothing
    }

    PolynomialUndistortionMatrixFitEigen(const typename UndistortionMatrix::Ptr & matrix)
      : Base(matrix)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionMatrixFitEigen()
    {
      // Do nothing
    }

    virtual void accumulateCloud(const typename Types_<Scalar>::Point3Matrix & cloud)
    {
      for (size_t i = 0; i < cloud.size(); ++i)
        accumulatePoint(cloud[i]);
    }

    virtual void accumulateCloud(const typename Types_<Scalar>::Point3Matrix & cloud,
                                 const std::vector<int> & indices)
    {
      for (size_t i = 0; i < indices.size(); ++i)
        accumulatePoint(cloud[indices[i]]);
    }

    virtual void accumulatePoint(const typename Types_<Scalar>::Point3 & point)
    {
      size_t x_index, y_index;
      Base::matrix_->getIndex(UndistortionMatrix::toSphericalCoordinates(point), x_index, y_index);
      Base::accumulation_bins_(x_index, y_index) += point;
    }

    virtual void addPoint(const typename Types_<Scalar>::Point3 & point,
                          const typename Types_<Scalar>::Plane & plane);

    virtual void undistort(typename Types_<Scalar>::Point3 & point) const
    {
      Base::matrix_->undistort(point);
    }

    virtual void undistort(typename Types_<Scalar>::Point3Matrix & cloud) const
    {
      Base::matrix_->undistort(cloud);
    }

    virtual void addAccumulatedPoints(const typename Types_<Scalar>::Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    virtual void update()
    {
      Base::update();
    }

    virtual typename UndistortionMatrix::Interface::Ptr clone() const
    {
      PolynomialUndistortionMatrixFitEigen::Ptr clone = boost::make_shared<PolynomialUndistortionMatrixFitEigen>(*this);
      clone->setMatrix(boost::shared_static_cast<UndistortionMatrix>(Base::matrix_->clone()));
      return clone;
    }

  };

template <typename Polynomial_, typename PCLPoint_>
  class PolynomialUndistortionMatrixFitPCL : public PolynomialUndistortionMatrixFit<Polynomial_,
                                               PolynomialUndistortionMatrixPCL<Polynomial_, PCLPoint_> >,
                                             public DepthUndistortionModelFitPCL<typename Traits<Polynomial_>::Scalar,
                                               PCLPoint_>
  {
  public:

    typedef boost::shared_ptr<PolynomialUndistortionMatrixFitPCL> Ptr;
    typedef boost::shared_ptr<const PolynomialUndistortionMatrixFitPCL> ConstPtr;

    typedef typename Traits<Polynomial_>::Scalar Scalar;
    typedef PolynomialUndistortionMatrixPCL<Polynomial_, PCLPoint_> UndistortionMatrix;
    typedef PolynomialUndistortionMatrixFit<Polynomial_, UndistortionMatrix> Base;

    PolynomialUndistortionMatrixFitPCL(size_t x_size,
                                       size_t y_size,
                                       Scalar x_fov,
                                       Scalar y_fov)
      : Base(x_size, y_size, x_fov, y_fov)
    {
      // Do nothing
    }

    PolynomialUndistortionMatrixFitPCL(const typename UndistortionMatrix::Ptr & matrix)
      : Base(matrix)
    {
      // Do nothing
    }

    virtual ~PolynomialUndistortionMatrixFitPCL()
    {
      // Do nothing
    }

    virtual void accumulateCloud(const pcl::PointCloud<PCLPoint_> & cloud)
    {
      for (size_t i = 0; i < cloud.points.size(); ++i)
        accumulatePoint(cloud.points[i]);
    }

    virtual void accumulateCloud(const pcl::PointCloud<PCLPoint_> & cloud,
                                 const std::vector<int> & indices)
    {
      for (size_t i = 0; i < indices.size(); ++i)
        accumulatePoint(cloud.points[indices[i]]);
    }

    virtual void accumulatePoint(const PCLPoint_ & point)
    {
      if (not pcl::isFinite(point))
        return;

      size_t x_index, y_index;
      Base::matrix_->getIndex(UndistortionMatrix::toSphericalCoordinates(point), x_index, y_index);
      Base::accumulation_bins_.at(x_index, y_index) += typename Types_<Scalar>::Point3(point.x, point.y, point.z);
    }

    virtual void addPoint(const PCLPoint_ & point,
                          const typename Types_<Scalar>::Plane & plane);

    virtual void undistort(PCLPoint_ & point) const
    {
      Base::matrix_->undistort(point);
    }

    virtual void undistort(pcl::PointCloud<PCLPoint_> & cloud) const
    {
      Base::matrix_->undistort(cloud);
    }

    virtual void addAccumulatedPoints(const typename Types_<Scalar>::Plane & plane)
    {
      Base::addAccumulatedPoints(plane);
    }

    virtual void update()
    {
      Base::update();
    }

    virtual typename UndistortionMatrix::Interface::Ptr clone() const
    {
      PolynomialUndistortionMatrixFitPCL::Ptr clone = boost::make_shared<PolynomialUndistortionMatrixFitPCL>(*this);
      clone->setMatrix(boost::shared_static_cast<UndistortionMatrix>(Base::matrix_->clone()));
      return clone;
    }

  };

} /* namespace calibration */

#include <impl/kinect/depth/polynomial_undistortion_matrix_fit.hpp>

#endif /* KINECT_DEPTH_POLYNOMIAL_UNDISTORTION_MATRIX_FIT_H_ */
