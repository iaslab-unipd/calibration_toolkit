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

#ifndef IMPL_KINECT_DEPTH_POLYNOMIAL_MATRIX_FIT_HPP_
#define IMPL_KINECT_DEPTH_POLYNOMIAL_MATRIX_FIT_HPP_

#include <kinect/depth/polynomial_matrix_fit.h>
#include <ceres/ceres.h>

namespace calibration
{

template <typename ModelImpl_>
  void PolynomialUndistortionMatrixFit_<ModelImpl_>::addAccumulatedPoints(const Plane & plane)
  {
    for (size_t i = 0; i < accumulation_bins_.size(); ++i)
    {
      if (accumulation_bins_[i].isEmpty())
        continue;

      typename Types<Scalar>::Point3 point = accumulation_bins_[i].average();
      typename Types<Scalar>::Line line(point, Types<Scalar>::Vector3::UnitZ());

      distorsion_bins_[i].push_back(std::make_pair(point.z(), line.intersectionPoint(plane).z()));
      accumulation_bins_[i].reset();
    }
  }

template <typename ModelImpl_>
  void PolynomialUndistortionMatrixFit_<ModelImpl_>::update()
  {
    assert(model_impl_);

#pragma omp parallel for
    for (size_t y_index = 0; y_index < distorsion_bins_.ySize(); ++y_index)
    {
      for (size_t x_index = 0; x_index < distorsion_bins_.xSize(); ++x_index)
      {
        const PointDistorsionBin & distorsion_bin = distorsion_bins_(x_index, y_index);
        const size_t bin_size = distorsion_bin.size();

        if (bin_size < 5 * Degree)
          continue;

        ceres::Problem problem;
        for (size_t i = 0; i < bin_size; ++i)
        {
          PolynomialResidual<Poly> * residual = new PolynomialResidual<Poly>(distorsion_bin[i].first,
                                                                             distorsion_bin[i].second);
          problem.AddResidualBlock(new ceres::AutoDiffCostFunction<PolynomialResidual<Poly>, 1, Size>(residual),
                                   NULL,
                                   model_impl_->model()->polynomial(x_index, y_index).data());
        }

        ceres::Solver::Options options;
        options.max_num_iterations = 25;
        options.linear_solver_type = ceres::DENSE_QR;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

      }
    }
  }

template <typename Polynomial_, typename ScalarT_>
  void PolynomialUndistortionMatrixFitEigen<Polynomial_, ScalarT_>::addPoint(const Point & point,
                                                                             const Plane & plane)
  {
    assert(Base::model_impl_);
    typename Types<Scalar>::Line line(point, Types<Scalar>::Vector3::UnitZ());

    size_t x_index, y_index;
    Base::model_impl_->model()->getIndex(UndistortionModel::toSphericalCoordinates(point), x_index, y_index);
    Base::distorsion_bins_(x_index, y_index).push_back(std::make_pair(point.z(), line.intersectionPoint(plane).z()));
  }

template <typename Polynomial_, typename PCLPoint_, typename ScalarT_>
  void PolynomialUndistortionMatrixFitPCL<Polynomial_, PCLPoint_, ScalarT_>::addPoint(const Point & point,
                                                                                      const Plane & plane)
  {
    assert(Base::model_impl_);
    if (not pcl::isFinite(point))
      return;

    typename Types<Scalar>::Point3 eigen_point(point.x, point.y, point.z);
    typename Types<Scalar>::Line line(eigen_point, Types<Scalar>::Vector3::UnitZ());

    size_t x_index, y_index;
    Base::model_impl_->model()->getIndex(UndistortionModel::project(point), x_index, y_index);
    Base::distorsion_bins_(x_index, y_index).push_back(std::make_pair(eigen_point.z(),
                                                                      line.intersectionPoint(plane).z()));
  }

} /* namespace calibration */
#endif /* IMPL_KINECT_DEPTH_POLYNOMIAL_MATRIX_FIT_HPP_ */
