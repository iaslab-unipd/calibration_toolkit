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

#ifndef IMPL_CALIBRATION_COMMON_CERES_PLANE_FIT_HPP_
#define IMPL_CALIBRATION_COMMON_CERES_PLANE_FIT_HPP_

#include <calibration_common/ceres/plane_fit.h>
#include <Eigen/Eigenvalues>

#include <ceres/ceres.h>

namespace calibration
{

template <typename ScalarT_>
  typename Types<ScalarT_>::Plane PlaneFit<ScalarT_>::fit(const typename Types<ScalarT_>::Cloud3 & points)
  {
    assert(points.elements() > 1);
//    typename Types<ScalarT_>::Point3 centroid = points.container().rowwise().mean();
//    typename Types<ScalarT_>::Cloud3::Container diff = points.container().colwise() - centroid;
//    Eigen::Matrix<ScalarT_, 3, 3> covariance_matrix = diff * diff.transpose() / ScalarT_(points.elements() - 1);

    typename Types<ScalarT_>::Point3 centroid = Types<ScalarT_>::Point3::Zero();
    typename Types<ScalarT_>::Cloud3::Container diff(3, points.elements());
    Size1 count = 0;
    for (Size1 i = 0; i < points.elements(); ++i)
    {
      if (points[i].allFinite())
      {
        centroid += points[i];
        diff.col(count++) = points[i];
      }
    }
    centroid /= count;
    diff.conservativeResize(3, count);
    diff.colwise() -= centroid;

    Eigen::Matrix<ScalarT_, 3, 3> covariance_matrix = diff * diff.transpose() / ScalarT_(count - 1);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<ScalarT_, 3, 3> > solver(covariance_matrix, Eigen::ComputeEigenvectors);

    typename Types<ScalarT_>::Point3 eigen_vector = solver.eigenvectors().col(0);
    return typename Types<ScalarT_>::Plane(eigen_vector, -eigen_vector.dot(centroid));
  }

template <typename ScalarT_>
  typename Types<ScalarT_>::Plane PlaneFit<ScalarT_>::fit(const PCLCloud3 & cloud)
  {
    assert(cloud.size() > 1);

    Types<float>::Point3 centroid = Types<float>::Point3::Zero();
    Types<float>::Cloud3::Container diff(3, cloud.size());

    Eigen::Map<const Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> > map = cloud.getMatrixXfMap(3, 4, 0);

    Size1 count = 0;
    for (Size1 i = 0; i < cloud.size(); ++i)
    {
      if (pcl::isFinite(cloud.points[i]))
      {
        diff.col(count) = map.row(i).transpose();
        centroid += diff.col(count++);
      }
    }
    centroid /= count;
    diff.conservativeResize(3, count);
    diff.colwise() -= centroid;

    Eigen::Matrix<float, 3, 3> covariance_matrix = diff * diff.transpose() / ScalarT_(count - 1);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance_matrix, Eigen::ComputeEigenvectors);

    Types<float>::Point3 eigen_vector = solver.eigenvectors().col(0);
    return typename Types<ScalarT_>::Plane(eigen_vector.template cast<ScalarT_>(), -eigen_vector.dot(centroid));
  }

template <typename ScalarT_>
  typename Types<ScalarT_>::Plane PlaneFit<ScalarT_>::robustFit(const typename Types<ScalarT_>::Plane & initial_plane,
                                                                const typename Types<ScalarT_>::Cloud3 & points,
                                                                ScalarT_ scale)
  {
    ceres::Problem problem;
    typename Types<ScalarT_>::Plane plane(initial_plane);

    for (Size1 i = 0; i < points.elements(); ++i)
    {
      ceres::CostFunction * cost_function =
        new ceres::AutoDiffCostFunction<PlaneResidual<ScalarT_>, 1, 4>(new PlaneResidual<ScalarT_>(points[i]));
      problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(scale), plane.coeffs().data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
//    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    return plane;
  }

} /* namespace calibration */
#endif /* IMPL_CALIBRATION_COMMON_CERES_PLANE_FIT_HPP_ */
