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

//template <typename Scalar_, typename Roots_>
//  inline void computeRoots2(const Scalar_ & b,
//                            const Scalar_ & c,
//                            Roots_ & roots)
//  {
//    roots(0) = Scalar_(0);
//    Scalar_ d = Scalar_(b * b - Scalar_(4.0) * c);
//    if (d < Scalar_(0)) // no real roots!!!! THIS SHOULD NOT HAPPEN!
//      d = Scalar_(0);
//
//    Scalar_ sd = ceres::sqrt(d);
//
//    roots(2) = Scalar_(0.5) * (b + sd);
//    roots(1) = Scalar_(0.5) * (b - sd);
//  }
//
//template <typename Matrix_, typename Roots_>
//  inline void computeRoots(const Matrix_ & m,
//                           Roots_ & roots)
//  {
//    typedef typename Matrix_::Scalar Scalar;
//
//    // The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0.  The
//    // eigenvalues are the roots to this equation, all guaranteed to be
//    // real-valued, because the matrix is symmetric.
//    Scalar c0 = m(0, 0) * m(1, 1) * m(2, 2) + Scalar(2) * m(0, 1) * m(0, 2) * m(1, 2) - m(0, 0) * m(1, 2) * m(1, 2)
//                - m(1, 1) * m(0, 2) * m(0, 2) - m(2, 2) * m(0, 1) * m(0, 1);
//    Scalar c1 = m(0, 0) * m(1, 1) - m(0, 1) * m(0, 1) + m(0, 0) * m(2, 2) - m(0, 2) * m(0, 2) + m(1, 1) * m(2, 2)
//      - m(1, 2) * m(1, 2);
//    Scalar c2 = m(0, 0) + m(1, 1) + m(2, 2);
//
//    if (ceres::abs(c0) < Eigen::NumTraits<Scalar>::epsilon())
//      calibration::computeRoots2(c2, c1, roots);
//    else
//    {
//      const Scalar s_inv3 = Scalar(1.0 / 3.0);
//      const Scalar s_sqrt3 = ceres::sqrt(Scalar(3.0));
//      // Construct the parameters used in classifying the roots of the equation
//      // and in solving the equation for the roots in closed form.
//      Scalar c2_over_3 = c2 * s_inv3;
//      Scalar a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
//      if (a_over_3 > Scalar(0))
//        a_over_3 = Scalar(0);
//
//      Scalar half_b = Scalar(0.5) * (c0 + c2_over_3 * (Scalar(2) * c2_over_3 * c2_over_3 - c1));
//
//      Scalar q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
//      if (q > Scalar(0))
//        q = Scalar(0);
//
//      // Compute the eigenvalues by solving for the roots of the polynomial.
//      Scalar rho = ceres::sqrt(-a_over_3);
//      Scalar theta = ceres::atan2(ceres::sqrt(-q), half_b) * s_inv3;
//      Scalar cos_theta = ceres::cos(theta);
//      Scalar sin_theta = ceres::sin(theta);
//      roots(0) = c2_over_3 + Scalar(2) * rho * cos_theta;
//      roots(1) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
//      roots(2) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);
//
//      // Sort in increasing order.
//      if (roots(0) >= roots(1))
//        ceres::swap(roots(0), roots(1));
//      if (roots(1) >= roots(2))
//      {
//        ceres::swap(roots(1), roots(2));
//        if (roots(0) >= roots(1))
//          ceres::swap(roots(0), roots(1));
//      }
//
//      if (roots(0) <= Scalar(0)) // eigenval for symetric positive semi-definite matrix can not be negative! Set it to 0
//        calibration::computeRoots2(c2, c1, roots);
//    }
//  }
//
//template <typename Matrix_, typename Vector_>
//  inline void eigen33(const Matrix_ & mat,
//                      typename Matrix_::Scalar & eigenvalue,
//                      Vector_ & eigenvector)
//  {
//    typedef typename Matrix_::Scalar Scalar;
//    // Scale the matrix so its entries are in [-1,1].  The scaling is applied
//    // only when at least one matrix entry has magnitude larger than 1.
//
//    Scalar scale = mat.cwiseAbs().maxCoeff();
//    if (scale <= std::numeric_limits<Scalar>::min())
//      scale = Scalar(1.0);
//
//    Matrix_ scaledMat = mat / scale;
//
//    Vector_ eigenvalues;
//    calibration::computeRoots(scaledMat, eigenvalues);
//
//    eigenvalue = eigenvalues(0) * scale;
//
//    scaledMat.diagonal().array() -= eigenvalues(0);
//
//    Vector_ vec1 = scaledMat.row(0).cross(scaledMat.row(1));
//    Vector_ vec2 = scaledMat.row(0).cross(scaledMat.row(2));
//    Vector_ vec3 = scaledMat.row(1).cross(scaledMat.row(2));
//
//    Scalar len1 = vec1.squaredNorm();
//    Scalar len2 = vec2.squaredNorm();
//    Scalar len3 = vec3.squaredNorm();
//
//    if (len1 >= len2 && len1 >= len3)
//      eigenvector = vec1 / ceres::sqrt(len1);
//    else if (len2 >= len1 && len2 >= len3)
//      eigenvector = vec2 / ceres::sqrt(len2);
//    else
//      eigenvector = vec3 / ceres::sqrt(len3);
//  }
//
//template <typename Scalar_>
//  void computeMeanAndCovarianceMatrix(const typename Types_<Scalar_>::Point3Matrix & points,
//                                      Eigen::Matrix<Scalar_, 3, 3> & covariance_matrix,
//                                      typename Types_<Scalar_>::Point3 & centroid)
//  {
//    Eigen::Matrix<Scalar_, 9, 1> accu(Eigen::Matrix<Scalar_, 9, 1>::Zero());
//    size_t point_count = points.size();
//
//    for (size_t i = 0; i < point_count; ++i)
//    {
//      accu.template head<3>() += points[i] * points[i][0];
//      accu[3] += points[i][1] * points[i][1];
//      accu[4] += points[i][1] * points[i][2];
//      accu[5] += points[i][2] * points[i][2];
//      accu.template tail<3>() += points[i];
//    }
//
//    accu /= static_cast<Scalar_>(point_count);
//    if (point_count != 0)
//    {
//      centroid = accu.template tail<3>();
//      covariance_matrix.coeffRef(0) = accu[0] - accu[6] * accu[6];
//      covariance_matrix.coeffRef(1) = accu[1] - accu[6] * accu[7];
//      covariance_matrix.coeffRef(2) = accu[2] - accu[6] * accu[8];
//      covariance_matrix.coeffRef(4) = accu[3] - accu[7] * accu[7];
//      covariance_matrix.coeffRef(5) = accu[4] - accu[7] * accu[8];
//      covariance_matrix.coeffRef(8) = accu[5] - accu[8] * accu[8];
//      covariance_matrix.coeffRef(3) = covariance_matrix.coeff(1);
//      covariance_matrix.coeffRef(6) = covariance_matrix.coeff(2);
//      covariance_matrix.coeffRef(7) = covariance_matrix.coeff(5);
//    }
//
//  }

template <typename Scalar_>
  typename Types<Scalar_>::Plane PlaneFit<Scalar_>::fit(const typename Types<Scalar_>::Cloud3 & points)
  {

    typename Types<Scalar_>::Point3 centroid = points.container().rowwise().mean();
    typename Types<Scalar_>::Cloud3::Container diff = points.container().colwise() - centroid;
    Eigen::Matrix<Scalar_, 3, 3> covariance_matrix = diff * diff.transpose() / Scalar_(points.size() - 1);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar_, 3, 3> > solver(covariance_matrix, Eigen::ComputeEigenvectors);
    typename Types<Scalar_>::Point3 eigen_vector = solver.eigenvectors().col(0);
    return typename Types<Scalar_>::Plane(eigen_vector, -eigen_vector.dot(centroid));

  }

template <typename Scalar_>
  typename Types<Scalar_>::Plane PlaneFit<Scalar_>::robustFit(const typename Types<Scalar_>::Plane & initial_plane,
                                                              const typename Types<Scalar_>::Cloud3 & points,
                                                              Scalar_ scale)
  {
    ceres::Problem problem;
    typename Types<Scalar_>::Plane plane(initial_plane);

    for (int i = 0; i < points.size(); ++i)
    {
      ceres::CostFunction * cost_function =
        new ceres::AutoDiffCostFunction<PlaneResidual<Scalar_>, 1, 4>(new PlaneResidual<Scalar_>(points[i]));
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
