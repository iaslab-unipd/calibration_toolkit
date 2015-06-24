/*
 *  Copyright (c) 2015-, Filippo Basso <bassofil@gmail.com>
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

#ifndef UNIPD_CALIBRATION_IMPL_CALIBRATION_ALGORITHMS_CERES_PLANE_FIT_HPP_
#define UNIPD_CALIBRATION_IMPL_CALIBRATION_ALGORITHMS_CERES_PLANE_FIT_HPP_

#include <calibration_algorithms/plane_fit.h>
#include <Eigen/Eigenvalues>

namespace unipd
{
namespace calib
{

template <typename ScalarT_>
  Plane3_<ScalarT_>
  plane_fit (const Cloud3_<ScalarT_> & points)
  {
    assert(points.elements() > 1);

    Point3_<ScalarT_> centroid = Point3_<ScalarT_>::Zero();
    typename Cloud3_<ScalarT_>::Container diff(3, points.elements());
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

    Point3_<ScalarT_> eigen_vector = solver.eigenvectors().col(0);
    return Plane3_<ScalarT_>(eigen_vector, -eigen_vector.dot(centroid));
  }

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_IMPL_CALIBRATION_ALGORITHMS_CERES_PLANE_FIT_HPP_
