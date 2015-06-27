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

#include <Eigen/SVD>
#include <calibration_algorithms/plane_to_plane_calibration.h>

namespace unipd
{
namespace calib
{

Transform3
PlaneToPlaneCalibration::estimateTransform (const std::vector<std::pair<Plane3, Plane3>> & plane_pair_vector)
{
  const Size1 SIZE = plane_pair_vector.size();
  assert(SIZE >= 10);

  Eigen::Matrix<Scalar, Eigen::Dynamic, 3> normals_1(SIZE, 3);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 3> normals_2(SIZE, 3);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> distances_1(SIZE);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> distances_2(SIZE);

  for (int i = 0; i < SIZE; ++i)
  {
    const Plane3 & plane_1 = plane_pair_vector[i].first;
    const Plane3 & plane_2 = plane_pair_vector[i].second;

    if (plane_1.offset() > 0)
    {
      normals_1.row(i) = -plane_1.normal();
      distances_1(i) = plane_1.offset();
    }
    else
    {
      normals_1.row(i) = plane_1.normal();
      distances_1(i) = -plane_1.offset();
    }

    if (plane_2.offset() > 0)
    {
      normals_2.row(i) = -plane_2.normal();
      distances_2(i) = plane_2.offset();
    }
    else
    {
      normals_2.row(i) = plane_2.normal();
      distances_2(i) = -plane_2.offset();
    }
  }

  Transform3 transform;

  Eigen::Matrix<Scalar, 3, 3> USV = normals_2.transpose() * normals_1;
  Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3> > svd;
  svd.compute(USV, Eigen::ComputeFullU | Eigen::ComputeFullV);
  transform.linear() = svd.matrixV() * svd.matrixU().transpose();

//  transform.translation() = (normals_1.transpose() * normals_1).inverse() * normals_1.transpose() * (distances_1 - distances_2);

  // Use QR decomposition
  Eigen::ColPivHouseholderQR<Eigen::Matrix<Scalar, Eigen::Dynamic, 3> > qr(normals_1.rows(), 3);
  qr.compute(normals_1);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 3> R = qr.matrixR().template triangularView<Eigen::Upper>();
  transform.translation() = (qr.colsPermutation() * R.transpose() * R * qr.colsPermutation().transpose()).inverse() * normals_1.transpose() * (distances_1 - distances_2);

  return transform;
}

} // namespace calib
} // namespace unipd
