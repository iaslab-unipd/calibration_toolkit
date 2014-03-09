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

#include <Eigen/SVD>
#include <calibration_common/algorithms/plane_to_plane_calibration.h>

namespace calibration
{

Transform PlaneToPlaneCalibration::estimateTransform(const std::vector<PlanePair> & plane_pair_vector)
{
  const int size = plane_pair_vector.size();

  Eigen::Matrix<Scalar, 3, Eigen::Dynamic> normals_1(3, size);
  Eigen::Matrix<Scalar, 3, Eigen::Dynamic> normals_2(3, size);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> distances_1(size);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> distances_2(size);

  for (int i = 0; i < size; ++i)
  {
    const Plane & plane_1 = plane_pair_vector[i].plane_1_;
    const Plane & plane_2 = plane_pair_vector[i].plane_2_;

    if (plane_1.offset() > 0)
    {
      normals_1.col(i) = -plane_1.normal();
      distances_1(i) = plane_1.offset();
    }
    else
    {
      normals_1.col(i) = plane_1.normal();
      distances_1(i) = -plane_1.offset();
    }

    if (plane_2.offset() > 0)
    {
      normals_2.col(i) = -plane_2.normal();
      distances_2(i) = plane_2.offset();
    }
    else
    {
      normals_2.col(i) = plane_2.normal();
      distances_2(i) = -plane_2.offset();
    }
  }

  //  std::cout << normals_1 << std::endl;
  //  std::cout << distances_1.transpose() << std::endl;
  //  std::cout << normals_2 << std::endl;
  //  std::cout << distances_2.transpose() << std::endl;

  Eigen::Matrix3d USV = normals_2 * normals_1.transpose();
  Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3> > svd;
  svd.compute(USV, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Pose pose;
  pose.translation() = (normals_1 * normals_1.transpose()).inverse() * normals_1 * (distances_1 - distances_2);
  pose.linear() = svd.matrixV() * svd.matrixU().transpose();

  //  // Point-Plane Constraints
  //
  //  // Initial system (Eq. 10)
  //
  //  Eigen::MatrixXd system(size * 3, 13);
  //  for (int i = 0; i < size; ++i)
  //  {
  //    const double d = plane_pair_vector[i].plane_1_.offset();
  //    const Eigen::Vector3d n = plane_pair_vector[i].plane_1_.normal();
  //
  //    const Point3d x = -plane_pair_vector[i].plane_2_.normal() * plane_pair_vector[i].plane_2_.offset();
  //
  //    Eigen::Matrix3d X(Eigen::Matrix3d::Random());
  //    while (X.determinant() < 1e-5)
  //      X = Eigen::Matrix3d::Random();
  //
  //    const Eigen::Vector3d & n2 = plane_pair_vector[i].plane_2_.normal();
  ////    X.col(1)[2] = -n2.head<2>().dot(X.col(1).head<2>()) / n2[2];
  ////    X.col(2)[2] = -n2.head<2>().dot(X.col(2).head<2>()) / n2[2];
  //    X.col(1) -= n2 * n2.dot(X.col(1));
  //    X.col(2) -= n2 * n2.dot(X.col(2));
  //
  //    X.col(0) = x;
  //    X.col(1) += x;
  //    X.col(2) += x;
  //
  //    for (int j = 0; j < 3; ++j)
  //    {
  //      const Point3d & x = X.col(j);
  //
  //      system.row(i + size * j) << d + n[0] * x[0] + n[1] * x[1] + n[2] * x[2],  // q_0^2
  //                                  2 * n[2] * x[1] - 2 * n[1] * x[2],            // q_0 * q_1
  //                                  2 * n[0] * x[2] - 2 * n[2] * x[0],            // q_0 * q_2
  //                                  2 * n[1] * x[0] - 2 * n[0] * x[1],            // q_0 * q_3
  //                                  d + n[0] * x[0] - n[1] * x[1] - n[2] * x[2],  // q_1^2
  //                                  2 * n[0] * x[1] + 2 * n[1] * x[0],            // q_1 * q_2
  //                                  2 * n[0] * x[2] + 2 * n[2] * x[0],            // q_1 * q_3
  //                                  d - n[0] * x[0] + n[1] * x[1] - n[2] * x[2],  // q_2^2
  //                                  2 * n[1] * x[2] + 2 * n[2] * x[1],            // q_2 * q_3
  //                                  d - n[0] * x[0] - n[1] * x[1] + n[2] * x[2],  // q_3^2
  //                                  n[0], n[1], n[2]; // q'*q*t
  //    }
  //  }
  //
  //  //std::cout << system << std::endl;
  //
  //  // Gaussian reduction
  //  for (int k = 0; k < 3; ++k)
  //    for (int i = k + 1; i < size * 3; ++i)
  //      system.row(i) = system.row(i) - system.row(k) * system.row(i)[10 + k] / system.row(k)[10 + k];
  //
  //  //std::cout << system << std::endl;
  //
  //  // Quaternion q
  //  Eigen::Vector4d q;
  //
  //  // Transform to inhomogeneous (Eq. 13)
  //  bool P_is_ok(false);
  //  while (not P_is_ok)
  //  {
  //    Eigen::Matrix4d P(Eigen::Matrix4d::Random().normalized());
  //    while (P.determinant() < 1e-5)
  //      P = Eigen::Matrix4d::Random().normalized();
  //
  //    Eigen::MatrixXd reduced_system(size * 3 - 3, 10);
  //    for (int i = 3; i < size * 3; ++i)
  //    {
  //      const Eigen::VectorXd & row = system.row(i);
  //      Eigen::Matrix4d Mi_tilde;
  //
  //      Mi_tilde << row[0]    , row[1] / 2, row[2] / 2, row[3] / 2,
  //                  row[1] / 2, row[4]    , row[5] / 2, row[6] / 2,
  //                  row[2] / 2, row[5] / 2, row[7]    , row[8] / 2,
  //                  row[3] / 2, row[6] / 2, row[8] / 2, row[9]    ;
  //
  //      Eigen::Matrix4d Mi_bar(P.transpose() * Mi_tilde * P);
  //
  //      reduced_system.row(i - 3) << Mi_bar(0, 0),
  //                                   Mi_bar(0, 1) + Mi_bar(1, 0),
  //                                   Mi_bar(0, 2) + Mi_bar(2, 0),
  //                                   Mi_bar(0, 3) + Mi_bar(3, 0),
  //                                   Mi_bar(1, 1),
  //                                   Mi_bar(1, 2) + Mi_bar(2, 1),
  //                                   Mi_bar(1, 3) + Mi_bar(3, 1),
  //                                   Mi_bar(2, 2),
  //                                   Mi_bar(2, 3) + Mi_bar(3, 2),
  //                                   Mi_bar(3, 3);
  //    }
  //
  //    // Solve  A m* = b
  //    Eigen::MatrixXd A = reduced_system.rightCols<9>();
  //
  //    Eigen::VectorXd b = - reduced_system.leftCols<1>();
  //    Eigen::VectorXd m_star = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
  //
  //    Eigen::Vector4d q_bar(1, m_star[0], m_star[1], m_star[2]);
  //
  //    Eigen::VectorXd err(6);
  //    err << q_bar[1] * q_bar[1],
  //           q_bar[1] * q_bar[2],
  //           q_bar[1] * q_bar[3],
  //           q_bar[2] * q_bar[2],
  //           q_bar[2] * q_bar[3],
  //           q_bar[3] * q_bar[3];
  //    err -= m_star.tail<6>();
  //
  //    if (err.norm() < 0.1) // P is ok?
  //      P_is_ok = true;
  //
  //    q = P * q_bar;
  //  }
  //
  //  // We want q.w > 0 (Why?)
  //  if (q[0] < 0)
  //    q = -q;
  //  Eigen::Quaterniond rotation(q[0], q[1], q[2], q[3]);
  //  rotation.normalize();
  //
  //  Eigen::VectorXd m(10);
  //  m << q[0] * q[0],
  //       q[0] * q[1],
  //       q[0] * q[2],
  //       q[0] * q[3],
  //       q[1] * q[1],
  //       q[1] * q[2],
  //       q[1] * q[3],
  //       q[2] * q[2],
  //       q[2] * q[3],
  //       q[3] * q[3];
  //
  //  // Solve A (q^T q t) = b
  //
  //  Eigen::Matrix3d A = system.topRightCorner<3, 3>();
  //  Eigen::Vector3d b = - system.topLeftCorner<3, 10>() * m;
  //
  //  std::cout << A << " " << b.transpose() << std::endl;
  //  Eigen::Translation3d translation(A.colPivHouseholderQr().solve(b) / q.squaredNorm());
  //
  //  Eigen::Quaterniond tmp(pose.linear());
  //  Eigen::Translation3d tmp2(pose.translation());
  //  std::cout << "Prev: " << tmp.normalized().coeffs().transpose() << " " << tmp2.vector().transpose() << std::endl;
  //  std::cout << "New : " << rotation.coeffs().transpose() << " " << translation.vector().transpose() << std::endl;
  //
  //  pose = translation * rotation;

  return pose;
}

} /* namespace calibration */
