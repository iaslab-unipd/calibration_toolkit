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

#ifndef UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_GEOMETRY_H_
#define UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_GEOMETRY_H_

#include <Eigen/Geometry>
#include <calibration_common/base/definitions.h>

namespace unipd
{
namespace calib
{

template <typename ScalarT_>
  using Vector1_ = Eigen::Matrix<ScalarT_, 1, 1>;                      ///< 1D Eigen Vector.
template <typename ScalarT_>
  using Point1_ = Vector1_<ScalarT_>;                                  ///< 1D Eigen Point.

template <typename ScalarT_>
  using Vector2_ = Eigen::Matrix<ScalarT_, 2, 1>;                      ///< 2D Eigen Vector.
template <typename ScalarT_>
  using Point2_ = Vector2_<ScalarT_>;                                  ///< 2D Eigen Point.

template <typename ScalarT_>
  using Translation2_ = Eigen::Translation<ScalarT_, 2>;               ///< 2D Eigen Translation.

template <typename ScalarT_>
  using Line2_ = Eigen::Hyperplane<ScalarT_, 2>;                       ///< 2D Eigen Line.

template <typename ScalarT_>
  using Vector3_ =  Eigen::Matrix<ScalarT_, 3, 1>;                     ///< 3D Eigen Vector.
template <typename ScalarT_>
  using Point3_ = Vector3_<ScalarT_>;                                  ///< 3D Eigen Point.

template <typename ScalarT_>
  using Translation3_ = Eigen::Translation<ScalarT_, 3>;               ///< 3D Eigen Translation.

template <typename ScalarT_>
  using Transform3_ = Eigen::Transform<ScalarT_, 3, Eigen::Affine>;    ///< 3D Eigen Affine Transform.
template <typename ScalarT_>
  using Pose3_ = Transform3_<ScalarT_>;                                ///< 3D Eigen Pose.

template <typename ScalarT_>
  using Quaternion_ = Eigen::Quaternion<ScalarT_>;                     ///< Eigen Quaternion.
template <typename ScalarT_>
  using AngleAxis_ = Eigen::AngleAxis<ScalarT_>;                       ///< Eigen AngleAxis angle representation.

template <typename ScalarT_>
  using Line3_ = Eigen::ParametrizedLine<ScalarT_, 3>;                 ///< 3D Eigen Line.
template <typename ScalarT_>
  using Plane3_ = Eigen::Hyperplane<ScalarT_, 3>;                      ///< 3D Eigen Plane.

using Vector1 = Vector1_<Scalar>;
using Point1 = Point1_<Scalar>;

using Vector2 = Vector2_<Scalar>;
using Point2 = Point2_<Scalar>;

using Translation2 = Translation2_<Scalar>;

using Line2 = Line2_<Scalar>;

using Vector3 = Vector3_<Scalar>;
using Point3 = Point3_<Scalar>;

using Transform3 = Transform3_<Scalar>;
using Pose3 = Pose3_<Scalar>;

using Quaternion = Quaternion_<Scalar>;
using AngleAxis = AngleAxis_<Scalar>;
using Translation3 = Translation3_<Scalar>;

using Plane3 = Plane3_<Scalar>;
using Line3 = Line3_<Scalar>;


//constexpr Plane3 PlaneXY = Plane3(Vector3::UnitZ(), 0);                 ///< The $x-y$ plane.
//constexpr Plane3 PlaneXZ = Plane3(Vector3::UnitY(), 0);                 ///< The $x-z$ plane.
//constexpr Plane3 PlaneYZ = Plane3(Vector3::UnitX(), 0);                 ///< The $y-z$ plane.

//Transform3 computeTransform(const Plane3 & from, const Plane3 & to)
//{
//  Quaternion rotation;
//  rotation.setFromTwoVectors(from.normal(), to.normal());
//  Translation3 translation(-to.normal() * to.offset());
//  return translation * rotation;
//}

}// namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATION_COMMON_BASE_GEOMETRY_H_
