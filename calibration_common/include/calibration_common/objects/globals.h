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

#ifndef CALIBRATION_COMMON_OBJECTS_GLOBALS_H_
#define CALIBRATION_COMMON_OBJECTS_GLOBALS_H_

#include <calibration_common/base/point_matrix.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/**
 * @namespace calibration
 */
namespace calibration
{

/**
 * @addtogroup Base
 * @{
 */

/**
 * @brief Base types.
 * @param ScalarT_ Scalar type (@c float or @c double).
 */
template <typename ScalarT_>
  struct Types
  {
    typedef ScalarT_ Scalar;                                       ///< Scalar type.

    typedef Eigen::Matrix<Scalar, 2, 1> Vector2;                   ///< 2D Eigen Vector.
    typedef Vector2 Point2;                                        ///< 2D Eigen Point.

    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;                   ///< 3D Eigen Vector.
    typedef Vector3 Point3;                                        ///< 3D Eigen Point.

    typedef Eigen::Matrix<Scalar, 4, 1> Vector4;                   ///< 4D Eigen Vector.
    typedef Vector4 Point4;                                        ///< 4D Eigen Point.

    typedef Eigen::Hyperplane<Scalar, 3> Plane;                    ///< 3D Eigen Plane.
    typedef Eigen::ParametrizedLine<Scalar, 3> Line;               ///< 3D Eigen Line.

    typedef Eigen::Transform<Scalar, 3, Eigen::Affine> Transform;  ///< 3D Eigen Affine Transform.
    typedef Transform Pose;                                        ///< 3D Eigen Pose.

    typedef Eigen::Quaternion<Scalar> Quaternion;                  ///< Eigen Quaternion.
    typedef Eigen::Translation<Scalar, 2> Translation2;            ///< 2D Eigen Translation.
    typedef Eigen::Translation<Scalar, 3> Translation3;            ///< 3D Eigen Translation.
    typedef Eigen::AngleAxis<Scalar> AngleAxis;                    ///< Eigen AngleAxis angle representation.
    typedef Eigen::Rotation2D<Scalar> Rotation2;
    typedef Eigen::Transform<Scalar, 2, Eigen::Affine> Transform2;

    typedef PointMatrix<Scalar, 2> Cloud2;                         ///< 2D Eigen PointCloud.
    typedef PointMatrix<Scalar, 3> Cloud3;                         ///< 3D Eigen PointCloud.
  };

typedef Types<double>::Scalar Scalar;               ///< Scalar type (@c float or @c double).

typedef Types<Scalar>::Vector2 Vector2;             ///< 2D Eigen Vector.
typedef Types<Scalar>::Point2 Point2;               ///< 2D Eigen Point.

typedef Types<Scalar>::Vector3 Vector3;             ///< 3D Eigen Vector.
typedef Types<Scalar>::Point3 Point3;               ///< 3D Eigen Point.

typedef Types<Scalar>::Vector4 Vector4;             ///< 4D Eigen Vector.
typedef Types<Scalar>::Point4 Point4;               ///< 4D Eigen Point.

typedef Types<Scalar>::Plane Plane;                 ///< 3D Eigen Plane.
typedef Types<Scalar>::Line Line;                   ///< 3D Eigen Line.

typedef Types<Scalar>::Transform Transform;         ///< 3D Eigen Affine Transform.
typedef Types<Scalar>::Pose Pose;                   ///< 3D Eigen Pose.

typedef Types<Scalar>::Quaternion Quaternion;       ///< Eigen Quaternion.
typedef Types<Scalar>::Translation2 Translation2;   ///< 2D Eigen Translation.
typedef Types<Scalar>::Translation3 Translation3;   ///< 3D Eigen Translation.
typedef Types<Scalar>::AngleAxis AngleAxis;         ///< Eigen AngleAxis angle representation.
typedef Types<Scalar>::Rotation2 Rotation2;
typedef Types<Scalar>::Transform2 Transform2;

typedef Types<Scalar>::Cloud2 Cloud2;               ///< 2D Eigen PointCloud.
typedef Types<Scalar>::Cloud3 Cloud3;               ///< 3D Eigen PointCloud.

// PCL

typedef pcl::PointXYZ PCLPoint3;                    ///< 3D pcl Point.
typedef pcl::PointCloud<PCLPoint3> PCLCloud3;       ///< 3D pcl PointCloud.

// Misc

typedef std::vector<int> Indices;
typedef boost::shared_ptr<Indices> IndicesPtr;
typedef boost::shared_ptr<const Indices> IndicesConstPtr;

/* ******************************************************************* */

const Plane PLANE_XY = Plane(Vector3::UnitZ(), 0);  ///< The $x-y$ plane.

// TODO move to another file?

/**
 * @brief Some utility methods.
 */
struct Util
{
  /**
   * @brief Estimates a possible affine transformation between two planes.
   * @param [in] from The plane for which to calculate the transformation.
   * @param [in] to The target plane.
   * @return The affine transformation that transform a point in @c from to a point in @c to.
   */
  inline static Transform plane3dTransform(const Plane & from,
                                           const Plane & to)
  {
    Quaternion rotation;
    rotation.setFromTwoVectors(from.normal(), to.normal());
    Translation3 translation(-to.normal() * to.offset());
    return translation * rotation;
  }
};

/* ******************************************************************* */

/**
 * @brief base class to define a constraint.
 * @param ObjectT_ The type for which to declare a constraint.
 */
template <typename ObjectT_>
  struct Constraint
  {
    typedef boost::shared_ptr<Constraint> Ptr;
    typedef boost::shared_ptr<const Constraint> ConstPtr;

    virtual ~Constraint()
    {
      // Do nothing
    }

    /**
     * @brief Test if the @c object satisfies the constraint.
     * @param object The object.
     * @return @c true if the constraint is satisfied, @c false otherwise.
     */
    virtual bool isValid(const ObjectT_ & object) const = 0;
  };

/**
 * @brief Dummy implementation. The constraint is always satisfied.
 * @param ObjectT_ The type for which to declare a constraint.
 */
template <typename ObjectT_>
  struct NoConstraint : public Constraint<ObjectT_>
  {
    typedef boost::shared_ptr<NoConstraint> Ptr;
    typedef boost::shared_ptr<const NoConstraint> ConstPtr;

    /**
     * @brief Test if the @c object satisfies the constraint.
     * @param object The object.
     * @return @c true
     */
    bool isValid(const ObjectT_ & object) const
    {
      return true;
    }
  };

/**
 * @}
 */

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_OBJECTS_GLOBALS_H_ */
