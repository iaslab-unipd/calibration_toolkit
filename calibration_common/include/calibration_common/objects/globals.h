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

namespace calibration
{

template <typename Scalar>
  struct Types_
  {
    typedef Eigen::Matrix<Scalar, 2, 1> Vector2;
    typedef Vector2 Point2;

    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Vector3 Point3;

    typedef Eigen::Hyperplane<Scalar, 3> Plane;
    typedef Eigen::ParametrizedLine<Scalar, 3> Line;

    typedef Eigen::Transform<Scalar, 3, Eigen::Affine> Transform;
    typedef Transform Pose;

    typedef Eigen::Quaternion<Scalar> Quaternion;
    typedef Eigen::Translation<Scalar, 2> Translation2;
    typedef Eigen::Translation<Scalar, 3> Translation3;
    typedef Eigen::AngleAxis<Scalar> AngleAxis;

    typedef PointMatrix<Scalar, 2> Point2Matrix;
    typedef PointMatrix<Scalar, 3> Point3Matrix;
  };

namespace Types
{

typedef double Scalar;

typedef Eigen::Matrix<Scalar, 2, 1> Vector2;
typedef Vector2 Point2;

typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Vector3 Point3;

typedef Eigen::Hyperplane<Scalar, 3> Plane;
typedef Eigen::ParametrizedLine<Scalar, 3> Line;

typedef Eigen::Transform<Scalar, 3, Eigen::Affine> Transform;
typedef Transform Pose;

typedef Eigen::Quaternion<Scalar> Quaternion;
typedef Eigen::Translation<Scalar, 2> Translation2;
typedef Eigen::Translation<Scalar, 3> Translation3;
typedef Eigen::AngleAxis<Scalar> AngleAxis;

typedef PointMatrix<Scalar, 2> Point2Matrix;
typedef PointMatrix<Scalar, 3> Point3Matrix;

} /* namespace Types */

/* ******************************************************************* */

const Types::Plane PLANE_XY = Types::Plane(Types::Point3::UnitZ(), 0);

// TODO move to another file ?  |
//                             _|_
//                             \ /
//                              v

struct Util
{
  inline static Types::Transform plane3dTransform(const Types::Plane & from,
                                                  const Types::Plane & to)
  {
    Types::Quaternion rotation;
    rotation.setFromTwoVectors(from.normal(), to.normal());
    Types::Translation3 translation(-to.normal() * to.offset());
    return translation * rotation;
  }
};

/* ******************************************************************* */

template <typename ObjectT>
  struct Constraint
  {
    typedef boost::shared_ptr<Constraint> Ptr;
    typedef boost::shared_ptr<const Constraint> ConstPtr;

    virtual ~Constraint()
    {
      // Do nothing
    }
    virtual bool isValid(const ObjectT & object) const = 0;
  };

template <typename ObjectT>
  struct NoConstraint : public Constraint<ObjectT>
  {
    typedef boost::shared_ptr<NoConstraint> Ptr;
    typedef boost::shared_ptr<const NoConstraint> ConstPtr;

    bool isValid(const ObjectT & object) const
    {
      return true;
    }
  };

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_OBJECTS_GLOBALS_H_ */
