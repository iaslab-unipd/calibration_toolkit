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

#ifndef CALIBRATION_COMMON_CERES_PLANE_FIT_H_
#define CALIBRATION_COMMON_CERES_PLANE_FIT_H_

#include <calibration_common/objects/globals.h>
#include <ceres/ceres.h>

namespace calibration
{

//template <typename Matrix_, typename RootsT_>
//void computeRoots(const Matrix_ & m,
//                  RootsT_ & roots);

//template <typename ScalarT_, typename RootsT_>
//void computeRoots2(const ScalarT_ & b,
//                   const ScalarT_ & c,
//                   RootsT_ & roots);

//template <typename Matrix_, typename VectorT_>
//void eigen33(const Matrix_ & mat,
//             typename Matrix_::Scalar & eigenvalue,
//             VectorT_ & eigenvector);

//template <typename ScalarT_>
//void computeMeanAndCovarianceMatrix(const typename Types<ScalarT_>::Point3Matrix & cloud,
//                                    Eigen::Matrix<ScalarT_, 3, 3> & covariance_matrix,
//                                    typename Types<ScalarT_>::Point3 & centroid);

template <typename ScalarT_>
  class PlaneResidual
  {
  public:

    PlaneResidual(const typename Types<ScalarT_>::Point3 & point)
      : point_(point.x(), point.y(), point.z(), ScalarT_(1))
    {
    }

    template <typename T>
      bool operator()(const T * const plane,
                      T * residual) const
      {
//    Eigen::Map<const typename Types<ScalarT_>::Vector4> plane_eigen(plane);
//    residual[0] = point_.dot(plane_eigen) / plane_eigen.template head<3>().norm();
        residual[0] = point_[0] * plane[0] + point_[1] * plane[1] + point_[2] * plane[2] + plane[3];
        residual[0] /= ceres::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
        return true;
      }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    const typename Types<ScalarT_>::Vector4 point_;

  };

template <typename ScalarT_>
  class PlaneFit
  {
    typedef Types<ScalarT_> T;

  public:

    void setCloud(const typename T::Cloud3::ConstPtr & cloud)
    {
      cloud_ = cloud;
    }

    typename T::Plane fit()
    {
      return fit(*cloud_);
    }

    typename T::Plane robustFit(ScalarT_ scale = ScalarT_(1.0))
    {
      return robustFit(*cloud_, scale);
    }

    typename T::Plane robustFit(const typename T::Plane & initial_plane,
                                ScalarT_ scale = ScalarT_(1.0))
    {
      return robustFit(initial_plane, *cloud_, scale);
    }

    static typename T::Plane robustFit(const typename T::Cloud3 & points,
                                       ScalarT_ scale = ScalarT_(1.0))
    {
      return robustFit(fit(points), points, scale);
    }

    static typename T::Plane fit(const typename T::Cloud3 & points);

    static typename T::Plane robustFit(const typename T::Plane & initial_plane,
                                       const typename T::Cloud3 & points,
                                       ScalarT_ scale = ScalarT_(1.0));

  private:

    typename T::Cloud3::ConstPtr cloud_;

  };

} /* namespace calibration */

#include <impl/calibration_common/ceres/plane_fit.hpp>

#endif /* CALIBRATION_COMMON_CERES_PLANE_FIT_H_ */
