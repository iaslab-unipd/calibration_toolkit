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

namespace calibration
{

template <typename Matrix_, typename Roots_>
  void computeRoots(const Matrix_ & m,
                    Roots_ & roots);

template <typename Scalar_, typename Roots_>
  void computeRoots2(const Scalar_ & b,
                     const Scalar_ & c,
                     Roots_ & roots);

template <typename Matrix_, typename Vector_>
  void eigen33(const Matrix_ & mat,
               typename Matrix_::Scalar & eigenvalue,
               Vector_ & eigenvector);

template <typename Scalar_>
  void computeMeanAndCovarianceMatrix(const typename Types_<Scalar_>::Point3Matrix & cloud,
                                      Eigen::Matrix<Scalar_, 3, 3> & covariance_matrix,
                                      typename Types_<Scalar_>::Point3 & centroid);

template <typename Scalar_>
  class PlaneResidual
  {
  public:

    PlaneResidual(const typename Types_<Scalar_>::Point3 & point)
      : point_(point.x(), point.y(), point.z())
    {
    }

    template <typename T>
      bool operator()(const T * const plane,
                      T * residual) const
      {
        residual[0] = point_[0] * plane[0] + point_[1] * plane[1] + point_[2] * plane[2] + plane[3];
        residual[0] /= ceres::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
        return true;
      }

  private:

    const Eigen::Matrix<Scalar_, 3, 1> point_;

  };

template <typename Scalar_>
  class PlaneFit
  {
  public:

    void setCloud(const typename Types_<Scalar_>::Point3Matrix::ConstPtr & cloud)
    {
      cloud_ = cloud;
    }

    typename Types_<Scalar_>::Plane fit()
    {
      return fit(*cloud_);
    }

    typename Types_<Scalar_>::Plane robustFit(Scalar_ scale = Scalar_(1.0))
    {
      return robustFit(*cloud_, scale);
    }

    typename Types_<Scalar_>::Plane robustFit(const typename Types_<Scalar_>::Plane & initial_plane,
                                              Scalar_ scale = Scalar_(1.0))
    {
      return robustFit(initial_plane, *cloud_, scale);
    }

    static typename Types_<Scalar_>::Plane fit(const typename Types_<Scalar_>::Point3Matrix & cloud);

    static typename Types_<Scalar_>::Plane robustFit(const typename Types_<Scalar_>::Point3Matrix & cloud,
                                                     Scalar_ scale = Scalar_(1.0));

    static typename Types_<Scalar_>::Plane robustFit(const typename Types_<Scalar_>::Plane & initial_plane,
                                                     const typename Types_<Scalar_>::Point3Matrix & cloud,
                                                     Scalar_ scale = Scalar_(1.0));

  private:

    typename Types_<Scalar_>::Point3Matrix::ConstPtr cloud_;

  };

} /* namespace calibration */

#include <impl/calibration_common/ceres/plane_fit.hpp>

#endif /* CALIBRATION_COMMON_CERES_PLANE_FIT_H_ */
