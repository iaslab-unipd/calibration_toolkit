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

#ifndef IMPL_KINECT_DEPTH_POLYNOMIAL_UNDISTORTION_MAP_FIT_HPP_
#define IMPL_KINECT_DEPTH_POLYNOMIAL_UNDISTORTION_MAP_FIT_HPP_

#include <kinect/depth/polynomial_undistortion_function_fit.h>
#include <calibration_common/ceres/polynomial_fit.h>
#include <ceres/ceres.h>

namespace calibration
{

template <typename Polynomial_, typename Function_>
  void PolynomialUndistortionFunctionFit<Polynomial_, Function_>::addAccumulatedPoints(const typename Types_<Scalar>::Plane & plane)
  {
    if (accumulation_bin_.isEmpty())
      return;

    typename Types_<Scalar>::Point3 point = accumulation_bin_.average();
    typename Types_<Scalar>::Line line(point, Types_<Scalar>::Vector3::UnitZ());

    distorsion_bin_.push_back(std::make_pair(point.z(), line.intersectionPoint(plane).z()));
    accumulation_bin_.reset();
  }

template <typename Polynomial_, typename Function_>
  void PolynomialUndistortionFunctionFit<Polynomial_, Function_>::update()
  {
    typedef PolynomialResidual<Polynomial_> PolynomialResidual_;

    const size_t & bin_size = distorsion_bin_.size();

    if (bin_size < 5 * Degree)
      return;

    ceres::Problem problem;
    for (size_t i = 0; i < bin_size; ++i)
    {
      problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<PolynomialResidual_, 1, Size>(
          new PolynomialResidual_(distorsion_bin_[i].first, distorsion_bin_[i].second)),
        NULL, function_->dataPtr());
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
//    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

//    std::cout << Base::polynomial_.coefficients().transpose() << std::endl;

  }

template <typename Polynomial_>
  void PolynomialUndistortionFunctionFitEigen<Polynomial_>::addPoint(const typename Types_<Scalar>::Point3 & point,
                                                                const typename Types_<Scalar>::Plane & plane)
  {
    typename Types_<Scalar>::Line line(point, Types_<Scalar>::Vector3::UnitZ());

    size_t x_index, y_index;
    Base::distorsion_bin_.push_back(std::make_pair(point.z(), line.intersectionPoint(plane).z()));
  }


template <typename Polynomial_, typename PCLPoint_>
  void PolynomialUndistortionFunctionFitPCL<Polynomial_, PCLPoint_>::addPoint(const PCLPoint_ & point,
                                                                              const typename Types_<Scalar>::Plane & plane)
  {
    if (not pcl::isFinite(point))
      return;

    typename Types_<Scalar>::Point3 eigen_point(point.x, point.y, point.z);
    typename Types_<Scalar>::Line line(eigen_point, Types_<Scalar>::Vector3::UnitZ());

    size_t x_index, y_index;
    Base::distorsion_bin_.push_back(std::make_pair(eigen_point.z(), line.intersectionPoint(plane).z()));
  }

} /* namespace calibration */
#endif /* IMPL_KINECT_DEPTH_POLYNOMIAL_UNDISTORTION_MAP_FIT_HPP_ */
