/*
 *  Copyright (C) 2013 - Filippo Basso <bassofil@dei.unipd.it>
 *                       Mauro Antonello <antonelm@dei.unipd.it>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef IMPL_CALIBRATION_COMMON_CERES_POLYNOMIAL_FIT_HPP_
#define IMPL_CALIBRATION_COMMON_CERES_POLYNOMIAL_FIT_HPP_

#include <calibration_common/ceres/polynomial_fit.h>

namespace calibration
{

template <typename PolynomialT_>
  bool PolynomialFit<PolynomialT_>::update()
  {
    typedef PolynomialResidual<PolynomialT_> PolynomialResidualT;

    const Size1 bin_size = data_bin_.size();

    if (bin_size < 5 * Degree)
      return false;

    ceres::Problem problem;
    for (Size1 i = 0; i < bin_size; ++i)
    {
      problem.AddResidualBlock(new ceres::AutoDiffCostFunction<PolynomialResidualT, 1, Size>(new PolynomialResidualT(data_bin_[i].first,
                                                                                                                     data_bin_[i].second)),
                               NULL,
                               polynomial_->dataPtr());
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
//  options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    return true;
  }


template <typename PolynomialT_>
  bool WeightedPolynomialFit<PolynomialT_>::update()
  {
    typedef WeightedPolynomialResidual<PolynomialT_> PolynomialResidualT;
    typedef ceres::AutoDiffCostFunction<PolynomialResidualT, 1, Size> CostFunction;

    const Size1 bin_size = data_bin_.size();

    if (bin_size < 5 * Degree)
      return false;

    ceres::Problem problem;
    for (Size1 i = 0; i < bin_size; ++i)
    {
      const Data & data_bin = data_bin_[i];
      problem.AddResidualBlock(new CostFunction(new PolynomialResidualT(data_bin.x_, data_bin.y_, data_bin.weight_)),
                               NULL,
                               polynomial_->dataPtr());
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
//  options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    return true;
  }

} /* namespace calibration */
#endif /* IMPL_CALIBRATION_COMMON_CERES_POLYNOMIAL_FIT_HPP_ */
