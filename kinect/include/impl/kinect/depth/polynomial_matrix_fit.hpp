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

#ifndef IMPL_KINECT_DEPTH_POLYNOMIAL_MATRIX_FIT_HPP_
#define IMPL_KINECT_DEPTH_POLYNOMIAL_MATRIX_FIT_HPP_

#include <kinect/depth/polynomial_matrix_fit.h>
#include <ceres/ceres.h>

namespace calibration
{

template <typename PolynomialT_>
  void PolynomialMatrixSimpleModelFit_<PolynomialT_>::addAccumulatedPoints(const Plane & plane)
  {
    for (Size1 i = 0; i < accumulation_bins_.elements(); ++i)
    {
      if (accumulation_bins_[i].isEmpty())
        continue;

      typename Types<Scalar>::Point3 point = accumulation_bins_[i].average();
      typename Types<Scalar>::Line line(point, Types<Scalar>::Vector3::UnitZ());
      //typename Types<Scalar>::Line line(Types<Scalar>::Vector3::Zero(), point.normalized());

      data_bins_[i].push_back(std::make_pair(point.z(), line.intersectionPoint(plane).z()));

      typename Types<Scalar>::Line line2(Types<Scalar>::Vector3::Zero(), point.normalized());
      std::cout << point.z() << ", " << line.intersectionPoint(plane).z() << " = " << line2.intersectionPoint(plane).z() << std::endl;

      accumulation_bins_[i].reset();
    }
  }

template <typename PolynomialT_>
  void PolynomialMatrixSimpleModelFit_<PolynomialT_>::update()
  {
    assert(model());

#pragma omp parallel for
    for (Size1 y_index = 0; y_index < data_bins_.size().y(); ++y_index)
    {
      for (Size1 x_index = 0; x_index < data_bins_.size().x(); ++x_index)
      {
        const DataBin & distorsion_bin = data_bins_(x_index, y_index);
        const Size1 bin_size = distorsion_bin.size();

        if (bin_size < 5 * Degree)
          continue;

        ceres::Problem problem;
        for (Size1 i = 0; i < bin_size; ++i)
        {
          PolynomialResidual<PolynomialT_> * residual = new PolynomialResidual<PolynomialT_>(distorsion_bin[i].first,
                                                                                             distorsion_bin[i].second);
          problem.AddResidualBlock(new ceres::AutoDiffCostFunction<PolynomialResidual<PolynomialT_>, 1, Size>(residual),
                                   NULL,
                                   model()->polynomial(x_index, y_index).data());
        }

        ceres::Solver::Options options;
        options.max_num_iterations = 25;
        options.linear_solver_type = ceres::DENSE_QR;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

      }
    }
  }

template <typename PolynomialT_, typename ScalarT_>
  void PolynomialMatrixSimpleModelFitEigen<PolynomialT_, ScalarT_>::addPoint(Size1 x_index,
                                                                             Size1 y_index,
                                                                             const Point & point,
                                                                             const Plane & plane)
  {
    assert(Base::model());
    if (point.hasNaN())
      return;

//    typename Types<Scalar>::Line line(point, Types<Scalar>::Vector3::UnitZ());
    typename Types<Scalar>::Line line(Types<Scalar>::Vector3::Zero(), point.normalized());
    Base::data_bins_(x_index, y_index).push_back(std::make_pair(point.z(), line.intersectionPoint(plane).z()));
  }

template <typename PolynomialT_, typename PCLPoint_, typename ScalarT_>
  void PolynomialMatrixSimpleModelFitPCL<PolynomialT_, PCLPoint_, ScalarT_>::addPoint(Size1 x_index,
                                                                                      Size1 y_index,
                                                                                      const Point & point,
                                                                                      const Plane & plane)
  {
    assert(Base::model());
    if (not pcl::isFinite(point))
      return;

    typename Types<Scalar>::Point3 eigen_point(point.x, point.y, point.z);
    typename Types<Scalar>::Line line(eigen_point, Types<Scalar>::Vector3::UnitZ());

    Base::data_bins_(x_index, y_index).push_back(std::make_pair(eigen_point.z(), line.intersectionPoint(plane).z()));
  }

template <typename PolynomialT_>
  void PolynomialMatrixSmoothModelFit_<PolynomialT_>::addAccumulatedPoints(const Plane & plane)
  {
    for (Size1 i = 0; i < accumulation_bins_.elements(); ++i)
    {
      Scalar x = 0;
      Scalar y = 0;
      Scalar weight = 0;

      std::vector<AccumulationBinData> & acc_vec = accumulation_bins_[i];
      for (Size1 j = 0; j < acc_vec.size(); ++j)
      {
        const AccumulationBinData & acc_data = acc_vec[j];
        //typename Types<Scalar>::Line line_old(acc_data.point_, Types<Scalar>::Vector3::UnitZ());
        typename Types<Scalar>::Line line(Types<Scalar>::Vector3::Zero(), acc_data.point_.normalized());

        x += acc_data.point_.z() * acc_data.weight_;
        y += line.intersectionPoint(plane).z() * acc_data.weight_;
        weight += acc_data.weight_;

      }
      if (acc_vec.size() > 0 and weight > 0)
      {
        x /= weight;
        y /= weight;
        data_bins_[i].push_back(Data(x, y, 1.0 / depth_error_function_.evaluate(y)));
        acc_vec.clear();
      }
    }
  }

template <typename PolynomialT_>
  void PolynomialMatrixSmoothModelFit_<PolynomialT_>::update()
  {
    assert(model());

#pragma omp parallel for
    for (Size1 y_index = 0; y_index < data_bins_.size().y(); ++y_index)
    {
      for (Size1 x_index = 0; x_index < data_bins_.size().x(); ++x_index)
      {
        const DataBin & data_bin = data_bins_(x_index, y_index);
        const Size1 bin_size = data_bin.size();

        if (bin_size < 5 * Degree)
          continue;


        ceres::Problem problem;

        for (Size1 i = 0; i < bin_size; ++i)
        {
          const Data & data = data_bin[i];

          WeightedPolynomialResidual<PolynomialT> * residual = new WeightedPolynomialResidual<PolynomialT>(data.x_, data.y_, data.weight_);
          problem.AddResidualBlock(new ceres::AutoDiffCostFunction<WeightedPolynomialResidual<PolynomialT>, 1, Size>(residual),
                                   NULL,
                                   model()->polynomial(x_index, y_index).data());
        }

        ceres::Solver::Options options;
        options.max_num_iterations = 25;
        options.linear_solver_type = ceres::DENSE_QR;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

      }
    }
  }

template <typename PolynomialT_, typename ScalarT_>
  void PolynomialMatrixSmoothModelFitEigen<PolynomialT_, ScalarT_>::addPoint(Size1 x_index,
                                                                             Size1 y_index,
                                                                             const Point & point,
                                                                             const Plane & plane)
  {
    assert(Base::model());
    if (point.hasNaN())
      return;

    typename Types<Scalar>::Line line(point, Types<Scalar>::Vector3::UnitZ());

    const std::vector<typename ModelT::LookupTableData> & lt_data = Base::model()->lookupTable(x_index, y_index);
    for (Size1 i = 0; i < lt_data.size(); ++i)
      Base::data_bins_(lt_data[i].index_).push_back(Data(point.z(), line.intersectionPoint(plane).z(), lt_data[i].weight_));
  }

template <typename PolynomialT_, typename PCLPoint_, typename ScalarT_>
  void PolynomialMatrixSmoothModelFitPCL<PolynomialT_, PCLPoint_, ScalarT_>::addPoint(Size1 x_index,
                                                                                      Size1 y_index,
                                                                                      const Point & point,
                                                                                      const Plane & plane)
  {
    assert(Base::model());
    if (not pcl::isFinite(point))
      return;

    typename Types<Scalar>::Point3 eigen_point(point.x, point.y, point.z);
    typename Types<Scalar>::Line line(eigen_point, Types<Scalar>::Vector3::UnitZ());

    const std::vector<typename ModelT::LookupTableData> & lt_data = Base::model()->lookupTable(x_index, y_index);
    for (Size1 i = 0; i < lt_data.size(); ++i)
      Base::data_bins_(lt_data[i].index_).push_back(Data(eigen_point.z(), line.intersectionPoint(plane).z(), lt_data[i].weight_));
  }

} /* namespace calibration */
#endif /* IMPL_KINECT_DEPTH_POLYNOMIAL_MATRIX_FIT_HPP_ */
