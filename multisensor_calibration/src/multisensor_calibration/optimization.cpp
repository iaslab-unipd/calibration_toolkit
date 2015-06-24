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

#include <multisensor_calibration/optimization.h>

namespace unipd
{
namespace calib
{

void
Optimization::addObject (const std::shared_ptr<BaseObject> & object)
{
  assert(transform_map_.count(object) == 0);
  std::shared_ptr<RawTransform> raw_transform = std::make_shared<RawTransform>(Pose3::Identity());
  transform_map_.emplace(object, raw_transform);
  problem_.AddParameterBlock(raw_transform->data, 7);
}

void
Optimization::set6DOFTransform (const std::shared_ptr<BaseObject> & object)
{
  assert(transform_map_.count(object) > 0);
  problem_.SetParameterization(transform_map_[object]->data, new Pose3DParameterization());
}

void
Optimization::setVariable (const std::shared_ptr<BaseObject> & object)
{
  assert(transform_map_.count(object) > 0);
  problem_.SetParameterBlockVariable(transform_map_[object]->data);
}

void
Optimization::setFixed (const std::shared_ptr<BaseObject> & object)
{
  assert(transform_map_.count(object) > 0);
  problem_.SetParameterBlockConstant(transform_map_[object]->data);
}

void
Optimization::setPlaneTransform (const std::shared_ptr<BaseObject> & object)
{
  assert(transform_map_.count(object) > 0);
  problem_.SetParameterization(transform_map_[object]->data, new PlaneParameterization());
}

void
Optimization::set3DOFTransform (const std::shared_ptr<BaseObject> & object)
{
  assert(transform_map_.count(object) > 0);
  assert(object->hasParent() and transform_map_.count(std::const_pointer_cast<BaseObject>(object->parent())) > 0);
  assert((object->pose().matrix().bottomRightCorner<2, 2>()).isApprox(Eigen::Matrix<Scalar, 2, 2>::Identity()));

  problem_.SetParameterization(transform_map_[object]->data, new Pose2DParameterization());
}

void
Optimization::addConstraint (const std::shared_ptr<Checkerboard> & checkerboard,
                             const std::shared_ptr<PinholeSensor> & sensor,
                             const std::shared_ptr<const IntensityData> & data)
{
  assert(transform_map_.count(checkerboard) > 0);
  assert(transform_map_.count(sensor) > 0);

  IntensityError * error = new IntensityError(checkerboard, sensor, data);
  using IntensityErrorFunction = ceres::AutoDiffCostFunction<IntensityError, ceres::DYNAMIC, 7, 7>;
  ceres::CostFunction * cost_function = new IntensityErrorFunction(error, error->residualSize());
  problem_.AddResidualBlock(cost_function, NULL, transform_map_[sensor]->data, transform_map_[checkerboard]->data);
}

void
Optimization::addConstraint (const std::shared_ptr<Checkerboard> & checkerboard,
                             const std::shared_ptr<DepthSensor> & sensor,
                             const std::shared_ptr<const DepthData> & data)
{
  assert(transform_map_.count(checkerboard) > 0);
  assert(transform_map_.count(sensor) > 0);

  DepthError * error = new DepthError(checkerboard, sensor, data);
  using DepthErrorFunction = ceres::AutoDiffCostFunction<DepthError, ceres::DYNAMIC, 7, 7>;
  ceres::CostFunction * cost_function = new DepthErrorFunction(error, error->residualSize());
  problem_.AddResidualBlock(cost_function, NULL, transform_map_[sensor]->data, transform_map_[checkerboard]->data);
}

void
Optimization::addConstraint (const std::shared_ptr<Checkerboard> & checkerboard,
                             const std::shared_ptr<PlanarObject> & plane,
                             const std::shared_ptr<PinholeSensor> & sensor,
                             const std::shared_ptr<const IntensityData> & data)
{
  assert(transform_map_.count(checkerboard) > 0);
  assert(transform_map_.count(plane) > 0);
  assert(transform_map_.count(sensor) > 0);

  OnPlaneIntensityError * error = new OnPlaneIntensityError(checkerboard, plane, sensor, data);
  using IntensityErrorFunction = ceres::AutoDiffCostFunction<OnPlaneIntensityError, ceres::DYNAMIC, 7, 7, 7>;
  ceres::CostFunction * cost_function = new IntensityErrorFunction(error, error->residualSize());
  problem_.AddResidualBlock(cost_function, NULL, transform_map_[sensor]->data, transform_map_[checkerboard]->data, transform_map_[plane]->data);
}

void
Optimization::addConstraint (const std::shared_ptr<Checkerboard> & checkerboard,
                             const std::shared_ptr<PlanarObject> & plane,
                             const std::shared_ptr<DepthSensor> & sensor,
                             const std::shared_ptr<const DepthData> & data)
{
  assert(transform_map_.count(checkerboard) > 0);
  assert(transform_map_.count(plane) > 0);
  assert(transform_map_.count(sensor) > 0);

//  OnPlaneDepthError * error = new OnPlaneDepthError(checkerboard, plane, sensor, data);
//  using DepthErrorFunction = ceres::AutoDiffCostFunction<OnPlaneDepthError, ceres::DYNAMIC, 7, 7, 7>;
//  ceres::CostFunction * cost_function = new DepthErrorFunction(error, error->residualSize());
//  problem_.AddResidualBlock(cost_function, NULL, transform_map_[sensor]->data, transform_map_[checkerboard]->data, transform_map_[plane]->data);
}

void
Optimization::perform ()
{
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.max_num_iterations = 200;
//  options.minimizer_progress_to_stdout = true;
  options.num_threads = 8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_, &summary);

  for (auto & pair : transform_map_)
  {
//    Scalar * data = pair.second->data;
    pair.first->transform(parser_.toPose3(pair.second->data));
    parser_.fromPose3(Pose3::Identity(), pair.second->data);
//    Scalar data[6]; fromPose3(pair.first->pose(), data);
    if (pair.first->frameId().substr(0, 13) != "/checkerboard")
      ROS_INFO_STREAM(log_ << "Pose optimized {" << *pair.first << "}");
  }
}

} // namespace calib
} // namespace unipd
