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

#include <multisensor_calibration/calibration.h>

namespace unipd
{
namespace calib
{

const std::string &
Calibration::beginStep (const Checkerboard & checkerboard)
{
  auto new_checkerboard = std::make_shared<Checkerboard>(checkerboard);
  std::stringstream ss;
  ss << checkerboard.frameId() << "_" << current_step_;
  new_checkerboard->setFrameId(ss.str());

  current_step_ = matrix_.newStep(new_checkerboard, current_plane_);
  all_objects_[new_checkerboard->frameId()] = new_checkerboard;

  if (current_plane_)
    current_plane_steps_.push_back(current_step_);
  ROS_DEBUG_STREAM(log_ << "==> Begin step " << current_step_ << " [" << new_checkerboard->frameId() << "]");
  return new_checkerboard->frameId();
}

void
Calibration::endStep ()
{
  assert(current_step_ >= 0);
  if (tryAddCheckerboardToTree(current_step_))
  {
    auto step_info = matrix_.stepInfo(current_step_);
    for (int sensor_index : step_info->sensors)
    {
      bool in_tree = inTree(matrix_.sensorInfo(sensor_index)->sensor);
      if (not in_tree)
        in_tree = tryAddSensorToTree(sensor_index);

      if (in_tree)
        addOptimizationConstraint(current_step_, sensor_index);
    }
    optimization_.perform();
  }

  ROS_DEBUG_STREAM(log_ << "<== End step " << current_step_ << " [" << matrix_.stepInfo(current_step_)->checkerboard->frameId() << "]");
}

const std::string &
Calibration::beginPlane ()
{
  assert(not current_plane_);
  current_plane_ = std::make_shared<PlanarObject>(Plane3(Vector3::UnitZ(), 0));
  std::stringstream ss;
  ss << "plane_" << current_step_;
  current_plane_->setFrameId(ss.str());
  current_plane_steps_.clear();
  all_objects_[current_plane_->frameId()] = current_plane_;
  ROS_INFO_STREAM(log_ << "--> Begin plane [" << current_plane_->frameId() << "]");
  return current_plane_->frameId();
}

void
Calibration::endPlane ()
{
  assert(current_plane_);
  ROS_INFO_STREAM(log_ << *current_plane_);
  current_plane_->reset();
  ROS_INFO_STREAM(log_ << "<-- End plane [" << current_plane_->frameId() << "]");
}

const std::shared_ptr<const BaseObject> &
Calibration::get (const std::string & frame_id)
{
  assert(all_objects_.count(frame_id) > 0);
  return all_objects_[frame_id];
}

bool
Calibration::isPoseEstimated (const std::string & frame_id)
{
  assert(all_objects_.count(frame_id) > 0);
  return inTree(all_objects_[frame_id]);
}

// protected:

Calibration::EstimatedPose
Calibration::estimateSensorPose (const SensorInfo & sensor_info)
{
  if (sensor_info.type == SensorInfo::Type::INTENSITY)
  {
    for (int step : sensor_info.steps)
    {
      std::shared_ptr<StepInfo> & step_info = matrix_.stepInfo(step);
      if (inTree(step_info->checkerboard))
      {
        std::shared_ptr<const IntensityData> data = matrix_.data<IntensityData>(step, sensor_info.index);
        std::shared_ptr<PinholeSensor> sensor = std::static_pointer_cast<PinholeSensor>(sensor_info.sensor);
        Transform3 inverse_transform = sensor->cameraModel().estimatePose(data->corners, step_info->initialCorners());
        return EstimatedPose{step_info->checkerboard, inverse_transform.inverse()};
      }
    }
  }
  else // (sensor_info.type == SensorInfo::Type::DEPTH)
  {
    // TODO
    return EstimatedPose();
  }

  return EstimatedPose();
}

Calibration::EstimatedPose
Calibration::estimateCheckerboardPose (const StepInfo & step_info)
{
  for (int sensor_index : step_info.sensors)
  {
    std::shared_ptr<SensorInfo> & sensor_info = matrix_.sensorInfo(sensor_index);
    if (sensor_info->type == SensorInfo::Type::INTENSITY and inTree(sensor_info->sensor))
    {
      std::shared_ptr<const IntensityData> data = matrix_.data<IntensityData>(step_info.index, sensor_index);
      std::shared_ptr<PinholeSensor> sensor = std::static_pointer_cast<PinholeSensor>(sensor_info->sensor);
      Transform3 transform = sensor->cameraModel().estimatePose(data->corners, step_info.checkerboard->corners());
      return EstimatedPose{sensor, transform};
    }
  }

  return EstimatedPose();
}



// private:

bool
Calibration::tryAddSensorToTree (int sensor_index)
{
  std::shared_ptr<SensorInfo> & sensor_info = matrix_.sensorInfo(sensor_index);
  assert(not inTree(sensor_info->sensor));

  if (sensor_info->sensor->hasParent() and not inTree(sensor_info->sensor->parent()))
    return false;

  bool added_to_tree = false;
  if (sensor_info->sensor->hasParent() and inTree(sensor_info->sensor->parent()))
  {
    addToTree(sensor_info->sensor);
    added_to_tree = true;
  }
  else
  {
    EstimatedPose pose = estimateSensorPose(*sensor_info);
    if (pose.parent)
    {
      addToTree(sensor_info->sensor, pose.parent, pose.transform);
      added_to_tree = true;
    }
  }

  return added_to_tree;
}

bool
Calibration::tryAddCheckerboardToTree (int step)
{
  std::shared_ptr<StepInfo> & step_info = matrix_.stepInfo(step);
  assert(not inTree(step_info->checkerboard));

  bool added_to_tree = false;

  EstimatedPose pose = estimateCheckerboardPose(*step_info);
  if (pose.parent)
  {
    addToTree(step_info->checkerboard, pose.parent, pose.transform);
    added_to_tree = true;

    optimization_.addObject(step_info->checkerboard);

    if (current_plane_)
    {
      if (not inTree(current_plane_))
      {
        ROS_INFO_STREAM("***");
        addToTree(current_plane_, pose.parent, pose.transform);
        optimization_.addObject(current_plane_);
        optimization_.setPlaneTransform(current_plane_);
      }

      ROS_INFO_STREAM("------ \n" << current_plane_->pose().matrix() << "\n" << step_info->checkerboard->pose().matrix());

      step_info->checkerboard->transform(current_plane_->pose().inverse());
      Quaternion q = Quaternion::FromTwoVectors(step_info->checkerboard->pose().rotation().col(2), Vector3::UnitZ());
      step_info->checkerboard->transform(Transform3::Identity() * q);
      Vector3 t = Plane3(Vector3::UnitZ(), 0).projection(step_info->checkerboard->pose().translation()) - step_info->checkerboard->pose().translation();
      step_info->checkerboard->transform(Transform3::Identity() * Translation3(t));
      step_info->checkerboard->setParent(current_plane_);

      ROS_INFO_STREAM(step_info->checkerboard->pose().translation().transpose() << "\n" << step_info->checkerboard->pose().matrix());

      optimization_.set3DOFTransform(step_info->checkerboard);
    }
    else
    {
      optimization_.set6DOFTransform(step_info->checkerboard);
    }

  }

  return added_to_tree;
}

void
Calibration::addOptimizationConstraint (int step,
                                        int sensor_index)
{
  assert (matrix_.data<Data>(step, sensor_index));

  const std::shared_ptr<SensorInfo> & sensor_info = matrix_.sensorInfo(sensor_index);
  const std::shared_ptr<StepInfo> & step_info = matrix_.stepInfo(step);
  if (sensor_info->type == SensorInfo::Type::INTENSITY)
  {
    if (current_plane_)
    {
      optimization_.addConstraint(step_info->checkerboard, step_info->plane,
                                  std::static_pointer_cast<PinholeSensor>(sensor_info->sensor),
                                  matrix_.data<IntensityData>(step, sensor_index));
    }
    else
    {
      optimization_.addConstraint(step_info->checkerboard,
                                  std::static_pointer_cast<PinholeSensor>(sensor_info->sensor),
                                  matrix_.data<IntensityData>(step, sensor_index));
    }
  }
  else
  {
//    if (current_plane_)
//    {
//      // TODO
//    }
//    else
//    {
      optimization_.addConstraint(step_info->checkerboard,
                                  std::static_pointer_cast<DepthSensor>(sensor_info->sensor),
                                  matrix_.data<DepthData>(step, sensor_index));
//    }
  }

  if (sensor_index != fixed_sensor_index_)
  {
    optimization_.setVariable(sensor_info->sensor);
    ROS_INFO_STREAM(log_ << " +  Object [" << sensor_info->sensor->frameId() << "] pose is now variable.");
  }
}

bool
Calibration::inTree (const std::shared_ptr<const BaseObject> & object) const
{
  return tree_.count(object) > 0;
}

void
Calibration::addToTree (const std::shared_ptr<BaseObject> & object,
                        const std::shared_ptr<const BaseObject> & parent,
                        const Pose3 & pose)
{
  assert(not object->hasParent());
  object->reset();
  object->transform(pose);
  object->setParent(parent);
  addToTree(object);
}

void
Calibration::addToTree (const std::shared_ptr<BaseObject> & object)
{
  assert(object->hasParent() and not inTree(object) and inTree(object->parent()));

  while (object->parent()->hasParent())
  {
    object->transform(object->parent()->pose());
    object->setParent(object->parent()->parent());
  }

  tree_.insert(object);
  ROS_INFO_STREAM(log_ << " +  Object [" << object->frameId() << "] added to tree.");
}

void
Calibration::initTree (const std::shared_ptr<Sensor> & sensor)
{
  assert(tree_.empty() and not sensor->hasParent());

  sensor->reset();
  sensor->setParent(world_);

  tree_.insert(world_);
  ROS_INFO_STREAM(log_ << " +  Object [" << world_->frameId() << "] is the tree root.");
  tree_.insert(sensor);
  ROS_INFO_STREAM(log_ << " +  Object [" << sensor->frameId() << "] added to tree.");
}

} // namespace calib
} // namespace unipd
