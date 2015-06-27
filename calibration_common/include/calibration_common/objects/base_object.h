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

#ifndef UNIPD_CALIBRATION_CALIBRATION_COMMON_OBJECTS_BASE_OBJECT_H_
#define UNIPD_CALIBRATION_CALIBRATION_COMMON_OBJECTS_BASE_OBJECT_H_

#include <ostream>
#include <calibration_common/base/geometry.h>

namespace unipd
{
namespace calib
{

class BaseObject
{
public:

  BaseObject () = default;

  BaseObject (const BaseObject & other) = default;

  BaseObject (BaseObject && other) = default;

  BaseObject & operator = (const BaseObject & other) = default;

  BaseObject & operator = (BaseObject && other) = default;

  explicit
  BaseObject (const std::string & frame_id)
    : frame_id_(frame_id)
  {
    // Do nothing
  }

  virtual
  ~BaseObject ()
  {
    // Do nothing
  }

  virtual void
  transform (const Transform3 & transform)
  {
    pose_ = transform * pose_;
  }

  const Pose3 &
  pose () const
  {
    return pose_;
  }

  const std::string &
  frameId () const
  {
    return frame_id_;
  }

  const std::shared_ptr<const BaseObject> &
  parent () const
  {
    return parent_;
  }

  bool
  hasParent () const
  {
    return static_cast<bool>(parent_);
  }

  void
  setFrameId (const std::string & frame_id)
  {
    frame_id_ = frame_id;
  }

  void
  setParent (const std::shared_ptr<const BaseObject> & parent)
  {
    parent_ = parent;
  }

  virtual void
  reset ()
  {
    pose_ = Pose3::Identity();
    parent_.reset();
  }

  virtual void
  setPose (const Pose3 & pose)
  {
    pose_ = pose;
  }

  friend std::ostream &
  operator << (std::ostream & stream,
               const BaseObject & object)
  {
    stream << "frame_id: " << object.frame_id_ << ", "
           << "parent: " << object.parent_ << " (= " << (object.hasParent() ? object.parent_->frameId() : "null") << "), "
           << "pose: {"
           << "rotation: [" << Quaternion(object.pose_.rotation()).coeffs().transpose() << "], "
           << "translation: [" << object.pose_.translation().transpose() << "]}";
    return stream;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  Pose3 pose_ = Pose3::Identity();
  std::string frame_id_;
  std::shared_ptr<const BaseObject> parent_;

};

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATION_COMMON_OBJECTS_BASE_OBJECT_H_
