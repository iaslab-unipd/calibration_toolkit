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

#ifndef CALIBRATION_COMMON_OBJECTS_BASE_OBJECT_H_
#define CALIBRATION_COMMON_OBJECTS_BASE_OBJECT_H_

#include <sstream>
#include <calibration_common/objects/globals.h>

namespace geometry_msgs
{
template <class ContainerAllocator>
  struct TransformStamped_;

typedef TransformStamped_<std::allocator<void> > TransformStamped;
}

namespace calibration
{

/**
 * @brief The BaseObject class
 */
class BaseObject
{
public:

  typedef boost::shared_ptr<BaseObject> Ptr;
  typedef boost::shared_ptr<const BaseObject> ConstPtr;

  static Size1 count_;

  /**
   * @brief BaseObject
   */
  BaseObject()
    : pose_(Pose::Identity())
  {
    ++count_;
    std::stringstream ss;
    ss << "object_" << count_;
    frame_id_ = ss.str();
  }

  /**
   * @brief BaseObject
   * @param frame_id
   */
  explicit BaseObject(const std::string & frame_id)
    : pose_(Pose::Identity()),
      frame_id_(frame_id)
  {
    ++count_;
  }

  /**
   * @brief ~BaseObject
   */
  virtual ~BaseObject()
  {
    // Do nothing
  }

  /**
   * @brief transform
   * @param transform
   */
  inline virtual void transform(const Transform & transform)
  {
    pose_ = transform * pose_;
  }

  /**
   * @brief pose
   * @return
   */
  inline const Pose & pose() const
  {
    return pose_;
  }

  /**
   * @brief frameId
   * @return
   */
  inline const std::string & frameId() const
  {
    return frame_id_;
  }

  /**
   * @brief parent
   * @return
   */
  inline const ConstPtr & parent() const
  {
    return parent_;
  }

  /**
   * @brief setFrameId
   * @param frame_id
   */
  inline void setFrameId(const std::string & frame_id)
  {
    frame_id_ = frame_id;
  }

  /**
   * @brief setParent
   * @param parent
   */
  inline void setParent(const ConstPtr & parent)
  {
    parent_ = parent;
  }

  /**
   * @brief setPose
   * @param pose
   */
  inline void setPose(const Pose & pose)
  {
    pose_ = pose;
  }

  /**
   * @brief toTF
   * @param transform_msg
   * @return
   */
  bool toTF(geometry_msgs::TransformStamped & transform_msg) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  Pose pose_;
  std::string frame_id_;
  ConstPtr parent_;

};

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_OBJECTS_BASE_OBJECT_H_ */
