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

#ifndef UNIPD_CALIBRATION_CALIBRATION_COMMON_OBJECTS_PLANAR_OBJECT_H_
#define UNIPD_CALIBRATION_CALIBRATION_COMMON_OBJECTS_PLANAR_OBJECT_H_

#include <calibration_common/objects/base_object.h>

namespace unipd
{
namespace calib
{

/**
 * @brief The PlanarObject class
 */
class PlanarObject : public BaseObject
{
public:

  PlanarObject (const PlanarObject & other) = default;

  PlanarObject (PlanarObject && other) = default;

  PlanarObject & operator = (const PlanarObject & other) = default;

  PlanarObject & operator = (PlanarObject && other) = default;

  explicit
  PlanarObject (const Plane3 & plane)
    : BaseObject(),
      plane_(plane),
      initial_plane_(plane)
  {
    // Do nothing
  }

  PlanarObject (const std::string & frame_id,
                const Plane3 & plane)
    : BaseObject(frame_id),
      plane_(plane),
      initial_plane_(plane)
  {
    // Do nothing
  }

  virtual
  ~PlanarObject ()
  {
    // Do nothing
  }

  virtual void
  reset () override
  {
    BaseObject::reset();
    plane_ = initial_plane_;
  }

  virtual void
  setPose (const Pose3 & pose) override
  {
    BaseObject::setPose(pose);
    plane_ = initial_plane_;
    plane_.transform(pose);
  }

  virtual void
  transform (const Transform3 & transform) override
  {
    BaseObject::transform(transform);
    plane_.transform(transform);
  }

  const Plane3 &
  plane () const
  {
    return plane_;
  }

private:

  Plane3 plane_;
  const Plane3 initial_plane_;

};

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATION_COMMON_OBJECTS_PLANAR_OBJECT_H_
