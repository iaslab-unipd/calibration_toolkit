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

#ifndef UNIPD_CALIBRATION_CALIBRATION_COMMON_OBJECTS_CHECKERBOARD_H_
#define UNIPD_CALIBRATION_CALIBRATION_COMMON_OBJECTS_CHECKERBOARD_H_

#include <calibration_common/base/eigen_cloud.h>
#include <calibration_common/objects/planar_object.h>
#include <calibration_common/objects/with_points.h>
#include <calibration_common/pinhole/view.h>

namespace unipd
{
namespace calib
{

class Checkerboard : public PlanarObject,
                     public WithPoints
{
public:

  Checkerboard (const Checkerboard & other) = default;

  Checkerboard (Checkerboard && other) = default;

  Checkerboard & operator = (const Checkerboard & other) = default;

  Checkerboard & operator = (Checkerboard && other) = default;

  Checkerboard (Size1 cols,
                Size1 rows,
                Scalar cell_width,
                Scalar cell_height)
    : PlanarObject(Plane3(Vector3::UnitZ(), 0)),
      corners_(Size2{cols, rows}),
      external_corners_(Size2{2, 2}),
      cell_width_(cell_width),
      cell_height_(cell_height)
  {
    assert(cell_width > 0.0);
    assert(cell_height > 0.0);
    resetCorners();
  }

  Checkerboard (const std::string & frame_id,
                Size1 cols,
                Size1 rows,
                Scalar cell_width,
                Scalar cell_height)
    : Checkerboard(cols, rows, cell_width, cell_height)
  {
    setFrameId(frame_id);
  }

  Checkerboard (const PinholePointsView<Checkerboard> & view)
    : Checkerboard(*view.object())
  {
    setParent(view.sensor());

    std::stringstream ss;
    ss << view.object()->frameId() << "_" << view.id();
    setFrameId(ss.str());

    transform(view.sensor()->cameraModel().estimatePose(view.points(), view.object()->corners()));
  }

  virtual
  ~Checkerboard ()
  {
    // Do nothing
  }

  virtual void
  reset () override
  {
    PlanarObject::reset();
    resetCorners();
  }

  virtual void
  setPose (const Pose3 & pose) override
  {
    PlanarObject::setPose(pose);
    resetCorners();
    corners_.transform(pose);
    external_corners_.transform(pose);
  }

  virtual void
  transform (const Transform3 & transform) override
  {
    PlanarObject::transform(transform);
    corners_.transform(transform);
    external_corners_.transform(transform);
  }

  const Cloud3 &
  corners() const
  {
    return corners_;
  }

  virtual const Cloud3 &
  points() const override
  {
    return corners_;
  }

  const Cloud3 &
  externalCorners() const
  {
    return external_corners_;
  }

  Point3
  center () const
  {
    return Point3((corners_(0, 0) + corners_(cols() - 1, rows() - 1)) / 2);
  }

  Size1
  rows() const
  {
    return corners_.size().y;
  }

  Size1
  cols() const
  {
    return corners_.size().x;
  }

  Scalar
  cellWidth () const
  {
    return cell_width_;
  }

  Scalar
  cellHeight () const
  {
    return cell_height_;
  }

  Scalar
  area () const
  {
    return width() * height();
  }

  Scalar
  width () const
  {
    return cellWidth() * (cols() + 1);
  }

  Scalar
  height () const
  {
    return cellHeight() * (rows() + 1);
  }

private:

  void
  resetCorners ()
  {
    for (Size1 y = 0; y < corners_.size().y; ++y)
      for (Size1 x = 0; x < corners_.size().x; ++x)
        corners_(x, y) << x * cell_width_, y * cell_height_, Scalar(0);

    external_corners_(0, 0) << -cell_width_, -cell_height_, Scalar(0);
    external_corners_(0, 1) << -cell_width_, corners_.size().y * cell_height_, Scalar(0);
    external_corners_(1, 0) << corners_.size().x * cell_width_, -cell_height_, Scalar(0);
    external_corners_(1, 1) << corners_.size().x * cell_width_, corners_.size().y * cell_height_, Scalar(0);
  }

  Scalar cell_width_;
  Scalar cell_height_;

  Cloud3 corners_;
  Cloud3 external_corners_;
  
};

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATION_COMMON_OBJECTS_CHECKERBOARD_H_
