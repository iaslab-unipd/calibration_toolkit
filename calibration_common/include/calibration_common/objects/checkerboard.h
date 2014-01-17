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

#ifndef CALIBRATION_COMMON_OBJECTS_CHECKERBOARD_H_
#define CALIBRATION_COMMON_OBJECTS_CHECKERBOARD_H_

#include <calibration_common/objects/planar_object.h>
#include <calibration_common/color/view.h>

namespace calibration
{

class Checkerboard : public PlanarObject
{
public:

  typedef boost::shared_ptr<Checkerboard> Ptr;
  typedef boost::shared_ptr<const Checkerboard> ConstPtr;

  typedef Types::Point3Matrix::Element Element;
  typedef Types::Point3Matrix::ConstElement ConstElement;

  Checkerboard(size_t cols,
               size_t rows,
               Types::Scalar cell_width,
               Types::Scalar cell_height)
    : PlanarObject(),
      corners_(cols, rows),
      cell_width_(cell_width),
      cell_height_(cell_height)
  {
    assert(cols > 0);
    assert(rows > 0);
    assert(cell_width > 0);
    assert(cell_height > 0);
    for (size_t r = 0; r < rows; ++r)
      for (size_t c = 0; c < cols; ++c)
        corners_(c, r) << c * cell_width, r * cell_height, Types::Scalar(0);
  }

  template <typename ColorSensor>
    Checkerboard(const ColorView<ColorSensor, Checkerboard> & view)
      : PlanarObject(),
        corners_(view.object()->corners()),
        cell_width_(view.object()->cellWidth()),
        cell_height_(view.object()->cellHeight())

    {
      setParent(view.sensor());
      setPlane(view.object()->plane());

      std::stringstream ss;
      ss << view.object()->frameId() << "_" << view.id();
      setFrameId(ss.str());

      transform(view.sensor()->cameraModel()->estimatePose(view.points(), view.object()->corners()));
    }

  virtual ~Checkerboard()
  {
    // Do nothing
  }

  virtual void transform(const Types::Transform & transform)
  {
    PlanarObject::transform(transform);
    corners_.transform(transform);
  }

  virtual const Types::Point3Matrix & points() const
  {
    return corners();
  }

  Types::Point3 center() const
  {
    return Types::Point3((corners_(0, 0) + corners_(cols() - 1, rows() - 1)) / 2);
  }

  Types::Point3Matrix & corners()
  {
    return corners_;
  }

  const Types::Point3Matrix & corners() const
  {
    return corners_;
  }

  size_t size() const
  {
    return corners_.size();
  }

  size_t rows() const
  {
    return corners_.ySize();
  }

  size_t cols() const
  {
    return corners_.xSize();
  }

  Types::Scalar cellWidth() const
  {
    return cell_width_;
  }

  Types::Scalar cellHeight() const
  {
    return cell_height_;
  }

  Types::Scalar area() const
  {
    return width() * height();
  }

  Types::Scalar width() const
  {
    return cellWidth() * (cols() + 1);
  }

  Types::Scalar height() const
  {
    return cellHeight() * (rows() + 1);
  }

  Element operator [](size_t index)
  {
    return corners_[index];
  }

  const ConstElement operator [](size_t index) const
  {
    return corners_[index];
  }

  const ConstElement at(size_t col,
                        size_t row) const
  {
    return corners_.at(col, row);
  }

  Element at(size_t col,
             size_t row)
  {
    return corners_.at(col, row);
  }

  const ConstElement operator ()(size_t col,
                                 size_t row) const
  {
    return corners_(col, row);
  }

  Element operator ()(size_t col,
                      size_t row)
  {
    return corners_(col, row);
  }

  void toMarker(visualization_msgs::Marker & marker) const;

private:

  Types::Scalar cell_width_;
  Types::Scalar cell_height_;

  Types::Point3Matrix corners_;

};

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_OBJECTS_CHECKERBOARD_H_ */
