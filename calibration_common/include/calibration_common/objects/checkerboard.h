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

/**
 * @brief The Checkerboard class
 */
class Checkerboard : public PlanarObject
{
public:

  typedef boost::shared_ptr<Checkerboard> Ptr;
  typedef boost::shared_ptr<const Checkerboard> ConstPtr;

  typedef Cloud3::Element Element;
  typedef Cloud3::ConstElement ConstElement;

  /**
   * @brief Checkerboard
   * @param cols
   * @param rows
   * @param cell_width
   * @param cell_height
   */
  Checkerboard(size_t cols,
               size_t rows,
               Scalar cell_width,
               Scalar cell_height)
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
        corners_(c, r) << c * cell_width, r * cell_height, Scalar(0);
  }

  /**
   * @brief Checkerboard
   * @param view
   */
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

  /**
   * @brief ~Checkerboard
   */
  virtual ~Checkerboard()
  {
    // Do nothing
  }

  /**
   * @brief transform
   * @param transform
   */
  inline virtual void transform(const Transform & transform)
  {
    PlanarObject::transform(transform);
    corners_.transform(transform);
  }

  /**
   * @brief points
   * @return
   */
  inline virtual const Cloud3 & points() const
  {
    return corners();
  }

  /**
   * @brief center
   * @return
   */
  inline Point3 center() const
  {
    return Point3((corners_(0, 0) + corners_(cols() - 1, rows() - 1)) / 2);
  }

  /**
   * @brief corners
   * @return
   */
  inline Cloud3 & corners()
  {
    return corners_;
  }

  /**
   * @brief corners
   * @return
   */
  inline const Cloud3 & corners() const
  {
    return corners_;
  }

  /**
   * @brief size
   * @return
   */
  inline size_t size() const
  {
    return corners_.size();
  }

  /**
   * @brief rows
   * @return
   */
  inline size_t rows() const
  {
    return corners_.ySize();
  }

  /**
   * @brief cols
   * @return
   */
  inline size_t cols() const
  {
    return corners_.xSize();
  }

  /**
   * @brief cellWidth
   * @return
   */
  inline Scalar cellWidth() const
  {
    return cell_width_;
  }

  /**
   * @brief cellHeight
   * @return
   */
  inline Scalar cellHeight() const
  {
    return cell_height_;
  }

  /**
   * @brief area
   * @return
   */
  inline Scalar area() const
  {
    return width() * height();
  }

  /**
   * @brief width
   * @return
   */
  inline Scalar width() const
  {
    return cellWidth() * (cols() + 1);
  }

  /**
   * @brief height
   * @return
   */
  inline Scalar height() const
  {
    return cellHeight() * (rows() + 1);
  }

  /**
   * @brief operator []
   * @param index
   * @return
   */
  inline Element operator [](size_t index)
  {
    return corners_[index];
  }

  /**
   * @brief operator []
   * @param index
   * @return
   */
  inline const ConstElement operator [](size_t index) const
  {
    return corners_[index];
  }

  /**
   * @brief at
   * @param col
   * @param row
   * @return
   */
  inline const ConstElement at(size_t col,
                        size_t row) const
  {
    return corners_.at(col, row);
  }

  /**
   * @brief at
   * @param col
   * @param row
   * @return
   */
  inline Element at(size_t col,
                    size_t row)
  {
    return corners_.at(col, row);
  }

  /**
   * @brief operator ()
   * @param col
   * @param row
   * @return
   */
  inline const ConstElement operator ()(size_t col,
                                        size_t row) const
  {
    return corners_(col, row);
  }

  /**
   * @brief operator ()
   * @param col
   * @param row
   * @return
   */
  inline Element operator ()(size_t col,
                             size_t row)
  {
    return corners_(col, row);
  }

  /**
   * @brief toMarker
   * @param marker
   */
  void toMarker(visualization_msgs::Marker & marker) const;

private:

  Scalar cell_width_;
  Scalar cell_height_;

  Cloud3 corners_;

};

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_OBJECTS_CHECKERBOARD_H_ */
