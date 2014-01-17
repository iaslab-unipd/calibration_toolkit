/*
 *  Copyright (C) 2013 - Filippo Basso <bassofil@dei.unipd.it>
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

#ifndef CALIBRATON_COMMON_OBJECTS_VIEW_H_
#define CALIBRATON_COMMON_OBJECTS_VIEW_H_

#include <calibration_common/base/point_matrix.h>
#include <visualization_msgs/Marker.h>
#include <calibration_common/objects/globals.h>

namespace calibration
{

class View
{
public:

  typedef boost::shared_ptr<View> Ptr;
  typedef boost::shared_ptr<const View> ConstPtr;

  void setId(const std::string & id)
  {
    id_ = id;
  }

  const std::string & id() const
  {
    return id_;
  }

protected:

  std::string id_;

};

template <typename Sensor, typename Data, typename Object, int Dimension>
  class View_ : public View
  {
  public:

    typedef boost::shared_ptr<View_> Ptr;
    typedef boost::shared_ptr<const View_> ConstPtr;

    View_()
      : points_std_dev_(Types::Scalar(1.0))
    {

    }

    void setPoints(const PointMatrix<Types::Scalar, Dimension> & points)
    {
      points_ = points;
    }

    void setPoints(const PointMatrix<Types::Scalar, Dimension> & points,
                   const std::vector<int> & indices)
    {
      points_.resize(indices.size(), 1);
      for (size_t i = 0; i < indices.size(); ++i)
        points_[i] << points[indices[i]];
    }

    const PointMatrix<Types::Scalar, Dimension> & points() const
    {
      return points_;
    }

    void setObject(const typename Object::ConstPtr & object)
    {
      object_ = object;
    }

    const typename Object::ConstPtr & object() const
    {
      return object_;
    }

    void setSensor(const typename Sensor::ConstPtr & sensor)
    {
      sensor_ = sensor;
    }

    const typename Sensor::ConstPtr & sensor() const
    {
      return sensor_;
    }

    void setData(const Data & data)
    {
      data_ = data;
    }

    const Data & data() const
    {
      return data_;
    }

    void setPointsStdDev(const Types::Scalar & points_std_dev)
    {
      points_std_dev_ = points_std_dev;
    }

    const Types::Scalar & pointsStdDev() const
    {
      return points_std_dev_;
    }

  protected:

    typename Sensor::ConstPtr sensor_;
    Data data_;
    typename Object::ConstPtr object_;
    PointMatrix<Types::Scalar, Dimension> points_;
    Types::Scalar points_std_dev_;

  };

} /* namespace calibration */
#endif /* CALIBRATON_COMMON_OBJECTS_VIEW_H_ */
