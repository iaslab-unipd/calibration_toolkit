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

#ifndef CALIBRATION_COMMON_OBJECTS_SENSOR_H_
#define CALIBRATION_COMMON_OBJECTS_SENSOR_H_

#include <calibration_common/objects/base_object.h>

namespace calibration
{

/**
 * @brief The Sensor class
 */
class Sensor : public BaseObject
{
public:

  typedef boost::shared_ptr<Sensor> Ptr;
  typedef boost::shared_ptr<const Sensor> ConstPtr;

};

}
/* namespace calibration */
#endif /* CALIBRATION_COMMON_OBJECTS_SENSOR_H_ */
