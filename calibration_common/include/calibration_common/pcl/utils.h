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

#ifndef CALIBRATION_COMMON_PCL_UTILS_H_
#define CALIBRATION_COMMON_PCL_UTILS_H_

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_ros/point_cloud.h>

namespace calibration
{

template <typename T>
  struct SensorDepthTraits
  {
  };

template <>
  struct SensorDepthTraits<uint16_t>
  {
    static inline bool valid(uint16_t depth)
    {
      return depth != 0;
    }
    static inline float toMeters(uint16_t depth)
    {
      return depth * 0.001f;
    } // originally mm
    static inline uint16_t fromMeters(float depth)
    {
      return (depth * 1000.0f) + 0.5f;
    }
    static inline void initializeBuffer(std::vector<uint8_t> & buffer)
    {
    } // Do nothing - already zero-filled
  };

template <>
  struct SensorDepthTraits<float>
  {
    static inline bool valid(float depth)
    {
      return std::isfinite(depth);
    }
    static inline float toMeters(float depth)
    {
      return depth;
    }
    static inline float fromMeters(float depth)
    {
      return depth;
    }

    static inline void initializeBuffer(std::vector<uint8_t> & buffer)
    {
      float * start = reinterpret_cast<float *>(&buffer[0]);
      float * end = reinterpret_cast<float *>(&buffer[0] + buffer.size());
      std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
    }
  };

template <>
  struct SensorDepthTraits<uint8_t>
  {
    static inline bool valid(uint8_t depth)
    {
      return depth != 0;
    }
    static inline float toMeters(uint8_t depth)
    {
      return depth * 0.001f;
    } // originally mm
    static inline uint8_t fromMeters(float depth)
    {
      return (depth * 1000.0f) + 0.5f;
    }
    static inline void initializeBuffer(std::vector<uint8_t> & buffer)
    {
    } // Do nothing - already zero-filled
  };

template <typename Scalar_, typename PCLPoint_>
  void convertToPointCloud(const sensor_msgs::Image & depth_msg,
                           const sensor_msgs::CameraInfo & info_msg,
                           typename pcl::PointCloud<PCLPoint_>::Ptr & cloud);

} /* namespace calibration */

#include <impl/calibration_common/pcl/utils.hpp>

#endif /* CALIBRATION_COMMON_PCL_UTILS_H_ */
