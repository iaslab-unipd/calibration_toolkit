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

#ifndef UNIPD_CALIBRATION_CALIBRATION_PCL_UTILITIES_PLANE_EXTRACTION_H_
#define UNIPD_CALIBRATION_CALIBRATION_PCL_UTILITIES_PLANE_EXTRACTION_H_

#include <calibration_common/base/geometry.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace unipd
{
namespace calib
{

/**
 * @brief Information about a plane extracted from a point cloud.
 */
struct PlaneInfo
{
  Plane3 plane;                         ///< The plane equation.
  boost::shared_ptr<Indices1> indices;  ///< The indices of the points belong to the plane.
  Scalar std_dev;                       ///< The standard deviation of the distance between the points and the plane.
};

/**
 * @brief Base class for the extraction of a well defined plane from a point cloud.
 * @param PCLPointT_ The point
 */
template <typename PCLPointT_>
  class PlaneExtraction
  {
  public:

    typedef pcl::PointCloud<PCLPointT_> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    virtual
    ~PlaneExtraction()
    {
      // Do nothing
    }

    /**
     * @brief setInputCloud Provide a pointer to the input dataset.
     * @param[in] cloud The const boost shared pointer to a PointCloud message.
     */
    virtual void
    setInputCloud (const PointCloudConstPtr & cloud) = 0;

    /**
     * @brief Extract the plane.
     * @param[out] plane_info The PlaneInfo of the extracted plane.
     * @return @c true if the plane is extracted from the point cloud, @c false otherwise.
     */
    virtual bool
    extract (PlaneInfo & plane_info) const = 0;

  };

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_CALIBRATION_PCL_UTILITIES_PLANE_EXTRACTION_H_
