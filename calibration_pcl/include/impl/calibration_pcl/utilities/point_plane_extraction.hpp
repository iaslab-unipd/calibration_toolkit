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

#ifndef UNIPD_CALIBRATION_IMPL_CALIBRATION_PCL_UTILITIES_POINT_PLANE_EXTRACTION_HPP_
#define UNIPD_CALIBRATION_IMPL_CALIBRATION_PCL_UTILITIES_POINT_PLANE_EXTRACTION_HPP_

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <pcl/impl/point_types.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/keyboard_event.h>

#include <vector>

#include <calibration_pcl/utilities/point_plane_extraction.h>

namespace unipd
{
namespace calib
{

template <typename PCLPointT_>
  void PointPlaneExtraction<PCLPointT_>::setInputCloud(const PointCloudConstPtr & cloud)
  {
    cloud_ = cloud;
    cloud_tree_->setInputCloud(cloud_);

    NormalEstimator normal_estimator;
    normal_estimator.setInputCloud(cloud_);
    normal_estimator.setSearchMethod(cloud_tree_);
    normal_estimator.setKSearch(100); // TODO setRadiusSearch
    //normal_estimator.setRadiusSearch(0.05f);
    normal_estimator.compute(*cloud_normals_);
  }

template <typename PCLPointT_>
  bool PointPlaneExtraction<PCLPointT_>::extract(PlaneInfo & plane_info) const
  {
    if (not plane_info.indices)
      plane_info.indices = boost::make_shared<Indices1>();
    else
      plane_info.indices->clear();

    pcl::PointIndices init_indices;
    std::vector<float> sqr_distances;
    cloud_tree_->radiusSearch(point_, radius_, init_indices.indices, sqr_distances);

    if (init_indices.indices.size() < 50)
    {
      //ROS_ERROR_STREAM("Not enough points found near (" << point_.x << ", " << point_.y << ", " << point_.z << ")");
      return false;
    }

    Eigen::VectorXf coefficients(Eigen::VectorXf::Zero(4));
    ModelNormalPlane model(cloud_);
    model.setNormalDistanceWeight(0.2);
    model.setInputNormals(cloud_normals_);
    Eigen::VectorXf new_coefficients(4);
    model.optimizeModelCoefficients(init_indices.indices, coefficients, new_coefficients);
    if (coefficients == new_coefficients)
      return false;
    coefficients = new_coefficients;

    boost::shared_ptr<std::vector<int> > all_cloud_indices = model.getIndices();

    model.setIndices(init_indices.indices);
    std::vector<Scalar> distances;
    model.getDistancesToModel(coefficients, distances);
    Eigen::VectorXd v = Eigen::VectorXd::Map(&distances[0], distances.size());
    plane_info.std_dev = 0.0;

    if (v.size() > 0)
      plane_info.std_dev = std::sqrt(v.cwiseAbs2().mean() - std::pow(v.mean(), 2));

    model.setIndices(all_cloud_indices);
    model.selectWithinDistance(coefficients, 5 * plane_info.std_dev, *plane_info.indices);

    model.optimizeModelCoefficients(*plane_info.indices, coefficients, new_coefficients);
    if (coefficients == new_coefficients)
      return false;
    coefficients = new_coefficients;

    model.setNormalDistanceWeight(0.05);
    model.selectWithinDistance(coefficients, 5 * plane_info.std_dev, *plane_info.indices);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PCLPointT_> ec;
    ec.setClusterTolerance(radius_ / 8);
    ec.setMinClusterSize(plane_info.indices->size() / 8);
    ec.setMaxClusterSize(plane_info.indices->size());
    //ec.setSearchMethod(cloud_tree_);
    ec.setInputCloud(cloud_);
    ec.setIndices(plane_info.indices);
    ec.extract(cluster_indices);

    if (cluster_indices.empty())
      return false;

    Size1 max_index = 0;
    Size1 max_size = cluster_indices[0].indices.size();
    for (Size1 i = 1; i < cluster_indices.size(); ++i)
    {
      if (cluster_indices[i].indices.size() > max_size)
      {
        max_size = cluster_indices[i].indices.size();
        max_index = i;
      }
    }

    *plane_info.indices = cluster_indices[max_index].indices;

    plane_info.plane.normal()[0] = coefficients[0];
    plane_info.plane.normal()[1] = coefficients[1];
    plane_info.plane.normal()[2] = coefficients[2];
    plane_info.plane.offset() = coefficients[3];

    return true;
  }

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_sIMPL_CALIBRATION_PCL_UTILITIES_POINT_PLANE_EXTRACTION_HPP_
