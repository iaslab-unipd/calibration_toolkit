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

#ifndef IMPL_CALIBRATION_COMMON_ALGORITHMS_PLANE_EXTRACTOR_HPP_
#define IMPL_CALIBRATION_COMMON_ALGORITHMS_PLANE_EXTRACTOR_HPP_

#include <boost/smart_ptr/make_shared.hpp>

#include <calibration_common/algorithms/plane_extractor.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <pcl/impl/point_types.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/keyboard_event.h>

#include <vector>

namespace calibration
{

template <typename PointT_>
  PointPlaneExtractor<PointT_>::PointPlaneExtractor()
    : cloud_normals_(new pcl::PointCloud<pcl::Normal>()),
      cloud_tree_(new KdTree()),
      radius_(0.1)
  {
    // Do nothing
  }

template <typename PointT_>
  PointPlaneExtractor<PointT_>::~PointPlaneExtractor()
  {
    // Do nothing
  }

template <typename PointT_>
  void PointPlaneExtractor<PointT_>::setRadius(Scalar radius)
  {
    radius_ = radius;
  }

template <typename PointT_>
  void PointPlaneExtractor<PointT_>::setInputCloud(const PointCloudConstPtr & cloud)
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

template <typename PointT_>
  void PointPlaneExtractor<PointT_>::setPoint(const PointT_ & point)
  {
    point_ = point;
  }

template <typename PointT_>
  void PointPlaneExtractor<PointT_>::setPoint(const PickingPoint & picking_point)
  {
    setRadius(picking_point.radius_);
    setPoint(picking_point.point_);
  }

template <typename PointT_>
  bool PointPlaneExtractor<PointT_>::extract(PlaneInfo & plane_info) const
  {
    if (not plane_info.indices_)
      plane_info.indices_ = boost::make_shared<std::vector<int> >();
    else
      plane_info.indices_->clear();

    pcl::PointIndices init_indices;
    std::vector<float> sqr_distances;
    cloud_tree_->radiusSearch(point_, radius_, init_indices.indices, sqr_distances);

    if (init_indices.indices.size() < 50)
    {
      //ROS_ERROR_STREAM("Not enough points found near (" << point_.x << ", " << point_.y << ", " << point_.z << ")");
      return false;
    }

    //    Eigen::Vector4f eigen_centroid;
    //    pcl::compute3DCentroid(*cloud_, init_indices, eigen_centroid);
    //    PointT_ centroid(eigen_centroid[0], eigen_centroid[1], eigen_centroid[2]);
    //
    //    std::vector<int> nn_indices(1);
    //    std::vector<float> nn_sqr_distances(1);
    //    cloud_tree_->nearestKSearch(centroid, 1, nn_indices, nn_sqr_distances);
    //
    //    int nn_index = nn_indices[0];
    //    ROS_INFO_STREAM(
    //      "Seed point: (" << cloud_->points[nn_index].x << ", " << cloud_->points[nn_index].y << ", "
    //        << cloud_->points[nn_index].z << ")");

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
    plane_info.std_dev_ = 0.0;

    if (v.size() > 0)
      plane_info.std_dev_ = std::sqrt(v.cwiseAbs2().mean() - std::pow(v.mean(), 2));

    model.setIndices(all_cloud_indices);
    model.selectWithinDistance(coefficients, 5 * plane_info.std_dev_, *plane_info.indices_);

    model.optimizeModelCoefficients(*plane_info.indices_, coefficients, new_coefficients);
    if (coefficients == new_coefficients)
      return false;
    coefficients = new_coefficients;

    model.setNormalDistanceWeight(0.05);
    model.selectWithinDistance(coefficients, 5 * plane_info.std_dev_, *plane_info.indices_);

    //    pcl::RegionGrowing<PointT_, pcl::Normal> region_grow;
    //    region_grow.setMinClusterSize(cloud_->size() / 10);
    //    region_grow.setMaxClusterSize(cloud_->size());
    //
    //    region_grow.setSearchMethod(cloud_tree_);
    //    region_grow.setNumberOfNeighbours(64);
    //    region_grow.setInputCloud(cloud_);
    //    //region_grow.setIndices(plane_indices);
    //    region_grow.setInputNormals(cloud_normals_);
    //    region_grow.setSmoothnessThreshold(10.0 / 180.0 * M_PI);
    //    region_grow.setCurvatureThreshold(0.07);
    //
    //    pcl::PointIndices cluster;
    //    region_grow.getSegmentFromPoint(nn_index, cluster);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT_> ec;
    ec.setClusterTolerance(radius_ / 8);
    ec.setMinClusterSize(plane_info.indices_->size() / 8);
    ec.setMaxClusterSize(plane_info.indices_->size());
    //ec.setSearchMethod(cloud_tree_);
    ec.setInputCloud(cloud_);
    ec.setIndices(plane_info.indices_);
    ec.extract(cluster_indices);

    if (cluster_indices.empty())
      return false;

    size_t max_index = 0;
    size_t max_size = cluster_indices[0].indices.size();
    for (size_t i = 1; i < cluster_indices.size(); ++i)
    {
      if (cluster_indices[i].indices.size() > max_size)
      {
        max_size = cluster_indices[i].indices.size();
        max_index = i;
      }
    }

    *plane_info.indices_ = cluster_indices[max_index].indices;

    //    Eigen::VectorXf coefficients(4);
    //    ModelNormalPlane model(cloud_);
    //    model.setInputNormals(cloud_normals_);
    //    Eigen::VectorXf new_coefficients(4);
    //    model.optimizeModelCoefficients(cluster.indices, coefficients, new_coefficients);
    //    if (coefficients == new_coefficients)
    //      return false;
    //    coefficients = new_coefficients;
    //
    //    if (coefficients[3] > 0)
    //      coefficients *= -1.0f;
    //
    //    *plane_indices = cluster.indices;

    //Plane plane;
    plane_info.plane_.normal()[0] = coefficients[0];
    plane_info.plane_.normal()[1] = coefficients[1];
    plane_info.plane_.normal()[2] = coefficients[2];
    plane_info.plane_.offset() = coefficients[3];

    //planar_object = boost::make_shared<PointCloudPlanarObject<PointT_> >(cloud_, plane_indices, plane);

    return true;
  }

//template <typename PointT_>
//  bool SimpleCheckerboardPlaneExtractor<PointT_>::extract(const RGBCheckerboard &rgb_checkerboard,
//                                                         pcl::IndicesPtr &plane_indices,
//                                                         Plane &plane,
//                                                         double &std_dev)
//  {
//    Point3d center = rgb_to_d_transform_ * rgb_checkerboard.center();
//    PointT_ p;
//    p.x = center[0];
//    p.y = center[1];
//    p.z = center[2];
//    extractFromPoint(p, plane_indices, plane, std_dev);
//  }

template <typename PointT_>
  void PointPlaneExtractorGUI<PointT_>::setInputCloud(const PointCloudConstPtr & cloud)
  {
    cloud_label_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZL> >();

    cloud_label_->header = cloud->header;
    cloud_label_->width = cloud->width;
    cloud_label_->height = cloud->height;
    cloud_label_->is_dense = cloud->is_dense;
    cloud_label_->sensor_origin_ = cloud->sensor_origin_;
    cloud_label_->sensor_orientation_ = cloud->sensor_orientation_;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      pcl::PointXYZL p;
      p.x = cloud->points[i].x;
      p.y = cloud->points[i].y;
      p.z = cloud->points[i].z;
      p.label = 0;
      cloud_label_->push_back(p);
    }

    Base::setInputCloud(cloud);
    current_index_ = 1;

    color_handler_ = ColorHandlerPtr(new ColorHandler(cloud_label_, "label"));
    if (viewer_)
      if (not viewer_->updatePointCloud<pcl::PointXYZL>(cloud_label_, *color_handler_, cloud_name_))
        viewer_->addPointCloud<pcl::PointXYZL>(cloud_label_, *color_handler_, cloud_name_);

    viewer_->addPointCloudNormals<pcl::PointXYZL, pcl::Normal>(cloud_label_,
                                                               Base::cloud_normals_,
                                                               100,
                                                               0.1f,
                                                               "Normals");

  }

template <typename PointT_>
  PointPlaneExtractorGUI<PointT_>::PointPlaneExtractorGUI()
    : cloud_name_("Cloud"),
      current_index_(1),
      next_plane_(false),
      viewer_(boost::make_shared<pcl_vis::PCLVisualizer>("Select Checkerboard Planes")),
      k_connection_(viewer_->registerKeyboardCallback(&PointPlaneExtractorGUI::keyboardCallback, *this)),
      pp_connection_(viewer_->registerPointPickingCallback(&PointPlaneExtractorGUI::pointPickingCallback, *this))
  {
    // Do nothing
  }

template <typename PointT_>
  PointPlaneExtractorGUI<PointT_>::~PointPlaneExtractorGUI()
  {
    if (viewer_ and cloud_label_)
      viewer_->removePointCloud(cloud_name_);

    k_connection_.disconnect();
    pp_connection_.disconnect();
  }

template <typename PointT_>
  bool PointPlaneExtractorGUI<PointT_>::extract(PlaneInfo & plane_info) const
  {
    if (not cloud_label_)
      throw std::runtime_error("Need to call setInputCloud(...) method first.");

    next_plane_ = false;

    while (not viewer_->wasStopped() and not next_plane_)
      viewer_->spinOnce(50);

    viewer_->spinOnce();

    if (next_plane_)
    {
      plane_info = last_plane_info_;
      current_index_++;
    }

    return next_plane_;
  }

template <typename PointT_>
  void PointPlaneExtractorGUI<PointT_>::pointPickingCallback(const pcl_vis::PointPickingEvent & event,
                                                             void * param)
  {
    for (size_t i = 0; i < cloud_label_->points.size(); ++i)
    {
      if (cloud_label_->points[i].label == current_index_)
        cloud_label_->points[i].label = 0;
    }

    pcl::PointXYZ p;
    event.getPoint(p.x, p.y, p.z);
    last_clicked_point_ << p.x, p.y, p.z;

    PointPlaneExtractor<PointT_>::setPoint(p);
    PointPlaneExtractor<PointT_>::extract(last_plane_info_);

    const std::vector<int> & indices = *last_plane_info_.indices_;
    for (size_t i = 0; i < indices.size(); ++i)
    {
      if (cloud_label_->points[indices[i]].label == 0)
        cloud_label_->points[indices[i]].label = current_index_;
    }

    //cloud_->points[event.getPointIndex()].label = current_index_;

    viewer_->updatePointCloud<pcl::PointXYZL>(cloud_label_, *color_handler_, cloud_name_);

  }

template <typename PointT_>
  inline void PointPlaneExtractorGUI<PointT_>::keyboardCallback(const pcl_vis::KeyboardEvent & event,
                                                                void * param)
  {
    if (event.getKeySym() == "space" and event.keyUp())
      next_plane_ = true;
  }

} /* namespace calibration */
#endif /* IMPL_CALIBRATION_COMMON_ALGORITHMS_PLANE_EXTRACTOR_HPP_ */
