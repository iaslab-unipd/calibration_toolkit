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

#include <pcl/sample_consensus/ransac.h>

#include <calibration_common/algorithms/plane_extraction.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <pcl/impl/point_types.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/keyboard_event.h>

#include <vector>

#include <pcl/visualization/pcl_visualizer.h>

namespace calibration
{

template <typename PointT_>
  void CheckerboardPlaneExtraction<PointT_>::setInputCloud(const PointCloudConstPtr & cloud)
  {
    cloud_ = cloud;
    cloud_tree_->setInputCloud(cloud_);

    NormalEstimator normal_estimator;
    normal_estimator.setInputCloud(cloud_);
    normal_estimator.setSearchMethod(cloud_tree_);
    if (cloud->width > 320)
      normal_estimator.setKSearch(80);
    else if (cloud->width > 160)
      normal_estimator.setKSearch(50);
    else
      normal_estimator.setKSearch(30);

    normal_estimator.compute(*cloud_normals_);
  }

template <typename PointT_>
  bool CheckerboardPlaneExtraction<PointT_>::extract(PlaneInfo & plane_info) const
  {

    /*float min_x = corners_.col(0).x(), max_x = min_x;
    float min_y = corners_.col(0).y(), max_y = min_y;
    for (int i = 1; i < 4; ++i)
    {
      min_x = std::min(min_x, corners_.col(i).x());
      max_x = std::max(max_x, corners_.col(i).x());
      min_y = std::min(min_y, corners_.col(i).y());
      max_y = std::max(max_y, corners_.col(i).y());
    }

    min_x = std::max(min_x, 0.0f);
    max_x = std::min(max_x, static_cast<float>(cloud_->width));
    min_y = std::max(min_y, 0.0f);
    max_y = std::min(max_y, static_cast<float>(cloud_->height));

    Eigen::ParametrizedLine<float, 2> lines[4];
    Eigen::Array4f multipliers;
    for (int i = 0; i < 4; ++i)
    {
      lines[i] = Eigen::ParametrizedLine<float, 2>::Through(corners_.col((i + 1) % 4), corners_.col(i));
      multipliers[i] = (lines[i].signedDistance(corners_.col((i + 2) % 4)) > 0) ? 1.0f : -1.0f;
    }



    if (not plane_info.indices_)
      plane_info.indices_ = boost::make_shared<std::vector<int> >();
    else
      plane_info.indices_->clear();

    plane_info.indices_->reserve(static_cast<size_t>((max_x - min_x) * (max_y - min_y)));

    for (float y = std::ceil(min_y); y < max_y; ++y)
    {
      for (float x = std::ceil(min_x); x < max_x; ++x)
      {
        bool ok = true;
        for (int i = 0; i < 4; ++i)
        {
          if (lines[i].signedDistance(Eigen::Vector2f(x, y)) * multipliers[i] < 0)
          {
            ok = false;
            break;
          }
        }
        if (ok)
          plane_info.indices_->push_back(static_cast<int>(y * cloud_->width + x));
      }
    }

    int indices_size = plane_info.indices_->size();
    std::cout << "indices_size: " << indices_size << std::endl;


    Eigen::VectorXf coefficients(Eigen::VectorXf::Zero(4));
    ModelNormalPlane model(cloud_);
    Eigen::VectorXf new_coefficients(4);
    model.optimizeModelCoefficients(*plane_info.indices_, coefficients, new_coefficients);
    if (coefficients == new_coefficients)
      return false;
    coefficients = new_coefficients;


    std::vector<Scalar> distances;
    model.getDistancesToModel(coefficients, distances);
    Eigen::Map<Eigen::VectorXd> v = Eigen::Map<Eigen::VectorXd>(&distances[0], distances.size());
    plane_info.std_dev_ = std::sqrt(v.cwiseAbs2().mean() - std::pow(v.mean(), 2));

    std::cout << "plane_info.std_dev_: " << plane_info.std_dev_ << std::endl;

    model.setIndices(plane_info.indices_);
    pcl::RandomSampleConsensus<PointT_> ransac(model);
    ransac.setDistanceThreshold(3 * plane_info.std_dev_);
    ransac.computeModel();
    ransac.getModelCoefficients(coefficients);
    ransac.getInliers(*plane_info.indices_);


    indices_size = plane_info.indices_->size();
    std::cout << "indices_size: " << indices_size << std::endl;


    model.setNormalDistanceWeight(0.2);
    model.setInputNormals(cloud_normals_);

    model.getDistancesToModel(coefficients, distances);
    v = Eigen::Map<Eigen::VectorXd>(&distances[0], distances.size());
    double std_dev_normal = std::sqrt(v.cwiseAbs2().mean() - std::pow(v.mean(), 2));

    std::cout << "std_dev_normal: " << std_dev_normal << std::endl;
    double new_threshold = (1 - model.getNormalDistanceWeight()) * 3 * plane_info.std_dev_
                           + model.getNormalDistanceWeight() * 10.0 * M_PI / 180.0;
    std::cout << "std_dev_normal: " << new_threshold << std::endl;

    model.setIndices(boost::make_shared<std::vector<int> >());
    ransac.setDistanceThreshold(new_threshold);
    ransac.computeModel();
    ransac.getModelCoefficients(coefficients);
    ransac.getInliers(*plane_info.indices_);


    indices_size = plane_info.indices_->size();
    std::cout << "indices_size: " << indices_size << std::endl;

    plane_info.plane_.normal()[0] = coefficients[0];
    plane_info.plane_.normal()[1] = coefficients[1];
    plane_info.plane_.normal()[2] = coefficients[2];
    plane_info.plane_.offset() = coefficients[3];


    std::cout << "-------" << std::endl;*/
  }




template <typename PointT_>
  void PointPlaneExtraction<PointT_>::setInputCloud(const PointCloudConstPtr & cloud)
  {
    cloud_ = cloud;
    cloud_tree_->setInputCloud(cloud_);

    NormalEstimator normal_estimator;
    normal_estimator.setInputCloud(cloud_);
    normal_estimator.setSearchMethod(cloud_tree_);
    if (cloud->width > 320)
      normal_estimator.setKSearch(512);
    else if (cloud->width > 160)
      normal_estimator.setKSearch(144);
    else
      normal_estimator.setKSearch(100);

    normal_estimator.compute(*cloud_normals_);
  }

template <typename PointT_>
  bool PointPlaneExtraction<PointT_>::extract(PlaneInfo & plane_info) const
  {
    if (not plane_info.indices_)
      plane_info.indices_ = boost::make_shared<std::vector<int> >();
    else
      plane_info.indices_->clear();

    std::vector<int> all_indices(cloud_->size());
    for (size_t i = 0; i < cloud_->size(); ++i)
      all_indices[i] = i;

    std::vector<float> sqr_distances;
    cloud_tree_->radiusSearch(point_, radius_, *plane_info.indices_, sqr_distances);

    if (plane_info.indices_->size() < 50)
    {
      //ROS_ERROR_STREAM("Not enough points found near (" << point_.x << ", " << point_.y << ", " << point_.z << ")");
      return false;
    }

    Eigen::VectorXf coefficients(Eigen::VectorXf::Zero(4));
    boost::shared_ptr<ModelNormalPlane> model = boost::make_shared<ModelNormalPlane>(cloud_);
    model->setInputNormals(cloud_normals_);
    model->setIndices(plane_info.indices_);

    model->setNormalDistanceWeight(0.0);

    Eigen::VectorXf new_coefficients(4);
    model->optimizeModelCoefficients(*plane_info.indices_, coefficients, new_coefficients);
    if (coefficients == new_coefficients)
      return false;
    coefficients = new_coefficients;


    std::vector<double> distances;
    model->getDistancesToModel(coefficients, distances);

    Eigen::Map<Eigen::VectorXd> v = Eigen::Map<Eigen::VectorXd>(&distances[0], distances.size());
    double v_mean = v.mean();
    plane_info.std_dev_ = std::sqrt(v.cwiseAbs2().mean() - v_mean * v_mean);

//    std::cout << "plane_info.std_dev_: " << plane_info.std_dev_ << std::endl;

    model->setIndices(all_indices);
    model->setAxis(Eigen::Vector3f(coefficients[0], coefficients[1], coefficients[2]));
    model->setEpsAngle(10.0);
    pcl::RandomSampleConsensus<PointT_> ransac(model);
    ransac.setDistanceThreshold(/*std::max(0.15, */10 * plane_info.std_dev_/*)*/);
    ransac.computeModel();
    ransac.getModelCoefficients(coefficients);
    ransac.getInliers(*plane_info.indices_);

    model->setIndices(plane_info.indices_);
    distances.clear();
    model->getDistancesToModel(coefficients, distances);

    Eigen::Map<Eigen::VectorXd> v2 = Eigen::Map<Eigen::VectorXd>(&distances[0], distances.size());
    double v2_mean = v2.mean();
    plane_info.std_dev_ = std::sqrt(v2.cwiseAbs2().mean() - v2_mean * v2_mean);

    model->setIndices(all_indices);
    //model->setAxis(Eigen::Vector3f(coefficients[0], coefficients[1], coefficients[2]));
    //model->setEpsAngle(10.0);

    //std::cout << "plane_info.std_dev_: " << plane_info.std_dev_ << std::endl;

    ransac = pcl::RandomSampleConsensus<PointT_>(model);
    ransac.setDistanceThreshold(/*std::max(0.15, */6 * plane_info.std_dev_/*)*/);
    ransac.computeModel();
    ransac.getModelCoefficients(coefficients);
    ransac.getInliers(*plane_info.indices_);

    //std::cout << "plane_info.std_dev_: " << plane_info.std_dev_ << std::endl;

    /*model->setNormalDistanceWeight(1.0);
    model->setIndices(plane_info.indices_);
    ransac.setDistanceThreshold(model->getNormalDistanceWeight() * 45.0 * M_PI / 180.0);
    ransac.computeModel();
    ransac.getModelCoefficients(coefficients);
    ransac.getInliers(*plane_info.indices_);*/


//    int indices_size = plane_info.indices_->size();
//    std::cout << "C indices_size: " << indices_size << std::endl;

    plane_info.plane_.normal()[0] = coefficients[0];
    plane_info.plane_.normal()[1] = coefficients[1];
    plane_info.plane_.normal()[2] = coefficients[2];
    plane_info.plane_.offset() = coefficients[3];

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
  void PointPlaneExtractionGUI<PointT_>::setInputCloud(const PointCloudConstPtr & cloud)
  {
    cloud_label_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZL> >();

    cloud_label_->header = cloud->header;
    cloud_label_->width = cloud->width;
    cloud_label_->height = cloud->height;
    cloud_label_->is_dense = cloud->is_dense;
    cloud_label_->sensor_origin_ = cloud->sensor_origin_;
    cloud_label_->sensor_orientation_ = cloud->sensor_orientation_;

    for (Size1 i = 0; i < cloud->points.size(); ++i)
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
  PointPlaneExtractionGUI<PointT_>::PointPlaneExtractionGUI()
    : cloud_name_("Cloud"),
      current_index_(1),
      next_plane_(false),
      viewer_(boost::make_shared<pcl_vis::PCLVisualizer>("Select Checkerboard Planes")),
      k_connection_(viewer_->registerKeyboardCallback(&PointPlaneExtractionGUI::keyboardCallback, *this)),
      pp_connection_(viewer_->registerPointPickingCallback(&PointPlaneExtractionGUI::pointPickingCallback, *this))
  {
    // Do nothing
  }

template <typename PointT_>
  PointPlaneExtractionGUI<PointT_>::~PointPlaneExtractionGUI()
  {
    if (viewer_ and cloud_label_)
      viewer_->removePointCloud(cloud_name_);

    k_connection_.disconnect();
    pp_connection_.disconnect();
  }

template <typename PointT_>
  bool PointPlaneExtractionGUI<PointT_>::extract(PlaneInfo & plane_info) const
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
  void PointPlaneExtractionGUI<PointT_>::pointPickingCallback(const pcl_vis::PointPickingEvent & event,
                                                             void * param)
  {
    for (Size1 i = 0; i < cloud_label_->points.size(); ++i)
    {
      if (cloud_label_->points[i].label == current_index_)
        cloud_label_->points[i].label = 0;
    }

    pcl::PointXYZ p;
    event.getPoint(p.x, p.y, p.z);
    last_clicked_point_ << p.x, p.y, p.z;

    PointPlaneExtraction<PointT_>::setPoint(p);
    PointPlaneExtraction<PointT_>::extract(last_plane_info_);

    const std::vector<int> & indices = *last_plane_info_.indices_;
    for (Size1 i = 0; i < indices.size(); ++i)
    {
      if (cloud_label_->points[indices[i]].label == 0)
        cloud_label_->points[indices[i]].label = current_index_;
    }

    //cloud_->points[event.getPointIndex()].label = current_index_;

    viewer_->updatePointCloud<pcl::PointXYZL>(cloud_label_, *color_handler_, cloud_name_);

  }

template <typename PointT_>
  inline void PointPlaneExtractionGUI<PointT_>::keyboardCallback(const pcl_vis::KeyboardEvent & event,
                                                                void * param)
  {
    if (event.getKeySym() == "space" and event.keyUp())
      next_plane_ = true;
  }

} /* namespace calibration */
#endif /* IMPL_CALIBRATION_COMMON_ALGORITHMS_PLANE_EXTRACTOR_HPP_ */
