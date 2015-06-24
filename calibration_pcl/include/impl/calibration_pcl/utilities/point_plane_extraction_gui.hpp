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

#ifndef UNIPD_CALIBRATION_IMPL_CALIBRATION_PCL_UTILITIES_POINT_PLANE_EXTRACTION_GUI_HPP_
#define UNIPD_CALIBRATION_IMPL_CALIBRATION_PCL_UTILITIES_POINT_PLANE_EXTRACTION_GUI_HPP_

#include <pcl/impl/point_types.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/keyboard_event.h>

#include <calibration_pcl/utilities/point_plane_extraction_gui.h>

namespace unipd
{
namespace calib
{

template <typename PCLPointT_>
  void PointPlaneExtractionGUI<PCLPointT_>::setInputCloud(const PointCloudConstPtr & cloud)
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

    viewer_->addPointCloudNormals<pcl::PointXYZL, pcl::Normal>(cloud_label_, Base::cloud_normals_, 100, 0.1f, "Normals");

  }

template <typename PCLPointT_>
  PointPlaneExtractionGUI<PCLPointT_>::PointPlaneExtractionGUI()
    : cloud_name_("Cloud"),
      current_index_(1),
      next_plane_(false),
      viewer_(boost::make_shared<pcl_vis::PCLVisualizer>("Select Checkerboard Planes")),
      k_connection_(viewer_->registerKeyboardCallback(&PointPlaneExtractionGUI::keyboardCallback, *this)),
      pp_connection_(viewer_->registerPointPickingCallback(&PointPlaneExtractionGUI::pointPickingCallback, *this))
  {
    // Do nothing
  }

template <typename PCLPointT_>
  PointPlaneExtractionGUI<PCLPointT_>::~PointPlaneExtractionGUI()
  {
    if (viewer_ and cloud_label_)
      viewer_->removePointCloud(cloud_name_);

    k_connection_.disconnect();
    pp_connection_.disconnect();
  }

template <typename PCLPointT_>
  bool PointPlaneExtractionGUI<PCLPointT_>::extract(PlaneInfo & plane_info) const
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

template <typename PCLPointT_>
  void PointPlaneExtractionGUI<PCLPointT_>::pointPickingCallback(const pcl_vis::PointPickingEvent & event,
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

    PointPlaneExtraction<PCLPointT_>::setPoint(p);
    PointPlaneExtraction<PCLPointT_>::extract(last_plane_info_);

    const std::vector<int> & indices = *last_plane_info_.indices;
    for (Size1 i = 0; i < indices.size(); ++i)
    {
      if (cloud_label_->points[indices[i]].label == 0)
        cloud_label_->points[indices[i]].label = current_index_;
    }

    //cloud_->points[event.getPointIndex()].label = current_index_;

    viewer_->updatePointCloud<pcl::PointXYZL>(cloud_label_, *color_handler_, cloud_name_);

  }

template <typename PCLPointT_>
  inline void PointPlaneExtractionGUI<PCLPointT_>::keyboardCallback(const pcl_vis::KeyboardEvent & event,
                                                                void * param)
  {
    if (event.getKeySym() == "space" and event.keyUp())
      next_plane_ = true;
  }

} // namespace calib
} // namespace unipd
#endif // UNIPD_CALIBRATION_IMPL_CALIBRATION_PCL_UTILITIES_POINT_PLANE_EXTRACTION_GUI_HPP_
