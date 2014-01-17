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

#ifndef CALIBRATION_COMMON_ALGORITHMS_PLANE_EXTRACTOR_H_
#define CALIBRATION_COMMON_ALGORITHMS_PLANE_EXTRACTOR_H_

#include <boost/make_shared.hpp>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/segmentation/extract_clusters.h>

#include <calibration_common/objects/globals.h>

namespace pcl_vis = pcl::visualization;
using pcl_vis::PointCloudColorHandlerGenericField;

namespace calibration
{

struct PlaneInfo
{
  Types::Plane equation_;
  pcl::IndicesPtr indices_;
  Types::Scalar std_dev_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename PointT>
  class PlaneExtractor
  {
  public:

    typedef boost::shared_ptr<PlaneExtractor> Ptr;
    typedef boost::shared_ptr<const PlaneExtractor> ConstPtr;

    typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

    virtual ~PlaneExtractor()
    {
    }

    virtual void setInputCloud(const PointCloudConstPtr & cloud) = 0;

    virtual bool extract(PlaneInfo & plane_info) const = 0;

  };

template <typename PointT>
  class PointPlaneExtractor : public PlaneExtractor<PointT>
  {
  public:

    struct PickingPoint
    {
      PickingPoint(PointT point,
                   Types::Scalar radius)
        : point_(point),
          radius_(radius)
      {
      }

      const PointT point_;
      const Types::Scalar radius_;
    };

    typedef boost::shared_ptr<PointPlaneExtractor> Ptr;
    typedef boost::shared_ptr<const PointPlaneExtractor> ConstPtr;

    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef pcl::search::KdTree<PointT> KdTree;
    typedef typename KdTree::Ptr KdTreePtr;
    typedef pcl::NormalEstimationOMP<PointT, pcl::Normal> NormalEstimator;
    typedef pcl::SampleConsensusModelNormalPlane<PointT, pcl::Normal> ModelNormalPlane;

    PointPlaneExtractor();

    virtual ~PointPlaneExtractor();

    void setRadius(Types::Scalar radius);

    virtual void setInputCloud(const PointCloudConstPtr & cloud);

    virtual void setPoint(const PointT & point);

    virtual void setPoint(const PickingPoint & picking_point);

    virtual bool extract(PlaneInfo & plane_info) const;

  protected:

    PointCloudConstPtr cloud_;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
    KdTreePtr cloud_tree_;

    PointT point_;

    Types::Scalar radius_;

  };

template <typename PointT>
  class PointPlaneExtractorGUI : public PointPlaneExtractor<PointT>
  {
  public:

    typedef boost::shared_ptr<PointPlaneExtractorGUI> Ptr;
    typedef boost::shared_ptr<const PointPlaneExtractorGUI> ConstPtr;

    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef pcl::PointCloud<pcl::PointXYZL> PointCloudLabel;
    typedef typename PointCloudLabel::Ptr PointCloudLabelPtr;

    typedef boost::shared_ptr<pcl_vis::PCLVisualizer> PCLVisualizerPtr;
    typedef PointCloudColorHandlerGenericField<pcl::PointXYZL> ColorHandler;
    typedef boost::shared_ptr<ColorHandler> ColorHandlerPtr;
    typedef boost::signals2::connection Connection;

    typedef PointPlaneExtractor<PointT> Base;

    PointPlaneExtractorGUI();

    virtual ~PointPlaneExtractorGUI();

    virtual bool extract(PlaneInfo & plane_info) const;

    virtual void setInputCloud(const PointCloudConstPtr & cloud);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:

    void pointPickingCallback(const pcl_vis::PointPickingEvent & event,
                              void * param);

    void keyboardCallback(const pcl_vis::KeyboardEvent & event,
                          void * param);

    mutable PCLVisualizerPtr viewer_;
    ColorHandlerPtr color_handler_;

    Connection k_connection_;
    Connection pp_connection_;

    PointCloudLabelPtr cloud_label_;
    std::string cloud_name_;

    mutable unsigned int current_index_;
    mutable bool next_plane_;

    mutable PlaneInfo last_plane_info_;
    mutable Types::Point3 last_clicked_point_;

  };

} /* namespace calibration */

#include <impl/calibration_common/algorithms/plane_extractor.hpp>

#endif /* CALIBRATION_COMMON_ALGORITHMS_PLANE_EXTRACTOR_H_ */
