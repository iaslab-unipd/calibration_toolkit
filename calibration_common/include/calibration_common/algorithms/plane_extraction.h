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

#include <boost/signals2/connection.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

#include <calibration_common/objects/globals.h>

#include <Eigen/src/Core/util/Memory.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_picking_event.h>

#include <string>

namespace pcl_vis = pcl::visualization;
using pcl_vis::PointCloudColorHandlerGenericField;

namespace calibration
{

/**
 * @brief Information about a plane extracted from a point cloud.
 */
struct PlaneInfo
{
  Plane plane_;             ///< The plane equation.
  pcl::IndicesPtr indices_; ///< The indices of the points belong to the plane.
  Scalar std_dev_;          ///< The standard deviation of the distance between the points and the plane.

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief Base class for the extraction of a well defined plane from a point cloud.
 * @param PCLPoint_ The point
 */
template <typename PCLPointT_>
  class PlaneExtraction
  {
  public:

    typedef boost::shared_ptr<PlaneExtraction> Ptr;
    typedef boost::shared_ptr<const PlaneExtraction> ConstPtr;

    typedef typename pcl::PointCloud<PCLPointT_>::ConstPtr PointCloudConstPtr;

    virtual ~PlaneExtraction()
    {
    }

    /**
     * @brief setInputCloud Provide a pointer to the input dataset.
     * @param [in] cloud The const boost shared pointer to a PointCloud message.
     */
    virtual void setInputCloud(const PointCloudConstPtr & cloud) = 0;

    /**
     * @brief Extract the plane.
     * @param [out] plane_info The PlaneInfo of the extracted plane.
     * @return @c true if the plane is extracted from the point cloud, @c false otherwise.
     */
    virtual bool extract(PlaneInfo & plane_info) const = 0;

  };

template <typename PCLPointT_>
  class CheckerboardPlaneExtraction : public PlaneExtraction<PCLPointT_>
  {

    typedef boost::shared_ptr<CheckerboardPlaneExtraction> Ptr;
    typedef boost::shared_ptr<const CheckerboardPlaneExtraction> ConstPtr;

    typedef pcl::PointCloud<PCLPointT_> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef pcl::search::KdTree<PCLPointT_> KdTree;
    typedef typename KdTree::Ptr KdTreePtr;
    typedef pcl::NormalEstimationOMP<PCLPointT_, pcl::Normal> NormalEstimator;
    typedef pcl::SampleConsensusModelNormalPlane<PCLPointT_, pcl::Normal> ModelNormalPlane;

    /**
     * @brief PointPlaneExtractor
     */
    CheckerboardPlaneExtraction()
      : cloud_normals_(boost::make_shared<pcl::PointCloud<pcl::Normal> >()),
        cloud_tree_(boost::make_shared<KdTree>()),
        corners_(Eigen::Matrix<float, 2, 4>::Zero())
    {
      // Do nothing
    }

    /**
     * @brief ~PointPlaneExtractor
     */
    virtual ~CheckerboardPlaneExtraction()
    {
      // Do nothing
    }

    /**
     * @brief setVertices
     * @param corners
     */
    inline void setVertices(const Eigen::Matrix<float, 2, 4> & corners)
    {
      corners_ = corners;
    }

    /**
     * @brief setInputCloud
     * @param cloud
     */
    virtual void setInputCloud(const PointCloudConstPtr & cloud);

    /**
     * @brief extract
     * @param plane_info
     * @return
     */
    virtual bool extract(PlaneInfo & plane_info) const;

  protected:

    PointCloudConstPtr cloud_;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
    KdTreePtr cloud_tree_;

    Eigen::Matrix<float, 2, 4> corners_;

  };

/**
 * @brief The PointPlaneExtractor class
 * @param PCLPointT_
 */
template <typename PCLPointT_>
  class PointPlaneExtraction : public PlaneExtraction<PCLPointT_>
  {
  public:

    /**
     * @brief The PickingPoint struct
     */
    struct PickingPoint
    {
      PickingPoint(PCLPointT_ point,
                   Scalar radius)
        : point_(point),
          radius_(radius)
      {
      }

      const PCLPointT_ point_;
      const Scalar radius_;
    };

    typedef boost::shared_ptr<PointPlaneExtraction> Ptr;
    typedef boost::shared_ptr<const PointPlaneExtraction> ConstPtr;

    typedef pcl::PointCloud<PCLPointT_> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef pcl::search::KdTree<PCLPointT_> KdTree;
    typedef typename KdTree::Ptr KdTreePtr;
    typedef pcl::NormalEstimationOMP<PCLPointT_, pcl::Normal> NormalEstimator;
    typedef pcl::SampleConsensusModelNormalParallelPlane<PCLPointT_, pcl::Normal> ModelNormalPlane;

    /**
     * @brief PointPlaneExtractor
     */
    PointPlaneExtraction()
      : cloud_normals_(boost::make_shared<pcl::PointCloud<pcl::Normal> >()),
        cloud_tree_(boost::make_shared<KdTree>()),
        radius_(0.1)
    {
      // Do nothing
    }

    /**
     * @brief ~PointPlaneExtractor
     */
    virtual ~PointPlaneExtraction()
    {
      // Do nothing
    }

    /**
     * @brief setRadius
     * @param radius
     */
    inline void setRadius(Scalar radius)
    {
      radius_ = radius;
    }

    /**
     * @brief setInputCloud
     * @param cloud
     */
    virtual void setInputCloud(const PointCloudConstPtr & cloud);

    /**
     * @brief setPoint
     * @param point
     */
    inline virtual void setPoint(const PCLPointT_ & point)
    {
      point_ = point;
    }

    /**
     * @brief setPoint
     * @param picking_point
     */
    inline virtual void setPoint(const PickingPoint & picking_point)
    {
      setRadius(picking_point.radius_);
      setPoint(picking_point.point_);
    }

    /**
     * @brief extract
     * @param plane_info
     * @return
     */
    virtual bool extract(PlaneInfo & plane_info) const;

    inline virtual pcl::PointCloud<pcl::Normal>::ConstPtr cloudNormals () const
    {
      return cloud_normals_;
    }

  protected:

    PointCloudConstPtr cloud_;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
    KdTreePtr cloud_tree_;

    PCLPointT_ point_;

    Scalar radius_;

  };

/**
 * @brief The PointPlaneExtractorGUI class
 * @param PCLPointT_
 */
template <typename PCLPointT_>
  class PointPlaneExtractionGUI : public PointPlaneExtraction<PCLPointT_>
  {
  public:

    typedef boost::shared_ptr<PointPlaneExtractionGUI> Ptr;
    typedef boost::shared_ptr<const PointPlaneExtractionGUI> ConstPtr;

    typedef pcl::PointCloud<PCLPointT_> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef pcl::PointCloud<pcl::PointXYZL> PointCloudLabel;
    typedef typename PointCloudLabel::Ptr PointCloudLabelPtr;

    typedef boost::shared_ptr<pcl_vis::PCLVisualizer> PCLVisualizerPtr;
    typedef PointCloudColorHandlerGenericField<pcl::PointXYZL> ColorHandler;
    typedef boost::shared_ptr<ColorHandler> ColorHandlerPtr;
    typedef boost::signals2::connection Connection;

    typedef PointPlaneExtraction<PCLPointT_> Base;

    /**
     * @brief PointPlaneExtractorGUI
     */
    PointPlaneExtractionGUI();

    /**
     * @brief ~PointPlaneExtractorGUI
     */
    virtual ~PointPlaneExtractionGUI();

    /**
     * @brief extract
     * @param plane_info
     * @return
     */
    virtual bool extract(PlaneInfo & plane_info) const;

    /**
     * @brief setInputCloud
     * @param cloud
     */
    virtual void setInputCloud(const PointCloudConstPtr & cloud);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:

    /**
     * @brief pointPickingCallback
     * @param event
     * @param param
     */
    void pointPickingCallback(const pcl_vis::PointPickingEvent & event,
                              void * param);

    /**
     * @brief keyboardCallback
     * @param event
     * @param param
     */
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
    mutable Point3 last_clicked_point_;

  };

} /* namespace calibration */

#include <impl/calibration_common/algorithms/plane_extraction.hpp>

#endif /* CALIBRATION_COMMON_ALGORITHMS_PLANE_EXTRACTOR_H_ */
