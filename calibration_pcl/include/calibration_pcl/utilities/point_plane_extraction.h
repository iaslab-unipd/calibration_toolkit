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

#ifndef UNIPD_CALIBRATION_CALIBRATION_PCL_UTILITIES_POINT_PLANE_EXTRACTION_H_
#define UNIPD_CALIBRATION_CALIBRATION_PCL_UTILITIES_POINT_PLANE_EXTRACTION_H_

#include <boost/make_shared.hpp>

#include <calibration_pcl/utilities/plane_extraction.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/search/kdtree.h>

namespace unipd
{
namespace calib
{

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
      const PCLPointT_ point;
      const Scalar radius;
    };

    typedef pcl::PointCloud<PCLPointT_> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef pcl::search::KdTree<PCLPointT_> KdTree;
    typedef typename KdTree::Ptr KdTreePtr;
    typedef pcl::NormalEstimationOMP<PCLPointT_, pcl::Normal> NormalEstimator;
    typedef pcl::SampleConsensusModelNormalPlane<PCLPointT_, pcl::Normal> ModelNormalPlane;

    PointPlaneExtraction ()
      : cloud_normals_(boost::make_shared<pcl::PointCloud<pcl::Normal> >()),
        cloud_tree_(boost::make_shared<KdTree>()),
        radius_(0.1)
    {
      // Do nothing
    }

    virtual
    ~PointPlaneExtraction ()
    {
      // Do nothing
    }

    /**
     * @brief setRadius
     * @param radius
     */
    inline void
    setRadius (Scalar radius)
    {
      radius_ = radius;
    }

    /**
     * @brief setInputCloud
     * @param cloud
     */
    virtual void
    setInputCloud (const PointCloudConstPtr & cloud);

    /**
     * @brief setPoint
     * @param point
     */
    inline virtual void
    setPoint (const PCLPointT_ & point)
    {
      point_ = point;
    }

    /**
     * @brief setPoint
     * @param picking_point
     */
    inline virtual void
    setPoint (const PickingPoint & picking_point)
    {
      setRadius(picking_point.radius);
      setPoint(picking_point.point);
    }

    /**
     * @brief extract
     * @param plane_info
     * @return
     */
    virtual bool
    extract (PlaneInfo & plane_info) const;

  protected:

    PointCloudConstPtr cloud_;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
    KdTreePtr cloud_tree_;

    PCLPointT_ point_;

    Scalar radius_;

  };

} // namespace calibration
} // namespace unipd

#include <impl/calibration_pcl/utilities/point_plane_extraction.hpp>

#endif // UNIPD_CALIBRATION_CALIBRATION_PCL_UTILITIES_POINT_PLANE_EXTRACTION_H_
