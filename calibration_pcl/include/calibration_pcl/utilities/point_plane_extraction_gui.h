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

#ifndef UNIPD_CALIBRATION_CALIBRATION_PCL_UTILITIES_POINT_PLANE_EXTRACTION_GUI_H_
#define UNIPD_CALIBRATION_CALIBRATION_PCL_UTILITIES_POINT_PLANE_EXTRACTION_GUI_H_

#include <boost/signals2/connection.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_picking_event.h>

#include <calibration_pcl/utilities/point_plane_extraction.h>

namespace pcl_vis = pcl::visualization;
using pcl_vis::PointCloudColorHandlerGenericField;

namespace unipd
{
namespace calib
{

/**
 * @brief The PointPlaneExtractorGUI class
 * @param PCLPointT_
 */
template <typename PCLPointT_>
  class PointPlaneExtractionGUI : public PointPlaneExtraction<PCLPointT_>
  {
  public:

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
    PointPlaneExtractionGUI ();

    /**
     * @brief ~PointPlaneExtractorGUI
     */
    virtual
    ~PointPlaneExtractionGUI ();

    /**
     * @brief extract
     * @param plane_info
     * @return
     */
    virtual bool
    extract (PlaneInfo & plane_info) const;

    /**
     * @brief setInputCloud
     * @param cloud
     */
    virtual void
    setInputCloud (const PointCloudConstPtr & cloud);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:

    /**
     * @brief pointPickingCallback
     * @param event
     * @param param
     */
    void
    pointPickingCallback (const pcl_vis::PointPickingEvent & event,
                          void * param);

    /**
     * @brief keyboardCallback
     * @param event
     * @param param
     */
    void
    keyboardCallback (const pcl_vis::KeyboardEvent & event,
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

} // namespace calib
} // namespace unipd

#include <impl/calibration_pcl/utilities/point_plane_extraction_gui.hpp>

#endif // CALIBRATION_PCL_UTILITIES_POINT_PLANE_EXTRACTION_GUI_H_
