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

#ifndef CALIBRATION_COMMON_DEPTH_UNDISTORTION_MODEL_H_
#define CALIBRATION_COMMON_DEPTH_UNDISTORTION_MODEL_H_

#include <pcl/point_cloud.h>

#include <calibration_common/base/matrix.h>
#include <calibration_common/depth/traits.h>

namespace calibration
{

/**
 * @brief The ModelTraits struct
 * @param ModelT_
 */
template <typename ModelT_>
  struct ModelTraits
  {
  };

///**
// * @brief The DepthUndistortionModel struct
// * @param ModelT_
// */
//template <typename ModelT_>
//  struct DepthUndistortionModel
//  {
//    typedef boost::shared_ptr<DepthUndistortionModel> Ptr;
//    typedef boost::shared_ptr<const DepthUndistortionModel> ConstPtr;

////    typedef typename ModelTraits<ModelT_>::Scalar Scalar;
////    typedef typename ModelTraits<ModelT_>::Data Data;

////    /**
////     * @brief ~DepthUndistortionModel
////     */
////    virtual ~DepthUndistortionModel()
////    {
////      // Do nothing
////    }

////    /**
////     * @brief setData
////     * @param data
////     */
////    virtual void setData(const boost::shared_ptr<Data> & data) = 0;

////    /**
////     * @brief data
////     * @return
////     */
////    virtual const boost::shared_ptr<Data> & data() const = 0;

//  };

/**
 * @brief The DepthUndistortion struct
 * @param DepthT_
 */
template <typename DepthT_>
  struct DepthUndistortion
  {
    typedef boost::shared_ptr<DepthUndistortion> Ptr;
    typedef boost::shared_ptr<const DepthUndistortion> ConstPtr;

    typedef typename DepthTraits<DepthT_>::Point Point;
    typedef typename DepthTraits<DepthT_>::Cloud Cloud;

    /**
     * @brief ~DepthUndistortion
     */
    virtual ~DepthUndistortion()
    {
      // Do nothing
    }

    /**
     * @brief undistort
     * @param x_index
     * @param y_index
     * @param point
     */
    virtual void undistort(Size1 x_index,
                           Size1 y_index,
                           Point & point) const = 0;

    /**
     * @brief undistort
     * @param index
     * @param point
     */
    virtual void undistort(const Size2 & index,
                           Point & point) const = 0;

    /**
     * @brief undistort
     * @param cloud
     */
    virtual void undistort(Cloud & cloud) const = 0;

  };

///**
// * @brief The DepthUndistortionImpl struct
// * @param ModelT_
// * @param DepthT_
// */
//template <typename ModelT_, typename DepthT_>
//  struct DepthUndistortionImpl
//  {

//  };

/**
 * @brief The NoUndistortion_ class
 * @param DepthT_
 */
template <typename DepthT_>
  class NoUndistortion_ : public DepthUndistortion<DepthT_>
  {
  public:

    typedef boost::shared_ptr<NoUndistortion_> Ptr;
    typedef boost::shared_ptr<const NoUndistortion_> ConstPtr;

    typedef DepthUndistortion<DepthT_> Interface;

    typedef typename Interface::Point Point;
    typedef typename Interface::Cloud Cloud;

    /**
     * @brief ~NoUndistortion_
     */
    virtual ~NoUndistortion_()
    {
      // Do nothing
    }

    /**
     * @brief undistort
     * @param point
     */
    virtual void undistort(Point & point) const
    {
      // Do nothing
    }

    /**
     * @brief undistort
     * @param cloud
     */
    virtual void undistort(Cloud & cloud) const
    {
      // Do nothing
    }

  };

/**
 * @brief The NoUndistortionEigen_ struct
 * @param ScalarT_
 */
template <typename ScalarT_>
  struct NoUndistortionEigen_ : public NoUndistortion_<DepthEigen_<ScalarT_> >
  {
    typedef boost::shared_ptr<NoUndistortionEigen_> Ptr;
    typedef boost::shared_ptr<const NoUndistortionEigen_> ConstPtr;
  };

/**
 * @brief The NoUndistortionPCL_ struct
 * @param PCLPointT_
 */
template <typename PCLPointT_>
  struct NoUndistortionPCL_ : public NoUndistortion_<DepthPCL_<PCLPointT_> >
  {
    typedef boost::shared_ptr<NoUndistortionPCL_> Ptr;
    typedef boost::shared_ptr<const NoUndistortionPCL_> ConstPtr;
  };

/**
 * @brief NoUndistortionPCL
 */
typedef NoUndistortionPCL_<PCLPoint3> NoUndistortionPCL;

/**
 * @brief NoUndistortionEigen
 */
typedef NoUndistortionEigen_<Scalar> NoUndistortionEigen;

} /* namespace calibration */
#endif /* CALIBRATION_COMMON_DEPTH_UNDISTORTION_MODEL_H_ */
