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

#ifndef CALIBRATION_COMMON_DEPTH_TRAITS_H_
#define CALIBRATION_COMMON_DEPTH_TRAITS_H_

#include <pcl/point_cloud.h>
#include <calibration_common/objects/globals.h>

namespace calibration
{

/**
 * @brief The DepthTraits struct
 * @param DepthT_
 */
template <typename DepthT_>
  struct DepthTraits
  {
  };

/**
 * @brief The DepthEigen_ struct
 * @param ScalarT_
 */
template <typename ScalarT_>
  struct DepthEigen_
  {
  };

template <typename ScalarT_>
  struct DepthTraits<DepthEigen_<ScalarT_> >
  {
    typedef ScalarT_ Scalar;
    typedef typename Types<ScalarT_>::Point3 Point;
    typedef typename Types<ScalarT_>::Cloud3 Cloud;
  };

/**
 * @brief The DepthPCL_ struct
 * @param PCLPointT_
 */
template <typename PCLPointT_>
  struct DepthPCL_
  {
  };

template <typename PCLPointT_>
  struct DepthTraits<DepthPCL_<PCLPointT_> >
  {
    typedef float Scalar;
    typedef PCLPointT_ Point;
    typedef pcl::PointCloud<PCLPointT_> Cloud;
  };

typedef DepthEigen_<Scalar> DepthEigen;
typedef DepthPCL_<PCLPoint3> DepthPCL;

} /* namespace calibration */

#endif /* CALIBRATION_COMMON_DEPTH_TRAITS_H_ */
