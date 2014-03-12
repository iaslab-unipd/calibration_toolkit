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

#ifndef KINECT_DEPTH_POLYNOMIAL_MATRIX_IO_H_
#define KINECT_DEPTH_POLYNOMIAL_MATRIX_IO_H_

#include <kinect/depth/polynomial_matrix.h>
#include <opencv2/opencv.hpp>

namespace calibration
{

template <class Polynomial_>
  class PolynomialUndistortionMatrixIO
  {
  public:

    typedef typename MathTraits<Polynomial_>::Scalar Scalar;
    typedef typename PolynomialMatrixModel<Polynomial_>::Data Data;

    bool write(const PolynomialMatrixModel<Polynomial_> & undistortion_matrix,
               const std::string & file_name) const
    {
      return write(*undistortion_matrix.data(), file_name);
    }

    bool write(const PolynomialMatrixProjectedModel<Polynomial_> & undistortion_matrix,
               const std::string & file_name) const
    {
      return write(*undistortion_matrix.data(),
                   file_name,
                   undistortion_matrix.fieldOfViewX(),
                   undistortion_matrix.fieldOfViewY());
    }

    bool read(typename PolynomialMatrixModel<Polynomial_>::Ptr & undistortion_matrix,
              const std::string & file_name) const
    {
      assert(undistortion_matrix);
      typename Data::Ptr data;
      Scalar x_fov, y_fov;
      if (not read(data, file_name, x_fov, y_fov))
        return false;

      undistortion_matrix->setData(data);
      return true;
    }

    bool read(typename PolynomialMatrixProjectedModel<Polynomial_>::Ptr & undistortion_matrix,
              const std::string & file_name) const
    {
      assert(undistortion_matrix);
      typename Data::Ptr data;
      Scalar x_fov, y_fov;
      if (not read(data, file_name, x_fov, y_fov) or x_fov == 0 or y_fov == 0)
        return false;

      undistortion_matrix->setData(data);
      undistortion_matrix->setFieldOfView(x_fov, y_fov);
      return true;
    }

    void toImage(const PolynomialMatrixModel<Polynomial_> & undistortion_matrix,
                 const Scalar z,
                 cv::Mat & image,
                 const Scalar max) const;

    void toImageAuto(const PolynomialMatrixModel<Polynomial_> & undistortion_matrix,
                     const Scalar z,
                     cv::Mat & image,
                     Scalar & max) const;

    bool writeImage(const PolynomialMatrixModel<Polynomial_> & undistortion_matrix,
                    Scalar z,
                    const std::string & file_name,
                    const Scalar max) const
    {
      cv::Mat image;
      toImage(undistortion_matrix, z, image, max);
      return cv::imwrite(file_name, image);
    }

    bool writeImageAuto(const PolynomialMatrixModel<Polynomial_> & undistortion_matrix,
                        Scalar z,
                        const std::string & file_name,
                        Scalar & max) const
    {
      cv::Mat image;
      toImageAuto(undistortion_matrix, z, image, max);
      return cv::imwrite(file_name, image);
    }

    bool write(const Data & data,
               const std::string & file_name,
               Scalar fov_x = 0.0,
               Scalar fov_y = 0.0) const;

    bool read(typename Data::Ptr & data,
              const std::string & file_name,
              Scalar & x_fov,
              Scalar & y_fov) const;

  protected:

    void toColorImage(const cv::Mat & float_image,
                      cv::Mat & image,
                      const Scalar max) const;

  };

} /* namespace calibration */

#include <impl/kinect/depth/polynomial_matrix_io.hpp>

#endif /* KINECT_DEPTH_POLYNOMIAL_MATRIX_IO_H_ */
