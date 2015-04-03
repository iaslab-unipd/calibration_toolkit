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
    typedef typename PolynomialMatrixSimpleModel<Polynomial_>::Data Data;

    bool write(const PolynomialMatrixSimpleModel<Polynomial_> & undistortion_matrix,
               const std::string & file_name) const
    {
      return write(*undistortion_matrix.matrix(), file_name);
    }

    bool write(const PolynomialMatrixSmoothModel<Polynomial_> & undistortion_matrix,
               const std::string & file_name) const
    {
      return write(*undistortion_matrix.matrix(), file_name);
    }

    bool read(typename PolynomialMatrixSimpleModel<Polynomial_>::Ptr & undistortion_matrix,
              const std::string & file_name) const
    {
      assert(undistortion_matrix);
      typename Data::Ptr matrix;
      if (not read(matrix, file_name))
        return false;

      undistortion_matrix->setMatrix(matrix);
      return true;
    }

    bool read(typename PolynomialMatrixSmoothModel<Polynomial_>::Ptr & undistortion_matrix,
              const std::string & file_name) const
    {
      assert(undistortion_matrix);
      typename Data::Ptr matrix;
      if (not read(matrix, file_name))
        return false;

      undistortion_matrix->setMatrix(matrix);
      return true;
    }

    template <typename ModelT_>
      void toImage(const ModelT_ & model,
                   const Scalar z,
                   cv::Mat & image,
                   const Scalar max) const;

    template <typename ModelT_>
      void toImageAuto(const ModelT_ & model,
                       const Scalar z,
                       cv::Mat & image,
                       Scalar & max) const;

    template <typename ModelT_>
      bool writeImage(const ModelT_ & model,
                      Scalar z,
                      const std::string & file_name,
                      const Scalar max) const
      {
        cv::Mat image;
        toImage(model, z, image, max);
        return cv::imwrite(file_name, image);
      }

    template <typename ModelT_>
      bool writeImageAuto(const ModelT_ & model,
                          Scalar z,
                          const std::string & file_name,
                          Scalar & max) const
      {
        cv::Mat image;
        toImageAuto(model, z, image, max);
        return cv::imwrite(file_name, image);
      }

    bool write(const Data & data,
               const std::string & file_name) const;

    bool read(typename Data::Ptr & data,
              const std::string & file_name) const;

  protected:

    void toColorImage(cv::Mat & float_image,
                      cv::Mat & image,
                      const Scalar max) const;

  };

} /* namespace calibration */

#include <impl/kinect/depth/polynomial_matrix_io.hpp>

#endif /* KINECT_DEPTH_POLYNOMIAL_MATRIX_IO_H_ */
