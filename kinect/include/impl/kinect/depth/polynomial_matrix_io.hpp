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

#ifndef IMPL_KINECT_DEPTH_POLYNOMIAL_MATRIX_IO_HPP_
#define IMPL_KINECT_DEPTH_POLYNOMIAL_MATRIX_IO_HPP_

#include <kinect/depth/polynomial_matrix_io.h>
#include <sstream>
#include <boost/lexical_cast.hpp>

namespace calibration
{

template <class Polynomial_>
  bool PolynomialUndistortionMatrixIO<Polynomial_>::write(const Data & data,
                                                          const std::string & file_name,
                                                          Scalar fov_x,
                                                          Scalar fov_y) const
  {
    const int degree = MathTraits<Polynomial_>::Degree;
    const int min_degree = MathTraits<Polynomial_>::MinDegree;
    const int size = MathTraits<Polynomial_>::Size;
    assert(size > 0);

    const int matrix_column_number = data.xSize();
    const int matrix_row_number = data.ySize();

    std::ofstream output_file;
    std::stringstream output_stream;

    output_stream << "Polynomial " << degree << " " << min_degree << std::endl;
    output_stream << "FOV " << fov_x << " " << fov_y << std::endl;
    output_stream << "Matrix " << matrix_column_number << " " << matrix_row_number << std::endl;

    for (int y = 0; y < matrix_row_number; ++y)
    {
      for (int x = 0; x < matrix_column_number; ++x)
      {
        for (int k = 0; k < data(x, y).size(); k++)
        {
          output_stream << data(x, y)[k];
          if (k != size - 1)
            output_stream << " ";
        }
        if (not (y == matrix_row_number - 1 and x == matrix_column_number - 1))
          output_stream << std::endl;
      }
    }

    output_file.open(file_name.c_str());
    output_file << output_stream.str();
    output_file.close();

    return true;
  }

template <class Polynomial_>
  bool PolynomialUndistortionMatrixIO<Polynomial_>::read(typename Data::Ptr & data,
                                                         const std::string & file_name,
                                                         Scalar & x_fov,
                                                         Scalar & y_fov) const
  {
    bool file_found = false;

    std::vector<std::vector<Scalar> > matrix;

    std::ifstream input_file;

    int degree;
    int min_degree;
    size_t matrix_column_number;
    size_t matrix_row_number;

    input_file.open(file_name.c_str());
    if (input_file.is_open())
    {
      file_found = true;
      std::string line = "";

      int line_count = 0;
      bool assigned = false;

      while (std::getline(input_file, line))
      {
        char split_char = ' ';
        std::istringstream split(line);
        std::vector<std::string> tokens;
        for (std::string each; std::getline(split, each, split_char); tokens.push_back(each));

        if (line_count == 0)
        {
          degree = boost::lexical_cast<int>(tokens.at(1));
          min_degree = boost::lexical_cast<int>(tokens.at(2));
          if (degree != MathTraits<Polynomial_>::Degree or min_degree != MathTraits<Polynomial_>::MinDegree)
            return false;
          line_count++;
        }
        else if (line_count == 1)
        {
          x_fov = boost::lexical_cast<Scalar>(tokens.at(1));
          y_fov = boost::lexical_cast<Scalar>(tokens.at(2));
          line_count++;
        }
        else if (line_count == 2)
        {
          matrix_column_number = boost::lexical_cast<size_t>(tokens.at(1));
          matrix_row_number = boost::lexical_cast<size_t>(tokens.at(2));
          line_count++;
        }
        else
        {
          std::vector<Scalar> matrix_line;
          for (std::vector<std::string>::iterator it = tokens.begin(); it != tokens.end(); ++it)
          {
            Scalar coeff = boost::lexical_cast<Scalar>(*it);
            matrix_line.push_back(coeff);
          }
          matrix.push_back(matrix_line);
        }
      }
      input_file.close();
    }
    else
      return false;

    data = boost::make_shared<Data>(matrix_column_number, matrix_row_number);

    int coeff_count = 0;
    for (std::vector<std::vector<double> >::iterator ext_it = matrix.begin(); ext_it != matrix.end(); ++ext_it)
    {
      int line_coeff_count = 0;
      for (std::vector<double>::iterator int_it = ext_it->begin(); int_it != ext_it->end(); ++int_it)
      {
        int x = coeff_count % matrix_column_number;
        int y = coeff_count / matrix_column_number;
        (*data)(x, y)[line_coeff_count++] = *int_it;
      }
      coeff_count++;
    }

    return true;
  }

template <class Polynomial_>
  void PolynomialUndistortionMatrixIO<Polynomial_>::toColorImage(const cv::Mat & float_image,
                                                                 cv::Mat & image,
                                                                 const Scalar max) const
  {
//    float_image /= (2 * max);
//    float_image += 0.5f;
//
//    cv::Mat image;
//    float_image.convertTo(image, CV_8UC1, 255, 0);

    float_image /= max;

    image.create(float_image.rows, float_image.cols, CV_8UC3);
    for (int y = 0; y < float_image.rows; ++y)
    {
      for (int x = 0; x < float_image.cols; ++x)
      {
        const float & value = float_image.at<float>(y, x);
//        if (value > 0)
//          image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, char((1.0f - value) * 255), char((1.0f - value) * 255));
//        else
//          image.at<cv::Vec3b>(y, x) = cv::Vec3b(char((1.0f + value) * 255), char((1.0f + value) * 255), 255);
        if (value > 0.5f)
          image.at<cv::Vec3b>(y, x) = cv::Vec3b(char((1.0f - value) * 2 * 255), 0, 0);
        else if (value > 0.0f)
          image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, char((0.5f - value) * 2 * 255), char((0.5f - value) * 2 * 255));
        else if (value > -0.5f)
          image.at<cv::Vec3b>(y, x) = cv::Vec3b(char((0.5f + value) * 2 * 255), char((0.5f + value) * 2 * 255), 255);
        else
          image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, char((1.0f + value) * 2 * 255));
      }
    }

  }

template <class Polynomial_>
  void PolynomialUndistortionMatrixIO<Polynomial_>::toImage(const PolynomialMatrixModel<Polynomial_> & undistortion_matrix,
                                                            const Scalar z,
                                                            cv::Mat & image,
                                                            Scalar max) const
  {
    assert(max > 0);
    cv::Mat float_image(undistortion_matrix.data()->ySize(), undistortion_matrix.data()->xSize(), CV_32FC1);

    for (int y = 0; y < float_image.rows; ++y)
    {
      for (int x = 0; x < float_image.cols; ++x)
      {
        Scalar z_tmp = z;
        undistortion_matrix.undistort(x, y, z_tmp);
        z_tmp -= z;
        float_image.at<float>(y, x) = z_tmp;
      }
    }

    toColorImage(float_image, image, max);

  }

template <class Polynomial_>
  void PolynomialUndistortionMatrixIO<Polynomial_>::toImageAuto(const PolynomialMatrixModel<Polynomial_> & undistortion_matrix,
                                                                const Scalar z,
                                                                cv::Mat & image,
                                                                Scalar & max) const
  {
    max = 0;
    cv::Mat float_image(undistortion_matrix.data()->ySize(), undistortion_matrix.data()->xSize(), CV_32FC1);

    for (int y = 0; y < float_image.rows; ++y)
    {
      for (int x = 0; x < float_image.cols; ++x)
      {
        Scalar z_tmp = z;
        undistortion_matrix.undistort(x, y, z_tmp);
        z_tmp -= z;
        float_image.at<float>(y, x) = z_tmp;
        if (z_tmp > max or z_tmp < -max)
          max = std::fabs(z_tmp);
      }
    }

    toColorImage(float_image, image, max);

  }

} /* namespace calibration */
#endif /* IMPL_KINECT_DEPTH_POLYNOMIAL_MATRIX_IO_HPP_ */
