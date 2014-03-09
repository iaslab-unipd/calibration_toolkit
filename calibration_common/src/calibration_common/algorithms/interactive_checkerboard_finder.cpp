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

#include <calibration_common/algorithms/interactive_checkerboard_finder.h>
#include <calibration_common/base/opencv_conversion.h>

namespace calibration
{

bool InteractiveCheckerboardFinder::find(const Checkerboard & checkerboard,
                                         Cloud2 & corners,
                                         bool try_automatically)
{
  std::vector<cv::Point2d> cv_corners;
  bool pattern_found = find(checkerboard, cv_corners, try_automatically);
  OpenCVConversion<Scalar>::toPointMatrix(cv_corners, corners);
  return pattern_found;
}

bool InteractiveCheckerboardFinder::find(const Checkerboard & checkerboard,
                                         std::vector<cv::Point2d> & corners,
                                         bool try_automatically)
{
  cv::Size pattern_size(checkerboard.cols(), checkerboard.rows());

  //ROS_INFO_STREAM("Select the checkerboard (" << checkerboard.rows() << "x" << checkerboard.cols() << ") in the image.");

  cv::imshow("SELECT CHECKERBOARD", image_);
  cv::setMouseCallback("SELECT CHECKERBOARD", &InteractiveCheckerboardFinder::selectSubImageCallback, this);

  bool pattern_found = false;

  if (try_automatically)
  {
    cv::Mat gray;
    cv::cvtColor(image_, gray, CV_BGR2GRAY);
    pattern_found = findChessboardCorners(gray, pattern_size, corners_float_);
  }
  if (pattern_found)
  {
    cv::drawChessboardCorners(image_, pattern_size, cv::Mat(corners_float_), pattern_found);
    cv::imshow("SELECT CHECKERBOARD", image_);
    cv::waitKey(500);
  }
  else
  {
    cv::waitKey();
  }

  bool automatic = true;

  while (/*ros::ok() and */not pattern_found)
  {
    for (int i = 1; /*ros::ok() and not*/pattern_found and i <= 5; ++i)
    {
      corners_float_.clear();

      cv::Mat sub_image;
      cv::Mat gray;
      cv::resize(image_(sub_image_rect_), sub_image, cv::Size(), i, i, cv::INTER_CUBIC);
      cv::cvtColor(sub_image, gray, CV_BGR2GRAY);

      if (automatic)
      {
        pattern_found = findChessboardCorners(gray, pattern_size, corners_float_);
      }
      else
      {
        cv::imshow("SELECT CORNERS", sub_image);
        cv::setMouseCallback("SELECT CORNERS", &InteractiveCheckerboardFinder::selectCornersCallback, this);
        cv::waitKey();
        pattern_found = (static_cast<int>(corners_float_.size()) == checkerboard.corners().size());
      }

      if (pattern_found)
        cv::cornerSubPix(gray,
                         corners_float_,
                         cv::Size(i * 2, i * 2),
                         cv::Size(-1, -1),
                         cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01));

      for (unsigned int j = 0; j < corners_float_.size(); ++j)
      {
        corners_float_[j].x = corners_float_[j].x / i + sub_image_rect_.x;
        corners_float_[j].y = corners_float_[j].y / i + sub_image_rect_.y;
      }

      if (pattern_found)
      {
        cv::drawChessboardCorners(image_, pattern_size, cv::Mat(corners_float_), pattern_found);
        cv::imshow("SELECT CHECKERBOARD", image_);
        cv::waitKey(500);
      }
    }

    if (not pattern_found)
    {
      automatic = false;
      //ROS_WARN("Pattern not found!!");
      //ROS_INFO("Select corners manually.");
    }
  }

  corners.clear();
  for (unsigned int j = 0; j < corners_float_.size(); ++j)
  {
    corners.push_back(cv::Point2d(corners_float_[j].x, corners_float_[j].y));
  }

  cv::destroyAllWindows();

  return pattern_found;
}

void InteractiveCheckerboardFinder::selectCornersCallback(int event,
                                                          int x,
                                                          int y,
                                                          int flags,
                                                          void *param)
{
  InteractiveCheckerboardFinder* cf = (InteractiveCheckerboardFinder*)param;
  switch (event)
  {
    case CV_EVENT_LBUTTONUP:
    {
      cv::Point2f p(x, y);
      cf->corners_float_.push_back(p);
      break;
    }
  }
}

void InteractiveCheckerboardFinder::selectSubImageCallback(int event,
                                                           int x,
                                                           int y,
                                                           int flags,
                                                           void *param)
{
  InteractiveCheckerboardFinder* cf = (InteractiveCheckerboardFinder*)param;
  switch (event)
  {
    case CV_EVENT_LBUTTONDOWN:
    {
      cf->sub_image_rect_.x = x;
      cf->sub_image_rect_.y = y;
      break;
    }
    case CV_EVENT_LBUTTONUP:
    {
      int min_x = std::min(x, cf->sub_image_rect_.x);
      int min_y = std::min(y, cf->sub_image_rect_.y);
      int width = std::abs(x - cf->sub_image_rect_.x);
      int height = std::abs(y - cf->sub_image_rect_.y);
      cf->sub_image_rect_.x = min_x;
      cf->sub_image_rect_.y = min_y;
      cf->sub_image_rect_.width = width;
      cf->sub_image_rect_.height = height;
      break;
    }
  }
}

} /* namespace calibration */
