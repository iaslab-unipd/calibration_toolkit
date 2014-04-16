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

//Include fondamentali
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>

//Include per libreria calibration
#include <kinect/depth/polynomial_undistortion_matrix.h>
#include <kinect/depth/polynomial_undistortion_matrix_io.h>
#include <calibration_common/base/math.h>

//Include per test
#include <pcl/visualization/cloud_viewer.h>
#include <math.h>
#include <sstream>
#include <vector>

//Namespace per Polynomial e PolynomialUndistortionMatrix
using namespace calibration;

typedef Polynomial<double, 2, 0> PolynomialT;
typedef PolynomialMatrixSphereModel<PolynomialT> UndistortionMatrixModel; // La matrice dei dati
typedef UndistortionMatrixModel::Data UndistortionMatrixData; // La matrice dei dati
typedef PolynomialUndistortionMatrixPCL<PolynomialT, pcl::PointXYZ> UndistortionMatrixPCL; // Il tipo di dati da anti-distorcere (PCL)
typedef PolynomialUndistortionMatrixIO<PolynomialT> UndistortionMatrixIO;

//Dichiarazione metodi
// void pcl_view (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

int main(int argc,
         char **argv)
{

  ros::init(argc, argv, "robotica");

  UndistortionMatrixPCL::Ptr um;
  UndistortionMatrixIO io;

// // Read
  double fov_x, fov_y;
  UndistortionMatrixData::Ptr um_data;

  if (io.read(um_data, "/tmp/matrix.txt", fov_x, fov_y))
  {
    UndistortionMatrixModel::Ptr model = boost::make_shared<UndistortionMatrixModel>();
    model->setData(um_data);
    model->setFieldOfView(fov_x, fov_y);
    um = boost::make_shared<UndistortionMatrixPCL>();
    um->setModel(model);
  }

// Write
  if (um) // Se um e' stata inizializzata = il metodo read ha ritornato true
    io.write(*um->model(), "/tmp/matrix.txt");
  else
    std::cerr << "No undistortion matrix!!" << std::endl;

//spengo tutto
  ros::shutdown();
  return 0;
}
