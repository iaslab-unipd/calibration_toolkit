#include <ros/ros.h>
#include <pcl/io/pcd_io.h>

#include <calibration_common/base/math.h>
#include <kinect/depth/polynomial_matrix.h>
#include <kinect/depth/polynomial_matrix_io.h>

#include <pcl/visualization/cloud_viewer.h>

#include <gtest/gtest.h>

using namespace calibration;

typedef Polynomial<double, 2, 0> PolynomialT;
typedef PolynomialMatrixProjectedModel<PolynomialT> ModelT;
typedef ModelT::Data DataT;
typedef DepthUndistortionImpl<ModelT, DepthPCL> MatrixPCL;
typedef PolynomialUndistortionMatrixIO<PolynomialT> MatrixIO;

TEST(PolynomialUndistortionMatrixIO, write_read)
{
  DataT::Ptr data = boost::make_shared<DataT>(15, 10, PolynomialT::IdentityCoefficients());

  ModelT::Ptr model = boost::make_shared<ModelT>();
  model->setData(data);
  model->setFieldOfView(M_PI/4.0, M_PI/6.0);

  MatrixPCL::Ptr um = boost::make_shared<MatrixPCL>();
  um->setModel(model);

  MatrixIO io;

// Write
  io.write(*um->model(), "/tmp/matrix.txt");

// Read
  double fov_x, fov_y;
  DataT::Ptr um_data;

  if (io.read(um_data, "/tmp/matrix.txt", fov_x, fov_y))
  {
    ModelT::Ptr model = boost::make_shared<ModelT>();
    model->setData(um_data);
    model->setFieldOfView(fov_x, fov_y);
    um = boost::make_shared<MatrixPCL>();
    um->setModel(model);
  }

  SUCCEED();
  return;
}

// Run all the tests that were declared with TEST()
int main(int argc,
         char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
