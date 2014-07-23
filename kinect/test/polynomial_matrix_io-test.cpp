#include <ros/ros.h>
#include <pcl/io/pcd_io.h>

#include <calibration_common/base/math.h>
#include <kinect/depth/polynomial_matrix.h>
#include <kinect/depth/polynomial_matrix_io.h>

#include <pcl/visualization/cloud_viewer.h>

#include <gtest/gtest.h>

using namespace calibration;

typedef Polynomial<double, 2, 0> PolynomialT;
typedef PolynomialMatrixSimpleModel<PolynomialT> ModelT;
typedef ModelT::Data DataT;
typedef PolynomialMatrixPCL<ModelT, Scalar, PCLPoint3> MatrixPCL;
typedef PolynomialUndistortionMatrixIO<PolynomialT> MatrixIO;

TEST(PolynomialUndistortionMatrixIO, write_read)
{
  DataT::Ptr data = boost::make_shared<DataT>(Size2(15, 10), PolynomialT::IdentityCoefficients());

  ModelT::Ptr model = boost::make_shared<ModelT>(Size2(640, 480));
  model->setMatrix(data);

  MatrixPCL::Ptr um = boost::make_shared<MatrixPCL>();
  um->setModel(model);

  MatrixIO io;

// Write
  io.write(*um->model(), "/tmp/matrix.txt");

// Read
  DataT::Ptr um_data;

  if (io.read(um_data, "/tmp/matrix.txt"))
  {
    ModelT::Ptr model = boost::make_shared<ModelT>(Size2(640, 480));
    model->setMatrix(um_data);
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
