/*
 *  Copyright (c) 2015-, Filippo Basso <bassofil@gmail.com>
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

#include <gtest/gtest.h>
#include <calibration_algorithms/plane_to_plane_calibration.h>

using namespace unipd::calib;

TEST(PlaneToPlaneCalibration, estimateTransform)
{
  PlaneToPlaneCalibration calib;
  calib.addPlanePair(std::make_pair(Plane3(Vector3(0.112691, -0.276207, 0.954469), -1.35423),
                                    Plane3(Vector3(0.218832, -0.330751, 0.917996), -3.0117)));
  calib.addPlanePair(std::make_pair(Plane3(Vector3(-0.0162984, -0.123521, 0.992208), -1.33999),
                                    Plane3(Vector3(0.0756903, -0.127099, 0.988998), -3.21313)));
  calib.addPlanePair(std::make_pair(Plane3(Vector3(-0.153918, -0.0842592, 0.984484), -1.26553),
                                    Plane3(Vector3(-0.0650428, -0.121503, 0.990458), -3.08435)));
  calib.addPlanePair(std::make_pair(Plane3(Vector3(-0.413197, -0.118166, 0.902942), -1.17928),
                                    Plane3(Vector3(-0.325596, -0.15166, 0.933266), -2.79794)));
  calib.addPlanePair(std::make_pair(Plane3(Vector3(-0.093132, -0.0931221, 0.991289), -1.2432),
                                    Plane3(Vector3(0.00197333, -0.125383, 0.992106), -3.10749)));
  calib.addPlanePair(std::make_pair(Plane3(Vector3(-0.139159, -0.117658, 0.983256), -1.27124),
                                    Plane3(Vector3(-0.0612503, -0.150665, 0.986686), -3.08316)));
  calib.addPlanePair(std::make_pair(Plane3(Vector3(-0.275229, -0.122331, 0.953564), -1.27538),
                                    Plane3(Vector3(-0.188487, -0.141886, 0.971772), -3.0207)));
  calib.addPlanePair(std::make_pair(Plane3(Vector3(-0.0924412, -0.168909, 0.981287), -0.670834),
                                    Plane3(Vector3(-0.00893065, -0.195327, 0.980697), -2.46828)));
  calib.addPlanePair(std::make_pair(Plane3(Vector3(-0.248585, -0.0603065, 0.966731), -0.730401),
                                    Plane3(Vector3(-0.157237, -0.0922465, 0.983243), -2.52748)));
  calib.addPlanePair(std::make_pair(Plane3(Vector3(-0.60247, -0.095165, 0.792448), -0.732968),
                                    Plane3(Vector3(-0.507328, -0.114602, 0.854099), -2.19752)));

  Transform3 transform = calib.estimateTransform();
  EXPECT_TRUE(transform.translation().isApprox(Vector3(-0.0547, -0.6576, -1.9342), 0.001));

}
