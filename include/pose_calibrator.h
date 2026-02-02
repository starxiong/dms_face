/*
 @Editors:  xiewei
 @E-mail: xie.wei@cidi.ai
 clibrate fronted face pose by ransac
*/

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <memory>
#include <random>
#include <vector>

#include "common.h"

FACEPOSE_BEGIN

class FrontalPoseCalibrator {
 public:
  FrontalPoseCalibrator() = default;
  void Init(double angle_threshold);
  void AddPose(const Mat3D& rotation, const Vec3D& translation);
  void Calibrate(std::vector<Mat3D>& rots, std::vector<Vec3D>& trans);
  void GetResult(Mat3D& R, Vec3D& t);

 private:
  std::vector<bool> ComputeInliers(const std::vector<Quat>& quats,
                                   const std::vector<Vec3D>& trans,
                                   const Quat& q_center, const Vec3D& t_center);
  void RansacEstimate(std::vector<Quat>& quats, std::vector<Vec3D>& trans);
  Quat ComputeQuatAverage(const std::vector<Quat>& quats);
  Vec3D ComputeTransAverage(const std::vector<Vec3D>& trans);
  double QuaternionDistance(const Quat& q1, const Quat& q2);

 private:
  std::mt19937 rng_;
  int iterations_;
  double angle_threshold_;
  CalibrationResult result_;
};

FACEPOSE_END