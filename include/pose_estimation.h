
/*
 @Editors:  xiewei
 @E-mail: znxiewei@163.com
 calibrate face model and estimate face pose and estimate moving keypoints
*/

#pragma once

#include "common.h"
#include "optimization.h"
#include "visualization.h"

FACEPOSE_BEGIN
class PoseEstimation {
public:
  PoseEstimation() = default;
  ~PoseEstimation() = default;
  int Init(std::string param_path);
  int Process(std::vector<Point2D> &points);
  void GetPose(Mat3D &R, Vec3D &t);
  void Display(cv::Mat &img);
  void Reset();

private:
  void UpdateFaceModel();
  void SolverFaceModel();
  float CalculateError();
  void CVPnpSolver(Mat3D &R, Vec3D &t);
  void ParseFace2D(std::vector<Point2D> &face_points);
  void DistortTransform(Mat3D &R, Vec3D &t, std::vector<Vec3D> &in,
                        std::vector<Vec2D> &out);
  void SolverFace(std::vector<Vec3D> &fixed_point3d,
                  std::vector<Vec2D> &fixed_point2d,
                  std::vector<Vec3D> &moving_point3d,
                  std::vector<Vec2D> &moving_point2d, Mat3D &R, Vec3D &t);

private:
  Mat3D K_;
  Mat3D R_;
  Vec3D t_;
  Vec5D D_;

  Vec3D delta_R_;
  Vec3D delta_t_;
  int keypoint_num_;
private:
  int buffer_size_{200};
  bool is_opt_facemodel_{0};
  std::vector<Mat3D> R_list_;
  std::vector<Vec3D> t_list_;
  std::vector<std::vector<Vec2D>> face_fixed2d_;
  std::vector<std::vector<Vec3D>> model_fixed3d_;
  std::vector<std::vector<Vec2D>> face_moving2d_;
  std::vector<std::vector<Vec3D>> model_moving3d_;

private:
  std::vector<Vec2D> fixed_point2D_;
  std::vector<Vec2D> moving_point2D_;

private:
  std::vector<Vec3D> fixed_points_;
  std::vector<Vec3D> moving_points_;
  std::vector<double> moving_delta_;
  std::vector<Vec3D> fixed_points_ori_;
  std::vector<Vec3D> moving_points_ori_;

private:
  std::unordered_map<int, int> same_moving_index_;
// show compare 
 private:
  Mat3D R_show_;
  Vec3D t_show_;
  Visualization visual_;
  std::vector<Vec3D> fixed_points_show_;
  std::vector<Vec3D> moving_points_show_;

};
FACEPOSE_END