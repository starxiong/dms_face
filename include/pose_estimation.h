
/*
 @Editors:  xiewei
 @E-mail: xie.wei@cidi.ai
 calibrate face model and estimate face pose and estimate moving keypoints
*/

#pragma once

#include "common.h"
#include "optimization.h"
#include "pose_calibrator.h"
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
  inline void SetNumber(int num) { visual_.SetNumber(num); }

private:
  void CalibrateFrontedFace();
  void ReconstructFace3D();
  float CaculatePoseAmplitude(Mat3D &R, Vec3D &t);

private:
  void UpdateFaceModel();
  void SolverFaceModel();
  float CalculateError();
  void CVPnpSolver(Mat3D &R, Vec3D &t);
  void ParseFace2D(std::vector<Point2D> &face_points);
  void SolverFace(std::vector<Vec3D> &fixed_point3d,
                  std::vector<Vec2D> &fixed_point2d,
                  std::vector<Vec3D> &moving_point3d,
                  std::vector<Vec2D> &moving_point2d, Mat3D &R, Vec3D &t);

private:
  Mat3D K_;
  Mat3D R_;
  Vec3D t_;
  Vec5D D_;

  Mat3D init_R_;
  Vec3D init_t_;

  Vec3D delta_R_;
  Vec3D delta_t_;

  Mat3D frontal_R_;
  Vec3D frontal_t_;
  int keypoint_num_;
  int frame_id_{0};

private:
  bool is_opt_facemodel_{0};

  int low_angle_num_{0};
  int media_angle_num_{0};
  int high_angle_num_{0};

  int low_angle_num_thresh_{100};
  int media_angle_num_thresh_{80};
  int high_angle_num_thresh_{60};

  float low_angle_thresh_{20};
  float high_angle_thresh_{40};

  std::vector<Mat3D> R_list_;
  std::vector<Vec3D> t_list_;
  std::vector<std::vector<Vec2D>> face_fixed2d_;
  std::vector<std::vector<Vec3D>> model_fixed3d_;
  std::vector<std::vector<Vec2D>> face_moving2d_;
  std::vector<std::vector<Vec3D>> model_moving3d_;

private:
  bool is_init_pose_{0};
  bool is_calibrate_{0};
  int calibrate_gap_num_{5};
  int calibrate_num_{100};

  std::vector<Mat3D> calibrate_R_list_;
  std::vector<Vec3D> calibrate_t_list_;
  FrontalPoseCalibrator frontal_calibrator_;
  std::vector<std::vector<Vec2D>> calibrate_face_fixed2d_;
  std::vector<std::vector<Vec3D>> calibrate_model_fixed3d_;
  std::vector<std::vector<Vec2D>> calibrate_face_moving2d_;
  std::vector<std::vector<Vec3D>> calibrate_model_moving3d_;

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
  std::unordered_map<int, int> symmetry_fixed_index_;
  std::unordered_map<int, int> symmetry_moving_index_;
  std::unordered_map<int, int> moving_fixed_constraint_index_;
  std::unordered_map<int, std::vector<int>> point_opt_fixed_dim_;
  std::unordered_map<int, std::vector<int>> point_opt_moving_dim_;
  // show compare
private:
  Mat3D R_show_;
  Vec3D t_show_;
  Visualization visual_;
  std::vector<Vec3D> fixed_points_show_;
  std::vector<Vec3D> moving_points_show_;
};
FACEPOSE_END