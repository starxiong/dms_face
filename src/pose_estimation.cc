#include "../include/pose_estimation.h"

FACEPOSE_BEGIN

int PoseEstimation::Init(std::string param_path) {
  // read param
  YAML::Node fs = YAML::LoadFile(param_path);
  std::vector<double> K_vec = fs["K"]["data"].as<std::vector<double>>();
  std::vector<double> D_vec = fs["D"]["data"].as<std::vector<double>>();

  std::vector<double> left_eyebrow_left =
      fs["fixed_point"]["left_eyebrow_left"].as<std::vector<double>>();
  std::vector<double> left_eyebrow_right =
      fs["fixed_point"]["left_eyebrow_right"].as<std::vector<double>>();
  std::vector<double> right_eyebrow_left =
      fs["fixed_point"]["right_eyebrow_left"].as<std::vector<double>>();
  std::vector<double> right_eyebrow_right =
      fs["fixed_point"]["right_eyebrow_right"].as<std::vector<double>>();
  std::vector<double> left_eye_left =
      fs["fixed_point"]["left_eye_left"].as<std::vector<double>>();
  std::vector<double> left_eye_right =
      fs["fixed_point"]["left_eye_right"].as<std::vector<double>>();
  std::vector<double> right_eye_left =
      fs["fixed_point"]["right_eye_left"].as<std::vector<double>>();
  std::vector<double> right_eye_right =
      fs["fixed_point"]["right_eye_right"].as<std::vector<double>>();
  std::vector<double> nose_upper =
      fs["fixed_point"]["nose_upper"].as<std::vector<double>>();
  std::vector<double> nose_lower =
      fs["fixed_point"]["nose_lower"].as<std::vector<double>>();
  std::vector<double> nose_left =
      fs["fixed_point"]["nose_left"].as<std::vector<double>>();
  std::vector<double> nose_center =
      fs["fixed_point"]["nose_center"].as<std::vector<double>>();
  std::vector<double> nose_right =
      fs["fixed_point"]["nose_right"].as<std::vector<double>>();
  std::vector<double> lip_left =
      fs["fixed_point"]["lip_left"].as<std::vector<double>>();
  std::vector<double> lip_right =
      fs["fixed_point"]["lip_right"].as<std::vector<double>>();
  std::vector<double> chin =
      fs["fixed_point"]["chin"].as<std::vector<double>>();

  std::vector<double> left_eyebrow_center =
      fs["moving_point"]["left_eyebrow_center"].as<std::vector<double>>();
  std::vector<double> right_eyebrow_center =
      fs["moving_point"]["right_eyebrow_center"].as<std::vector<double>>();
  std::vector<double> left_eye_up1 =
      fs["moving_point"]["left_eye_up1"].as<std::vector<double>>();
  std::vector<double> left_eye_up2 =
      fs["moving_point"]["left_eye_up2"].as<std::vector<double>>();
  std::vector<double> left_eye_dn1 =
      fs["moving_point"]["left_eye_dn1"].as<std::vector<double>>();
  std::vector<double> left_eye_dn2 =
      fs["moving_point"]["left_eye_dn2"].as<std::vector<double>>();
  std::vector<double> right_eye_up1 =
      fs["moving_point"]["right_eye_up1"].as<std::vector<double>>();
  std::vector<double> right_eye_up2 =
      fs["moving_point"]["right_eye_up2"].as<std::vector<double>>();
  std::vector<double> right_eye_dn1 =
      fs["moving_point"]["right_eye_dn1"].as<std::vector<double>>();
  std::vector<double> right_eye_dn2 =
      fs["moving_point"]["right_eye_dn2"].as<std::vector<double>>();
  std::vector<double> lip_up1 =
      fs["moving_point"]["lip_up1"].as<std::vector<double>>();
  std::vector<double> lip_up2 =
      fs["moving_point"]["lip_up2"].as<std::vector<double>>();
  std::vector<double> lip_up3 =
      fs["moving_point"]["lip_up3"].as<std::vector<double>>();
  std::vector<double> lip_dn1 =
      fs["moving_point"]["lip_dn1"].as<std::vector<double>>();
  std::vector<double> lip_dn2 =
      fs["moving_point"]["lip_dn2"].as<std::vector<double>>();
  std::vector<double> lip_dn3 =
      fs["moving_point"]["lip_dn3"].as<std::vector<double>>();

  K_ = Mat3D(K_vec.data()).transpose();
  D_ = Vec5D(D_vec.data());

  fixed_points_.emplace_back(Vec3D(left_eyebrow_left.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(left_eyebrow_right.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(right_eyebrow_left.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(right_eyebrow_right.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(left_eye_left.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(left_eye_right.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(right_eye_left.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(right_eye_right.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(nose_upper.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(nose_lower.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(nose_left.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(nose_center.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(nose_right.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(lip_left.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(lip_right.data()) / 10.0);
  fixed_points_.emplace_back(Vec3D(chin.data()) / 10.0);

  moving_points_.emplace_back(Vec3D(left_eyebrow_center.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(right_eyebrow_center.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(left_eye_up1.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(left_eye_up2.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(left_eye_dn1.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(left_eye_dn2.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(right_eye_up1.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(right_eye_up2.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(right_eye_dn1.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(right_eye_dn2.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(lip_up1.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(lip_up2.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(lip_up3.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(lip_dn1.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(lip_dn2.data()) / 10.0);
  moving_points_.emplace_back(Vec3D(lip_dn3.data()) / 10.0);

  fixed_point2D_.resize(fixed_points_.size());
  moving_point2D_.resize(moving_points_.size());
  moving_delta_ = std::vector<double>(moving_points_.size(), 0);

  R_ = Mat3D::Identity();
  t_ = Vec3D::Zero();
  delta_R_ = Vec3D::Zero();
  delta_t_ = Vec3D::Zero();
  R_show_ = Mat3D::Identity();
  t_show_ = Vec3D::Zero();

  fixed_points_ori_ = fixed_points_;
  moving_points_ori_ = moving_points_;

  // show compare
  fixed_points_show_ = fixed_points_;
  moving_points_show_ = moving_points_;

  visual_.SetParameters(K_, D_);
  same_moving_index_[0] = 0;
  same_moving_index_[1] = 0;
  same_moving_index_[2] = 2;
  same_moving_index_[3] = 2;
  same_moving_index_[4] = 4;
  same_moving_index_[5] = 4;
  same_moving_index_[6] = 6;
  same_moving_index_[7] = 6;
  same_moving_index_[8] = 8;
  same_moving_index_[9] = 8;
  same_moving_index_[10] = 10;
  same_moving_index_[11] = 11;
  same_moving_index_[12] = 10;
  same_moving_index_[13] = 13;
  same_moving_index_[14] = 14;
  same_moving_index_[15] = 13;

  return 0;
}
void PoseEstimation::ParseFace2D(std::vector<Point2D>& points) {
  fixed_point2D_[0] = Vec2D(points[0].x, points[0].y);
  fixed_point2D_[1] = Vec2D(points[2].x, points[2].y);

  fixed_point2D_[2] = Vec2D(points[3].x, points[3].y);
  fixed_point2D_[3] = Vec2D(points[5].x, points[5].y);

  fixed_point2D_[4] = Vec2D(points[6].x, points[6].y);
  fixed_point2D_[5] = Vec2D(points[9].x, points[9].y);

  fixed_point2D_[6] = Vec2D(points[12].x, points[12].y);
  fixed_point2D_[7] = Vec2D(points[15].x, points[15].y);

  fixed_point2D_[8] = Vec2D(points[18].x, points[18].y);
  fixed_point2D_[9] = Vec2D(points[19].x, points[19].y);

  fixed_point2D_[10] = Vec2D(points[20].x, points[20].y);
  fixed_point2D_[11] = Vec2D(points[21].x, points[21].y);
  fixed_point2D_[12] = Vec2D(points[22].x, points[22].y);

  fixed_point2D_[13] = Vec2D(points[23].x, points[23].y);
  fixed_point2D_[14] = Vec2D(points[27].x, points[27].y);

  fixed_point2D_[15] = Vec2D(points[31].x, points[31].y);

  moving_point2D_[0] = Vec2D(points[1].x, points[1].y);
  moving_point2D_[1] = Vec2D(points[4].x, points[4].y);

  moving_point2D_[2] = Vec2D(points[7].x, points[7].y);
  moving_point2D_[3] = Vec2D(points[8].x, points[8].y);
  moving_point2D_[4] = Vec2D(points[10].x, points[10].y);
  moving_point2D_[5] = Vec2D(points[11].x, points[11].y);

  moving_point2D_[6] = Vec2D(points[13].x, points[13].y);
  moving_point2D_[7] = Vec2D(points[14].x, points[14].y);
  moving_point2D_[8] = Vec2D(points[16].x, points[16].y);
  moving_point2D_[9] = Vec2D(points[17].x, points[17].y);

  moving_point2D_[10] = Vec2D(points[24].x, points[24].y);
  moving_point2D_[11] = Vec2D(points[25].x, points[25].y);
  moving_point2D_[12] = Vec2D(points[26].x, points[26].y);

  moving_point2D_[13] = Vec2D(points[28].x, points[28].y);
  moving_point2D_[14] = Vec2D(points[29].x, points[29].y);
  moving_point2D_[15] = Vec2D(points[30].x, points[30].y);
}

void PoseEstimation::CVPnpSolver(Mat3D& R, Vec3D& t) {
  std::vector<cv::Point3f> cvpoint3d;
  std::vector<cv::Point2f> cvpoint2d;
  for (int i = 0; i < fixed_points_.size(); i++) {
    cvpoint3d.emplace_back(cv::Point3f(fixed_points_[i](0), fixed_points_[i](1),
                                       fixed_points_[i](2)));
    cvpoint2d.emplace_back(
        cv::Point2f(fixed_point2D_[i](0), fixed_point2D_[i](1)));
  }

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat rot, trans, rot_vec;
  cv::eigen2cv(K_, K);
  cv::Mat distortion = cv::Mat::zeros(1, 5, cv::DataType<double>::type);
  cv::solvePnP(cvpoint3d, cvpoint2d, K, distortion, rot_vec, trans);
  Mat3D Ri;
  Vec3D ti;
  cv::Rodrigues(rot_vec, rot);
  cv::cv2eigen(rot, R);
  cv::cv2eigen(trans, t);
}

void PoseEstimation::DistortTransform(Mat3D& R, Vec3D& t,
                                      std::vector<Vec3D>& in,
                                      std::vector<Vec2D>& out) {
  double k1 = D_(0);
  double k2 = D_(1);
  double p1 = D_(2);
  double p2 = D_(3);
  double k3 = D_(4);
  out = std::vector<Vec2D>(in.size());
  for (int i = 0; i < in.size(); i++) {
    Vec3D p3d = R * in[i] + t;
    double x = p3d(0) / p3d(2);
    double y = p3d(1) / p3d(2);
    double r2 = x * x + y * y;
    double kr = 1 + (k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    double x_ = x * kr + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    double y_ = y * kr + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    Vec2D c = Vec2D(x_, y_);
    out[i] = K_.block<2, 2>(0, 0) * c + K_.block<2, 1>(0, 2);
  }
}

void PoseEstimation::Display(cv::Mat& img) {
  Mat3D R = Mat3D::Identity();
  Vec3D t = Vec3D(0, 0, 9);
  SolverFace(fixed_points_show_, fixed_point2D_, moving_points_show_,
             moving_point2D_, R_show_, t_show_);
  CVPnpSolver(R, t);
  visual_.SetDetectKeyPoints(fixed_point2D_, moving_point2D_);
  visual_.SetJointOptimization(fixed_points_, moving_points_, R_, t_);
  visual_.SetOpencvOptimization(fixed_points_show_, moving_points_show_, R, t);
  visual_.Display(img);
}

float PoseEstimation::CalculateError() {
  float error = 0;
  for (int i = 0; i < moving_points_.size(); i++) {
    Vec3D p = K_ * (R_ * moving_points_[i] + t_);
    Vec2D p2 = Vec2D(p(0) / p(2), p(1) / p(2));
    error += (p2 - moving_point2D_[i]).norm();
  }
  for (int i = 0; i < fixed_points_.size(); i++) {
    Vec3D p = K_ * (R_ * fixed_points_[i] + t_);
    Vec2D p2 = Vec2D(p(0) / p(2), p(1) / p(2));
    error += (p2 - fixed_point2D_[i]).norm();
  }
  error /= float(fixed_points_.size() + moving_points_.size());
  return error;
}

void PoseEstimation::SolverFace(std::vector<Vec3D>& fixed_point3d,
                                std::vector<Vec2D>& fixed_point2d,
                                std::vector<Vec3D>& moving_point3d,
                                std::vector<Vec2D>& moving_point2d, Mat3D& R,
                                Vec3D& t) {
  Vec3D delta_R = Vec3D::Zero();
  Vec3D delta_t = Vec3D::Zero();
  int mn = moving_point3d.size();
  std::vector<double> moving_delta = std::vector<double>(mn, 0.0);
  ceres::Problem problem;
  for (int i = 0; i < fixed_point3d.size(); i++) {
    auto p2d = fixed_point2d[i];
    auto p3d = fixed_point3d[i];
    ceres::CostFunction* cost =
        new ceres::AutoDiffCostFunction<Optimization, 2, 3, 3, 16>(
            new Optimization(p2d, p3d, K_, R, t, false, 0));
    problem.AddResidualBlock(cost, nullptr, delta_R.data(), delta_t.data(),
                             moving_delta.data());
  }
  for (int i = 0; i < moving_point3d.size(); i++) {
    auto p2d = moving_point2d[i];
    auto p3d = moving_point3d[i];
    int id = same_moving_index_[i];
    ceres::CostFunction* cost =
        new ceres::AutoDiffCostFunction<Optimization, 2, 3, 3, 16>(
            new Optimization(p2d, p3d, K_, R, t, true, id));
    problem.AddResidualBlock(cost, nullptr, delta_R.data(), delta_t.data(),
                             moving_delta.data());
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 200;
  options.num_threads = 4;
  // 求解
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.BriefReport() << std::endl;

  double theta = delta_R.norm();
  double sin_val = std::sin(theta);
  double minus = 1 - std::cos(theta);
  Mat3D skew, deltaR;
  skew << 0, -delta_R(2), delta_R(1), delta_R(2), 0, -delta_R(0), -delta_R(1),
      delta_R(0), 0;
  if (theta < 1e-9) {
    deltaR = Mat3D::Identity() + skew + (skew * skew) * 0.5;
  } else {
    Eigen::AngleAxis<double> angle_axis(theta, delta_R / theta);
    deltaR = angle_axis.toRotationMatrix();
  }
  R = deltaR * R;
  t = deltaR * t + delta_t;
  for (int i = 0; i < moving_delta.size(); i++) {
    // moving_point3d[i](1) += moving_delta[i];
    if (same_moving_index_[i] != i) {
      moving_point3d[i](1) += moving_delta[same_moving_index_[i]];
    } else {
      moving_point3d[i](1) += moving_delta[i];
    }
  }
}

void PoseEstimation::SolverFaceModel() {
  ceres::Problem problem;
  int fn = fixed_points_.size();
  int mn = moving_points_.size();
  std::vector<Vec3D> fix_delta = std::vector<Vec3D>(fn, Vec3D::Zero());
  std::vector<Vec3D> move_delta = std::vector<Vec3D>(mn, Vec3D::Zero());
  for (int i = 0; i < model_fixed3d_.size(); i++) {
    Mat3D R = R_list_[i];
    Vec3D t = t_list_[i];
    for (int j = 0; j < model_fixed3d_[i].size(); j++) {
      auto p2d = face_fixed2d_[i][j];
      auto p3d = model_fixed3d_[i][j];
      ceres::CostFunction* cost =
          new ceres::AutoDiffCostFunction<FaceModelOptimization, 2, 3>(
              new FaceModelOptimization(p2d, p3d, K_, R, t));
      problem.AddResidualBlock(cost, nullptr, fix_delta[j].data());
    }
    for (int j = 0; j < model_moving3d_[i].size(); j++) {
      auto p2d = face_moving2d_[i][j];
      auto p3d = model_moving3d_[i][j];
      ceres::CostFunction* cost =
          new ceres::AutoDiffCostFunction<FaceModelOptimization, 2, 3>(
              new FaceModelOptimization(p2d, p3d, K_, R, t));
      problem.AddResidualBlock(cost, nullptr, move_delta[j].data());
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 200;
  options.num_threads = 4;
  // 求解
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.BriefReport() << std::endl;
  for (int i = 0; i < model_fixed3d_.size(); i++) {
    for (int j = 0; j < fix_delta.size(); j++) {
      model_fixed3d_[i][j] += fix_delta[j];
    }
  }
  for (int i = 0; i < model_moving3d_.size(); i++) {
    for (int j = 0; j < move_delta.size(); j++) {
      model_moving3d_[i][j] += move_delta[j];
    }
  }
  for (int i = 0; i < fix_delta.size(); i++) {
    fixed_points_[i] += fix_delta[i];
  }
  for (int i = 0; i < move_delta.size(); i++) {
    moving_points_[i] += move_delta[i];
  }
}

void PoseEstimation::UpdateFaceModel() {
  SolverFaceModel();
  for (int i = 0; i < 2; i++) {
    for (int i = 0; i < face_fixed2d_.size(); i++) {
      auto fp2d = face_fixed2d_[i];
      auto fp3d = model_fixed3d_[i];
      auto mp2d = face_moving2d_[i];
      auto mp3d = model_moving3d_[i];
      auto& R = R_list_[i];
      auto& t = t_list_[i];
      SolverFace(fp3d, fp2d, mp3d, mp2d, R, t);
    }
    SolverFaceModel();
  }
  R_list_.clear();
  t_list_.clear();
  face_fixed2d_.clear();
  face_moving2d_.clear();
  model_fixed3d_.clear();
  model_moving3d_.clear();
}

int PoseEstimation::Process(std::vector<Point2D>& face_points) {
  ParseFace2D(face_points);
  SolverFace(fixed_points_, fixed_point2D_, moving_points_, moving_point2D_, R_,
             t_);
  float error = CalculateError();
  if (error > 15) {
    Reset();
    SolverFace(fixed_points_, fixed_point2D_, moving_points_, moving_point2D_,
               R_, t_);
  }
  std::cout << "error: " << error << std::endl;
  if (model_fixed3d_.size() < buffer_size_) {
    face_fixed2d_.emplace_back(fixed_point2D_);
    model_fixed3d_.emplace_back(fixed_points_);
    face_moving2d_.emplace_back(moving_point2D_);
    model_moving3d_.emplace_back(moving_points_);
    R_list_.emplace_back(R_);
    t_list_.emplace_back(t_);
  } else if (!is_opt_facemodel_) {
    UpdateFaceModel();
    is_opt_facemodel_ = true;
  }
  return 0;
}

void PoseEstimation::Reset() {
  R_list_.clear();
  t_list_.clear();
  face_fixed2d_.clear();
  face_moving2d_.clear();
  model_fixed3d_.clear();
  model_moving3d_.clear();
  is_opt_facemodel_ = false;
  R_ = Mat3D::Identity();
  t_ = Vec3D::Zero();
  delta_R_ = Vec3D::Zero();
  delta_t_ = Vec3D::Zero();
  fixed_points_ = fixed_points_ori_;
  moving_points_ = moving_points_ori_;
  moving_delta_ = std::vector<double>(moving_delta_.size(), 0.0);
}

void PoseEstimation::GetPose(Mat3D& R, Vec3D& t) {
  R = R_;
  t = t_;
}

FACEPOSE_END