#include "../include/visualization.h"

FACEPOSE_BEGIN

void Visualization::DistortTransform(Mat3D& R, Vec3D& t, std::vector<Vec3D>& in,
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

void Visualization::Draw3DCoordinateSystem(cv::Mat& image,
                                           const Vec3D& world_origin,
                                           double axis_length, const Mat3D& R,
                                           const Vec3D& t) {
  // 1. 定义世界坐标系中的3D点（X左，Z外，Y上）
  std::vector<cv::Point3f> world_points_3d;

  // 将Vec3D转换为cv::Point3f
  cv::Point3f origin(static_cast<float>(world_origin(0)),
                     static_cast<float>(world_origin(1)),
                     static_cast<float>(world_origin(2)));

  // 根据坐标系定义：X左，Z外，Y上
  world_points_3d.push_back(origin);  // 原点
  world_points_3d.push_back(
      origin + cv::Point3f(-axis_length, 0, 0));  // X轴末端（向左）
  world_points_3d.push_back(origin +
                            cv::Point3f(0, axis_length, 0));  // Y轴末端（向上）
  world_points_3d.push_back(
      origin + cv::Point3f(0, 0, -axis_length));  // Z轴末端（向外）
  // 2. 转换为OpenCV格式
  cv::Mat cv_K, cv_D, cv_R, cv_t;
  cv::eigen2cv(K_, cv_K);
  cv::eigen2cv(D_, cv_D);
  cv::eigen2cv(R, cv_R);
  cv::eigen2cv(t, cv_t);
  // 3. 投影到图像平面
  std::vector<cv::Point2f> image_points;
  // projectPoints需要旋转向量而不是旋转矩阵
  cv::Mat rvec;
  cv::Rodrigues(cv_R, rvec);  // 旋转矩阵 -> 旋转向量
  // 执行投影
  cv::projectPoints(world_points_3d, rvec, cv_t, cv_K, cv_D, image_points);
  // 5. 绘制坐标轴
  cv::Scalar colors[] = {
      cv::Scalar(0, 0, 255),     // 红色 - X轴（左）
      cv::Scalar(0, 255, 0),     // 绿色 - Y轴（上）
      cv::Scalar(255, 0, 0),     // 蓝色 - Z轴（外）
      cv::Scalar(255, 255, 255)  // 白色 - 原点
  };
  const char* labels[] = {"X", "Y", "Z", "O"};
  // 绘制三个轴
  for (int i = 1; i <= 3; ++i) {
    cv::Point2f p1 = image_points[0];
    cv::Point2f p2 = image_points[i];
    // 检查点是否在图像可见范围内（可以适当放宽限制）
    bool p1_visible = (p1.x >= -100 && p1.x <= image.cols + 100 &&
                       p1.y >= -100 && p1.y <= image.rows + 100);
    bool p2_visible = (p2.x >= -100 && p2.x <= image.cols + 100 &&
                       p2.y >= -100 && p2.y <= image.rows + 100);

    if (p1_visible && p2_visible) {
      // 绘制带箭头的轴
      // 添加标签和方向指示
      std::string label = std::string(labels[i - 1]);
      cv::Point label_pos;
      // 根据轴方向调整标签位置
      switch (i) {
        case 1:  // X轴（左）
          cv::arrowedLine(image, p1, p2, colors[i - 1], 2, cv::LINE_AA, 0, 0.1);
          label_pos =
              cv::Point(static_cast<int>(p2.x - 25), static_cast<int>(p2.y));
          break;
        case 2:  // Y轴（上）
          cv::arrowedLine(image, p1, p2, colors[i - 1], 1, cv::LINE_AA, 0, 0.1);
          label_pos =
              cv::Point(static_cast<int>(p2.x), static_cast<int>(p2.y - 15));
          break;
        case 3:  // Z轴（外）
          cv::arrowedLine(image, p1, p2, colors[i - 1], 2, cv::LINE_AA, 0, 0.1);
          label_pos =
              cv::Point(static_cast<int>(p2.x + 5), static_cast<int>(p2.y));
          break;
      }
      cv::putText(image, label, label_pos, cv::FONT_HERSHEY_SIMPLEX, 0.2,
                  colors[i - 1], 1);
    }
  }
}

void Visualization::DrawAlignedPoints(cv::Mat& img, cv::Mat& img2,
                                      std::vector<Vec3D>& point1,
                                      std::vector<Vec3D>& point2) {
  int width = 300, height = 300;
  std::vector<Vec2D> point2d;
  Mat3D R = Mat3D::Identity();
  R = AxisD(3.14159, Vec3D::UnitZ()) * AxisD(0, Vec3D::UnitY()) *
      AxisD(0, Vec3D::UnitX());
  Vec3D trans = Vec3D(width / 2, width / 2, 0);
  cv::Mat small_img(width, height, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat small_img2(width, height, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 0; i < point1.size(); i++) {
    Vec3D p3d = R * point1[i];
    double x = p3d(0) * width / 2 + width / 2;
    double y = p3d(1) * width / 2 + height / 2;
    cv::circle(small_img, cv::Point(x, y), 2, cv::Scalar(255, 255, 255), -1);
  }
  for (int i = 0; i < point2.size(); i++) {
    Vec3D p3d = R * point2[i];
    double x = p3d(0) * width / 2 + width / 2;
    double y = p3d(1) * width / 2 + height / 2;
    cv::circle(small_img, cv::Point(x, y), 2, cv::Scalar(255, 0, 255), -1);
  }

  cv::Mat roi = img(cv::Rect(70, 30, small_img.cols, small_img.rows));
  small_img.copyTo(roi);

  // 目标平面上的对应5个点
  Vec2D p1 = fixed_point2D_[0];
  Vec2D p2 = fixed_point2D_[1];
  Vec2D p3 = fixed_point2D_[2];
  Vec2D p4 = fixed_point2D_[3];
  Vec2D p5 = fixed_point2D_[13];
  Vec2D p6 = fixed_point2D_[14];
  std::vector<cv::Point2f> srcPoints = {cv::Point2f(p1.x(), p1.y()),
                                        // cv::Point2f(p2.x(), p2.y()),
                                        // cv::Point2f(p3.x(), p3.y()),
                                        cv::Point2f(p4.x(), p4.y()),
                                        cv::Point2f(p5.x(), p5.y()),
                                        cv::Point2f(p6.x(), p6.y())};

  std::vector<cv::Point2f> dstPoints = {
      cv::Point2f(width * 4 / 16, height / 4),
      // cv::Point2f(width * 7 / 16, height / 4),
      // cv::Point2f(width * 9 / 16, height / 4),
      cv::Point2f(width * 12 / 16, height / 4),
      cv::Point2f(width * 3 / 8, height * 3 / 5),
      cv::Point2f(width * 5 / 8, height * 3 / 5)};

  cv::Mat M = cv::getPerspectiveTransform(srcPoints, dstPoints);
  // cv::Mat M = cv::estimateAffine2D(srcPoints, dstPoints);
  for (auto p : fixed_point2D_) {
    // 创建点向量
    std::vector<cv::Point2f> src_point = {cv::Point2f(p.x(), p.y())};
    std::vector<cv::Point2f> dst_point;
    // 使用cv::transform变换单个点
    // cv::transform(src_point, dst_point, M);
    cv::perspectiveTransform(src_point, dst_point, M);
    // 绘制结果
    cv::circle(small_img2, dst_point[0], 2, cv::Scalar(255, 255, 255), -1);
  }
  for (auto p : moving_point2D_) {
    // 创建点向量
    std::vector<cv::Point2f> src_point = {cv::Point2f(p.x(), p.y())};
    std::vector<cv::Point2f> dst_point;
    // 使用cv::transform变换单个点
    // cv::transform(src_point, dst_point, M);
    cv::perspectiveTransform(src_point, dst_point, M);
    // 绘制结果
    cv::circle(small_img2, dst_point[0], 2, cv::Scalar(255, 0, 255), -1);
  }
  roi = img2(cv::Rect(70, 30, small_img2.cols, small_img2.rows));
  small_img2.copyTo(roi);
}

void Visualization::DrawFrontalVec(cv::Mat& img, Mat3D& R, double reverse) {
  int w = 140, h = 150;
  double L = std::min(w / 2.5, h / 2.5);
  std::vector<Vec2D> point2d;
  cv::Mat small_img(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat small_img2(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
  Vec3D p = Vec3D(0, 0, L);

  Vec3D p1 = R.transpose() * p;
  cv::Point p0_ = cv::Point(w / 2, h / 2);
  cv::Point p1_ = cv::Point(p1(0), p1(2)) + p0_;
  if (reverse < 0) p1_ = cv::Point(-p1(0), p1(2)) + p0_;

  cv::arrowedLine(small_img, p0_, p1_, cv::Scalar(255, 255, 255), 2,
                  cv::LINE_AA, 0, 0.1);
  p1_ = cv::Point(p1(2), p1(1)) + p0_;
  if (reverse < 0) p1_ = cv::Point(p1(2), -p1(1)) + p0_;
  cv::arrowedLine(small_img2, p0_, p1_, cv::Scalar(255, 255, 255), 2,
                  cv::LINE_AA, 0, 0.1);

  cv::Mat roi =
      img(cv::Rect(2 * h - w + 70, 40 + 300, small_img.cols, small_img.rows));
  cv::flip(small_img, small_img, 1);
  cv::flip(small_img2, small_img2, 1);
  small_img.copyTo(roi);
  roi = img(cv::Rect(70, 40 + 300, small_img2.cols, small_img2.rows));
  small_img2.copyTo(roi);
}

float Visualization::CalculateError(std::vector<Vec2D> p1,
                                    std::vector<Vec2D> p2) {
  float error = 0;
  for (int i = 0; i < p1.size(); i++) {
    error += (p1[i] - p2[i]).norm();
  }
  error /= float(p1.size());
  return error;
}

void Visualization::DisplayFace3D() {
  cv::viz::Viz3d window("3D Face Landmarks Visualization");
  Mat3D R = Mat3D::Identity();
  R << -1, 0, 0, 0, -1, 0, 0, 0, 1;
  std::vector<cv::Point3f> fixed_points;
  std::vector<cv::Point3f> moving_points;
  for (auto p : fixed_point3D_opt_) {
    p = R * p;
    fixed_points.push_back(cv::Point3f(p(0), p(1), p(2)));
  }
  for (auto p : moving_point3D_opt_) {
    p = R * p;
    moving_points.push_back(cv::Point3f(p(0), p(1), p(2)));
  }
  cv::viz::WCloud cloud1(fixed_points, cv::viz::Color::green());
  cv::viz::WCloud cloud2(moving_points, cv::viz::Color::red());
  cloud1.setRenderingProperty(cv::viz::POINT_SIZE, 8.0);
  cloud2.setRenderingProperty(cv::viz::POINT_SIZE, 8.0);
  window.showWidget("fix points", cloud1);
  window.showWidget("moving points", cloud2);

  std::vector<cv::Point3f> left_eyebrow = {fixed_points[0], moving_points[0],
                                           fixed_points[1]};
  std::vector<cv::Point3f> right_eyebrow = {fixed_points[2], moving_points[1],
                                            fixed_points[3]};

  std::vector<cv::Point3f> left_eye_up = {fixed_points[4], moving_points[2],
                                          moving_points[3], fixed_points[5]};
  std::vector<cv::Point3f> left_eye_dn = {fixed_points[4], moving_points[4],
                                          moving_points[5], fixed_points[5]};
  std::vector<cv::Point3f> right_eye_up = {fixed_points[6], moving_points[6],
                                           moving_points[7], fixed_points[7]};
  std::vector<cv::Point3f> right_eye_dn = {fixed_points[6], moving_points[8],
                                           moving_points[9], fixed_points[7]};

  std::vector<cv::Point3f> noseup = {fixed_points[8], fixed_points[9]};
  std::vector<cv::Point3f> nosedn = {fixed_points[10], fixed_points[11],
                                     fixed_points[12]};
  std::vector<cv::Point3f> lip_up = {fixed_points[13], moving_points[10],
                                     moving_points[11], moving_points[12],
                                     fixed_points[14]};
  std::vector<cv::Point3f> lip_dn = {fixed_points[13], moving_points[14],
                                     moving_points[13], moving_points[15],
                                     fixed_points[14]};

  cv::viz::WPolyLine left_eyebrow_line(left_eyebrow, cv::viz::Color::blue());
  cv::viz::WPolyLine right_eyebrow_line(right_eyebrow, cv::viz::Color::blue());
  cv::viz::WPolyLine left_eyeup_line(left_eye_up, cv::viz::Color::blue());
  cv::viz::WPolyLine left_eyedn_line(left_eye_dn, cv::viz::Color::blue());
  cv::viz::WPolyLine right_eyeup_line(right_eye_up, cv::viz::Color::blue());
  cv::viz::WPolyLine right_eyedn_line(right_eye_dn, cv::viz::Color::blue());
  cv::viz::WPolyLine noseup_line(noseup, cv::viz::Color::blue());
  cv::viz::WPolyLine nosedn_line(nosedn, cv::viz::Color::blue());
  cv::viz::WPolyLine lipup_line(lip_up, cv::viz::Color::blue());
  cv::viz::WPolyLine lipdn_line(lip_dn, cv::viz::Color::blue());

  window.showWidget("left eyebrow", left_eyebrow_line);
  window.showWidget("right eyebrow", right_eyebrow_line);
  window.showWidget("left eye up", left_eyeup_line);
  window.showWidget("left eye dn", left_eyedn_line);
  window.showWidget("right eye up", right_eyeup_line);
  window.showWidget("right eye dn", right_eyedn_line);
  window.showWidget("nose_up", noseup_line);
  window.showWidget("nose_dn", nosedn_line);
  window.showWidget("lip up", lipup_line);
  window.showWidget("lip dn", lipdn_line);
  // 添加坐标系
  window.showWidget("Coordinate System", cv::viz::WCoordinateSystem(0.3));
  // 设置视角
  window.setViewerPose(cv::Affine3f().translate(cv::Vec3f(0.0f, 0.0f, 2.0f)));
  std::cout << "按ESC键退出3D窗口..." << std::endl;
  window.spin();
}

void Visualization::Display(cv::Mat& img) {
  cv::Mat img2 = img.clone();
  std::vector<Vec2D> fixed_show, fixed_show2;
  std::vector<Vec2D> moving_show, moving_show2;
  DistortTransform(cv_R_, cv_t_, fixed_point3D_, fixed_show2);
  DistortTransform(cv_R_, cv_t_, moving_point3D_, moving_show2);
  DistortTransform(opt_R_, opt_t_, fixed_point3D_opt_, fixed_show);
  DistortTransform(opt_R_, opt_t_, moving_point3D_opt_, moving_show);

  float opt_error = CalculateError(fixed_show, fixed_point2D_);
  opt_error = (opt_error + CalculateError(moving_show, moving_point2D_)) / 2.0;
  float cv_error = CalculateError(fixed_show2, fixed_point2D_);
  // cv_error = (cv_error + CalculateError(moving_show2, moving_point2D_))
  // / 2.0;

  for (int i = 0; i < fixed_point2D_.size(); i++) {
    Vec2D p0 = fixed_point2D_[i];
    Vec2D p1 = fixed_show[i];
    Vec2D p2 = fixed_show2[i];
    cv::circle(img, cv::Point(p0(0), p0(1)), 2, cv::Scalar(0, 255, 0), -1);
    cv::circle(img, cv::Point(p1(0), p1(1)), 2, cv::Scalar(0, 0, 255), -1);

    cv::circle(img2, cv::Point(p0(0), p0(1)), 2, cv::Scalar(0, 255, 0), -1);
    cv::circle(img2, cv::Point(p2(0), p2(1)), 2, cv::Scalar(0, 0, 255), -1);
  }
  for (int i = 0; i < moving_point2D_.size(); i++) {
    Vec2D p0 = moving_point2D_[i];
    Vec2D p1 = moving_show[i];
    Vec2D p2 = moving_show2[i];
    cv::circle(img, cv::Point(p0(0), p0(1)), 1, cv::Scalar(0, 255, 0), -1);
    cv::circle(img, cv::Point(p1(0), p1(1)), 1, cv::Scalar(0, 0, 255), -1);

    cv::circle(img2, cv::Point(p0(0), p0(1)), 1, cv::Scalar(0, 255, 0), -1);
    cv::circle(img2, cv::Point(p2(0), p2(1)), 1, cv::Scalar(0, 0, 255), -1);
  }

  Vec3D coor_chin = fixed_point3D_opt_[13] + fixed_point3D_opt_[14];
  coor_chin[1] = coor_chin[1] / 2.0 - 0.55, coor_chin[2] = coor_chin[2] / 2.0;
  // Mat3D opt_R = opt_R_.transpose() * frontal_R_;
  // Vec3D opt_t = opt_R_.transpose() * (frontal_t_ - opt_t_);
  // Mat3D cv_R = cv_R_.transpose() * frontal_R_;
  // Vec3D cv_t = cv_R_.transpose() * (frontal_t_ - cv_t_);
  Mat3D opt_R = frontal_R_.transpose() * opt_R_;
  Vec3D opt_t = frontal_R_.transpose() * (frontal_t_ - opt_t_);
  Mat3D cv_R = frontal_R_.transpose() * cv_R_;
  Vec3D cv_t = frontal_R_.transpose() * (frontal_t_ - cv_t_);

  Quat q = Eigen::Quaterniond(opt_R);
  opt_R = q.normalized().toRotationMatrix();
  q = Eigen::Quaterniond(cv_R);
  cv_R = q.normalized().toRotationMatrix();

  Draw3DCoordinateSystem(img, coor_chin, 0.5, opt_R_, opt_t_);
  Draw3DCoordinateSystem(img2, coor_chin, 0.5, cv_R_, cv_t_);

  cv::Mat r_mat, trans_vec;
  cv::Mat r_mat2, trans_vec2;
  cv::eigen2cv(opt_R, r_mat);
  cv::eigen2cv(cv_R, r_mat2);
  cv::eigen2cv(opt_t, trans_vec);
  cv::eigen2cv(cv_t, trans_vec2);
  std::vector<cv::Mat> matrices = {r_mat, trans_vec};
  std::vector<cv::Mat> matrices2 = {r_mat2, trans_vec2};
  cv::Mat pose_mat, pose_mat2;
  cv::hconcat(matrices, pose_mat);
  cv::hconcat(matrices2, pose_mat2);
  cv::Mat c_m, r_m, t_v, r_mx, r_my, r_mz;
  std::vector<double> euler_angles, euler_angles2;
  cv::decomposeProjectionMatrix(pose_mat, c_m, r_m, t_v, r_mx, r_my, r_mz,
                                euler_angles);

  cv::decomposeProjectionMatrix(pose_mat2, c_m, r_m, t_v, r_mx, r_my, r_mz,
                                euler_angles2);

  Vec3D euler1 = opt_R.eulerAngles(0, 1, 2) * 180 / 3.1415926;  // XYZ顺序
  Vec3D euler2 = cv_R.eulerAngles(0, 1, 2) * 180 / 3.1415926;   // XYZ顺序

  euler1 << euler_angles[0], euler_angles[1], euler_angles[2];
  euler2 << euler_angles2[0], euler_angles2[1], euler_angles2[2];

  Vec3D p0 = Vec3D(0, 0, 1);
  Vec3D p1 = opt_R * p0;
  Vec3D p2 = cv_R * p0;
  double yaw1 = std::atan(p1(0) / p1(2)) * 180 / 3.1415926;
  double yaw2 = std::atan(p2(0) / p2(2)) * 180 / 3.1415926;
  double pitch1 = std::atan(-p1(1) / std::sqrt(p1(0) * p1(0) + p1(2) * p1(2))) * 180 / 3.1415926;
  double pitch2 = std::atan(-p2(1) / std::sqrt(p2(0) * p2(0) + p2(2) * p2(2))) * 180 / 3.1415926;

  std::string euler_str1 = "opt_R: " + std::to_string(euler1[0]) + ", " +
                           std::to_string(euler1[1]) + ", " +
                           std::to_string(euler1[2]);
  std::string euler_str2 = "cv_R: " + std::to_string(euler2[0]) + ", " +
                           std::to_string(euler2[1]) + ", " +
                           std::to_string(euler2[2]);
  std::string trans_str1 = "opt_t: " + std::to_string(opt_t[0]) + ", " +
                           std::to_string(opt_t[1]) + ", " +
                           std::to_string(opt_t[2]);
  std::string trans_str2 = "cv_t: " + std::to_string(cv_t[0]) + ", " +
                           std::to_string(cv_t[1]) + ", " +
                           std::to_string(cv_t[2]);
  cv::putText(img, "joint optimization", cv::Point(640, 50),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
  cv::putText(img2, "cvPnP optimization", cv::Point(640, 50),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
  cv::putText(img, euler_str1, cv::Point(400, 70), cv::FONT_HERSHEY_SIMPLEX,
              0.8, cv::Scalar(255, 255, 255), 2);
  cv::putText(img, trans_str1, cv::Point(400, 110), cv::FONT_HERSHEY_SIMPLEX,
              0.8, cv::Scalar(255, 255, 255), 2);
  cv::putText(img, "reprojection error: " + std::to_string(opt_error),
              cv::Point(400, 145), cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(255, 255, 255), 2);
  cv::putText(
      img, "yaw-pitch: " + std::to_string(yaw1) + ", " + std::to_string(pitch1),
      cv::Point(400, 180), cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(255, 255, 255), 2);
  cv::putText(img, "frame_id : " + std::to_string(frame_id_),
              cv::Point(1000, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(255, 255, 255), 2);
  cv::putText(img, "number : " + std::to_string(number_), cv::Point(1000, 110),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
  if (!is_calibrate_) {
    cv::putText(img, "uncalibrate !!!", cv::Point(1000, 140),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
  } else {
    cv::putText(img, "calibrate success", cv::Point(1000, 140),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
  }
  if (!is_reconstruct_) {
    cv::putText(img, "unreconstruct !!!", cv::Point(1000, 170),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
  } else {
    cv::putText(img, "reconstruct success", cv::Point(1000, 170),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
  }
  cv::putText(img2, euler_str2, cv::Point(400, 70), cv::FONT_HERSHEY_SIMPLEX,
              0.8, cv::Scalar(255, 255, 255), 2);
  cv::putText(img2, trans_str2, cv::Point(400, 110), cv::FONT_HERSHEY_SIMPLEX,
              0.8, cv::Scalar(255, 255, 255), 2);
  cv::putText(img2, "reprojection error: " + std::to_string(cv_error),
              cv::Point(400, 145), cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(255, 255, 255), 2);

  cv::putText(
      img2,
      "yaw-pitch: " + std::to_string(yaw2) + ", " + std::to_string(pitch2),
      cv::Point(400, 180), cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(255, 255, 255), 2);

  DrawAlignedPoints(img, img2, fixed_point3D_, moving_point3D_opt_);
  DrawFrontalVec(img, opt_R);
  DrawFrontalVec(img2, cv_R);

  cv::Mat join_mat = cv::Mat::zeros(img.rows * 2, img.cols, img.type());
  cv::Mat roi = join_mat(cv::Rect(0, 0, img.cols, img.rows));
  img.copyTo(roi);
  roi = join_mat(cv::Rect(0, img.rows, img.cols, img.rows));
  img2.copyTo(roi);
  // cv::Mat resize_img;
  // cv::resize(join_mat, resize_img, cv::Size(), 0.5, 0.5);
  // cv::imwrite("/home/cidi/workspace/DMS/to_xiewei/all_2_result/" +
  //                 std::to_string(frame_id_) + ".jpg",
  //             join_mat);
  img = join_mat;
  cv::namedWindow("optimization", cv::WINDOW_NORMAL);
  cv::imshow("optimization", join_mat);
  cv::waitKey(1);

  std::cout << "frame_id_: " << frame_id_ << std::endl;

  frame_id_++;
}

FACEPOSE_END