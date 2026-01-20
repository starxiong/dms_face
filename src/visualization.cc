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
          cv::arrowedLine(image, p1, p2, colors[i - 1], 1, cv::LINE_AA, 0, 0.1);
          label_pos =
              cv::Point(static_cast<int>(p2.x + 5), static_cast<int>(p2.y));
          break;
      }
      cv::putText(image, label, label_pos, cv::FONT_HERSHEY_SIMPLEX, 0.2,
                  colors[i - 1], 1);
    }
  }
}

void Visualization::DrawAlignedPoints(cv::Mat& img, std::vector<Vec3D>& point1,
                                      std::vector<Vec3D>& point2) {
  int width = 250, height = 250;
  std::vector<Vec2D> point2d;
  Mat3D R = Mat3D::Identity();
  R = AxisD(3.14159, Vec3D::UnitZ()) * AxisD(0, Vec3D::UnitY()) *
      AxisD(0, Vec3D::UnitX());
  Vec3D trans = Vec3D(width / 2, width / 2, 0);
  cv::Mat small_img(width, height, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 0; i < point1.size(); i++) {
    Vec3D p3d = R * point1[i];
    double x = p3d(0) * width / 2 + width / 2;
    double y = p3d(1) * width / 2 + height / 2;
    cv::circle(small_img, cv::Point(x, y), 1, cv::Scalar(255, 255, 255), -1);
  }
  for (int i = 0; i < point2.size(); i++) {
    Vec3D p3d = R * point2[i];
    double x = p3d(0) * width / 2 + width / 2;
    double y = p3d(1) * width / 2 + height / 2;
    cv::circle(small_img, cv::Point(x, y), 1, cv::Scalar(255, 0, 255), -1);
  }

  cv::Mat roi =
      img(cv::Rect(img.cols - width - 1, 50, small_img.cols, small_img.rows));
  small_img.copyTo(roi);
}

void Visualization::DrawAlignedPoints(cv::Mat& img, Mat3D& R, Vec3D& t) {
  // 核心思想：从当前姿态到正脸姿态的单应性变换

  // 当前姿态的投影：P_cam = R * P_face + t
  // 正脸姿态的投影：P_cam_front = I * P_face + t

  // 单应性矩阵：H = K * (I) * (R - t * n^T / d)^{-1} * K^{-1}
  // 简化：H = K * (R - t * n^T / d)^{-1} * K^{-1}

  double face_plane_distance = 1.0;
  Vec3D face_plane_normal = Vec3D(0, 0, 1);
  Mat3D K_inv = K_.inverse();

  // 计算平面单应性矩阵
  Mat3D H = R - (t * face_plane_normal.transpose()) / face_plane_distance;
  Mat3D H_inv = H.inverse();

  // 完整的单应性矩阵：从当前图像到正脸图像
  Mat3D H_total = K_ * H_inv * K_inv;
  for (auto p1 : fixed_point2D_) {
    // 应用变换
    Vec3D p1_hom(p1.x(), p1.y(), 1.0);
    Vec3D p2_hom = H_total * p1_hom;

    // 齐次坐标归一化
    Vec2D p2 = Vec2D(p2_hom.x() / p2_hom.z(), p2_hom.y() / p2_hom.z());

    cv::circle(img, cv::Point(p2(0), p2(1)), 1, cv::Scalar(0, 255, 0), -1);
  }
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
  // cv_error = (cv_error + CalculateError(moving_show2, moving_point2D_)) / 2.0;

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

  Draw3DCoordinateSystem(img, fixed_point3D_opt_.back(), 0.5, opt_R_, opt_t_);
  Draw3DCoordinateSystem(img2, fixed_point3D_.back(), 0.5, cv_R_, cv_t_);

  Vec3D euler1 = opt_R_.eulerAngles(2, 1, 0) * 180 / 3.1415926;  // ZYX顺序
  Vec3D euler2 = cv_R_.eulerAngles(2, 1, 0) * 180 / 3.1415926;   // ZYX顺序
  std::string euler_str1 = "opt_R: " + std::to_string(euler1[0]) + ", " +
                           std::to_string(euler1[1]) + ", " +
                           std::to_string(euler1[2]);
  std::string euler_str2 = "cv_R: " + std::to_string(euler2[0]) + ", " +
                           std::to_string(euler2[1]) + ", " +
                           std::to_string(euler2[2]);
  std::string trans_str1 = "opt_t: " + std::to_string(opt_t_[0]) + ", " +
                           std::to_string(opt_t_[1]) + ", " +
                           std::to_string(opt_t_[2]);
  std::string trans_str2 = "cv_t: " + std::to_string(cv_t_[0]) + ", " +
                           std::to_string(cv_t_[1]) + ", " +
                           std::to_string(cv_t_[2]);
  cv::putText(img, euler_str1, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(255, 255, 255), 2);
  cv::putText(img, trans_str1, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX,
              0.8, cv::Scalar(255, 255, 255), 2);
  cv::putText(img, "reprojection error: " + std::to_string(opt_error),
              cv::Point(10, 145), cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(255, 255, 255), 2);
  cv::putText(img2, euler_str2, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX,
              0.8, cv::Scalar(255, 255, 255), 2);
  cv::putText(img2, trans_str2, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX,
              0.8, cv::Scalar(255, 255, 255), 2);
  cv::putText(img2, "reprojection error: " + std::to_string(cv_error),
              cv::Point(10, 145), cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(255, 255, 255), 2);

  DrawAlignedPoints(img, fixed_point3D_, moving_point3D_);
  DrawAlignedPoints(img2, cv_R_, cv_t_);

  cv::Mat join_mat = cv::Mat::zeros(img.rows, img.cols * 2, img.type());
  cv::Mat roi = join_mat(cv::Rect(0, 0, img.cols, img.rows));
  img.copyTo(roi);
  roi = join_mat(cv::Rect(img.cols, 0, img.cols, img.rows));
  img2.copyTo(roi);
  cv::imwrite("result/" + std::to_string(frame_id_) + ".jpg", join_mat);
  cv::namedWindow("optimization", cv::WINDOW_NORMAL);
  cv::imshow("optimization", join_mat);
  cv::waitKey(1);

  frame_id_++;
}

FACEPOSE_END