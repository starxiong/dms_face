#include "../include/pose_calibrator.h"

#include <algorithm>
#include <cmath>
#include <numeric>

FACEPOSE_BEGIN

void FrontalPoseCalibrator::Init(double angle_threshold) {
  std::random_device rd;
  rng_ = std::mt19937(rd());
  iterations_ = 1000;
  angle_threshold_ = angle_threshold;
}

Quat FrontalPoseCalibrator::ComputeQuatAverage(const std::vector<Quat>& quats) {
  if (quats.empty()) return Quat::Identity();
  // 确保所有四元数在同一半球
  std::vector<Quat> adjusted = quats;
  for (size_t i = 1; i < adjusted.size(); ++i) {
    if (adjusted[0].coeffs().dot(adjusted[i].coeffs()) < 0) {
      adjusted[i].coeffs() = -adjusted[i].coeffs();
    }
  }

  Eigen::MatrixXd Q(4, adjusted.size());
  for (size_t i = 0; i < adjusted.size(); ++i) {
    Q.col(i) = adjusted[i].coeffs();
  }
  Mat4D M = Q * Q.transpose();
  Eigen::SelfAdjointEigenSolver<Mat4D> eigensolver(M);
  if (eigensolver.info() != Eigen::Success) {
    return Quat::Identity();
  }

  Vec4D eigenvec = eigensolver.eigenvectors().col(3);
  Quat avg_q(eigenvec(3), eigenvec(0), eigenvec(1), eigenvec(2));
  avg_q.normalize();

  return avg_q;
}
Vec3D FrontalPoseCalibrator::ComputeTransAverage(
    const std::vector<Vec3D>& trans) {
  if (trans.empty()) {
    return Vec3D::Zero();
  }

  std::vector<double> x_vals, y_vals, z_vals;
  x_vals.reserve(trans.size());
  y_vals.reserve(trans.size());
  z_vals.reserve(trans.size());

  for (const auto& t : trans) {
    x_vals.push_back(t.x());
    y_vals.push_back(t.y());
    z_vals.push_back(t.z());
  }

  auto median = [](std::vector<double>& v) -> double {
    if (v.empty()) return 0.0;
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    double median_val = v[n];

    if (v.size() % 2 == 0) {
      std::nth_element(v.begin(), v.begin() + n - 1, v.end());
      median_val = (median_val + v[n - 1]) / 2.0;
    }

    return median_val;
  };

  Vec3D result;
  result.x() = median(x_vals);
  result.y() = median(y_vals);
  result.z() = median(z_vals);

  return result;
}

double FrontalPoseCalibrator::QuaternionDistance(const Quat& q1,
                                                 const Quat& q2) {
  Quat q2_adj = q2;
  if (q1.coeffs().dot(q2.coeffs()) < 0) {
    q2_adj.coeffs() = -q2.coeffs();
  }

  double dot = std::abs(q1.coeffs().dot(q2_adj.coeffs()));
  dot = std::clamp(dot, -1.0, 1.0);
  return 2.0 * std::acos(dot);
  
  // Quat q_rel = q1.conjugate() * q2;
  // if (q_rel.w() < 0) {
  //   q_rel.coeffs() = -q_rel.coeffs();
  // }
  // double angle = 2.0 * std::atan2(q_rel.vec().norm(), std::abs(q_rel.w()));
  // return std::min(angle, 2.0 * M_PI - angle);
}

std::vector<bool> FrontalPoseCalibrator::ComputeInliers(
    const std::vector<Quat>& quats, const std::vector<Vec3D>& trans,
    const Quat& q_center, const Vec3D& t_center) {
  std::vector<bool> inliers(quats.size(), false);

  for (size_t i = 0; i < quats.size(); ++i) {
    double rot_dist = QuaternionDistance(quats[i], q_center);
    double trans_dist = (trans[i] - t_center).norm();

    double normalized_dist = rot_dist / angle_threshold_ + trans_dist / 0.1;

    if (rot_dist < angle_threshold_) {
      inliers[i] = true;
    }
  }

  return inliers;
}

void FrontalPoseCalibrator::RansacEstimate(std::vector<Quat>& quats,
                                           std::vector<Vec3D>& trans) {
  int size = quats.size();
  std::uniform_int_distribution<size_t> dist(0, size - 1);
  for (int i = 0; i < iterations_; i++) {
    std::vector<size_t> sample_indices;
    while (sample_indices.size() < 3) {
      size_t idx = dist(rng_);
      if (std::find(sample_indices.begin(), sample_indices.end(), idx) ==
          sample_indices.end()) {
        sample_indices.push_back(idx);
      }
    }

    std::vector<Quat> sample_quats;
    std::vector<Vec3D> sample_trans;
    for (auto idx : sample_indices) {
      sample_quats.push_back(quats[idx]);
      sample_trans.push_back(trans[idx]);
    }

    Quat q_center = ComputeQuatAverage(sample_quats);
    Vec3D t_center = ComputeTransAverage(sample_trans);

    std::vector<bool> inliers = ComputeInliers(quats, trans, q_center, t_center);

    int num_inliers = std::count(inliers.begin(), inliers.end(), true);
    if (num_inliers > result_.num_inliers) {
      result_.num_inliers = num_inliers;
      std::vector<Quat> inlier_quats;
      std::vector<Vec3D> inlier_trans;
      for (size_t i = 0; i < size; ++i) {
        if (inliers[i]) {
          inlier_quats.push_back(quats[i]);
          inlier_trans.push_back(trans[i]);
        }
      }
      if (!inlier_quats.empty()) {
        result_.q = ComputeQuatAverage(inlier_quats);
        result_.t = ComputeTransAverage(inlier_trans);
        result_.inliners = inliers;
        result_.confidence = static_cast<double>(num_inliers) / size;
      }
    } 
  }
}

void FrontalPoseCalibrator::Calibrate(std::vector<Mat3D>& rots,
                                      std::vector<Vec3D>& trans) {
  std::vector<Quat> quats = std::vector<Quat>(rots.size());
  for (int i = 0; i < rots.size(); i++) quats[i] = Quat(rots[i]);
  RansacEstimate(quats, trans);
}

void FrontalPoseCalibrator::GetResult(Mat3D& R, Vec3D& t) {
  R = result_.q.toRotationMatrix();
  t = result_.t;
}

FACEPOSE_END
