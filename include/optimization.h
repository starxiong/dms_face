/*
 @Editors:  xiewei
 @E-mail: xie.wei@cidi.ai
 Optimization for face keypoint by ceres
 FaceModelOptimization for face model by ceres
*/

#pragma once
#include <cmath>

#include "common.h"

FACEPOSE_BEGIN

class Optimization {
 public:
  Optimization(const Vec2D& point2D, const Vec3D& point3D, const Mat3D& K,
               const Mat3D& R, const Vec3D& t, const bool is_moving,
               const int moving_id)
      : point2D_(point2D),
        point3D_(point3D),
        K_(K),
        R_(R),
        t_(t),
        is_moving_(is_moving),
        moving_id_(moving_id) {}
  template <typename T>
  bool operator()(const T* const lie_r, const T* const lie_t,
                  const T* const delta, T* residuals) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> w(lie_r);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> v(lie_t);

    Eigen::Matrix<T, 3, 3> R = R_.template cast<T>();
    Eigen::Matrix<T, 3, 1> t = t_.template cast<T>();
    Eigen::Matrix<T, 3, 3> K = K_.template cast<T>();

    T theta = w.norm();
    T sin_val = sin(theta);
    T minus = T(1) - cos(theta);
    Eigen::Matrix<T, 3, 3> skew;
    Eigen::Matrix<T, 3, 3> deltaR;
    skew << T(0), -w(2), w(1), w(2), T(0), -w(0), -w(1), w(0), T(0);
    Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity();

    if (theta < T(1e-9)) {
      deltaR = I + skew + (skew * skew) * T(0.5);
    } else {
      Eigen::AngleAxis<T> angle_axis(theta, w / theta);
      deltaR = angle_axis.toRotationMatrix();
    }

    Eigen::Matrix<T, 3, 3> Rd = deltaR * R;
    Eigen::Matrix<T, 3, 1> dt = deltaR * t + v;
    Eigen::Matrix<T, 3, 1> source;
    Eigen::Matrix<T, 2, 1> target;

    target << T(point2D_(0)), T(point2D_(1));
    source << T(point3D_(0)), T(point3D_(1)), T(point3D_(2));
    if (is_moving_) {
      source << T(point3D_(0)), T(point3D_(1) + delta[moving_id_]),
          T(point3D_(2));
    }

    Eigen::Matrix<T, 3, 1> po = K * (Rd * source + dt);
    if (ceres::abs(po(2)) < T(1e-9)) {
      residuals[0] = T(0);
      residuals[1] = T(0);
    } else {
      residuals[0] = (po(0)) / po(2) - target(0);
      residuals[1] = (po(1)) / po(2) - target(1);
    }

    return true;
  }
  static ceres::CostFunction* Create(const Vec2D& point2D, const Vec3D& point3D,
                                     const Mat3D& K, const Mat3D& R,
                                     const Vec3D& t, const bool is_moving,
                                     const int moving_id) {
    auto cost_function =
        new ceres::AutoDiffCostFunction<Optimization, 2, 3, 3, 16>(
            new Optimization(point2D, point3D, K, R, t, is_moving, moving_id));
    return cost_function;
  }

 private:
  Mat3D K_;
  Mat3D R_;
  Vec3D t_;
  Vec2D point2D_;
  Vec3D point3D_;
  bool is_moving_;
  int moving_id_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class FaceModelOptimization {
 public:
  FaceModelOptimization(const Vec2D& point2D, const Vec3D& point3D,
                        const Mat3D& K, const Mat3D& R, const Vec3D& t)
      : point2D_(point2D), point3D_(point3D), K_(K), R_(R), t_(t) {}
  template <typename T>
  bool operator()(const T* const delta, T* residuals) const {
    Eigen::Matrix<T, 3, 3> R = R_.template cast<T>();
    Eigen::Matrix<T, 3, 1> t = t_.template cast<T>();
    Eigen::Matrix<T, 3, 3> K = K_.template cast<T>();

    Eigen::Matrix<T, 3, 1> source;
    Eigen::Matrix<T, 2, 1> target;

    target << T(point2D_(0)), T(point2D_(1));
    source << T(point3D_(0)) + delta[0], T(point3D_(1)) + delta[1],
        T(point3D_(2)) + delta[2];

    Eigen::Matrix<T, 3, 1> po = K * (R * source + t);
    residuals[0] = (po(0)) / po(2) - target(0);
    residuals[1] = (po(1)) / po(2) - target(1);

    return true;
  }
  static ceres::CostFunction* Create(const Vec2D& point2D, const Vec3D& point3D,
                                     const Mat3D& K, const Mat3D& R,
                                     const Vec3D& t) {
    auto cost = new ceres::AutoDiffCostFunction<FaceModelOptimization, 2, 3>(
        new FaceModelOptimization(point2D, point3D, K, R, t));
    return cost;
  }

 private:
  Mat3D K_;
  Mat3D R_;
  Vec3D t_;
  Vec2D point2D_;
  Vec3D point3D_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

FACEPOSE_END