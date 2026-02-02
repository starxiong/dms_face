/*
 @Editors:  xiewei
 @E-mail: xie.wei@cidi.ai
 param, struct
*/

#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
// #include <execution>  // 需要C++17及以上
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "../dependency/ceres/include/ceres/ceres.h"
#include "../dependency/ceres/include/ceres/rotation.h"
#include "defines.h"
#include "log.h"

FACEPOSE_BEGIN

#define FACEPOSE_OK (0x00000000)                // 正常返回
#define FACEPOSE_FAILED (0x00000001)            // 异常返回
#define FACEPOSE_ERR_LOAD_CONFIG (0x00000002)   // 配置文件读取异常
#define FACEPOSE_ERR_PARSE_CONFIG (0x00000003)  // 配置文件解析异常
#define FACEPOSE_ERR_LOAD_FILE (0x00000004)     // 文件加载异常
#define FACEPOSE_ERR_TIMEOUT (0x00000005)       // 超时异常

struct MacroMap {
  std::unordered_map<std::string, int> macro;
  MacroMap() {
    macro["FACEPOSE_OK"] = FACEPOSE_OK;
    macro["FACEPOSE_FAILED"] = FACEPOSE_FAILED;
    macro["FACEPOSE_ERR_LOAD_FILE"] = FACEPOSE_ERR_LOAD_FILE;
    macro["FACEPOSE_ERR_LOAD_CONFIG"] = FACEPOSE_ERR_LOAD_CONFIG;
    macro["FACEPOSE_ERR_PARSE_CONFIG"] = FACEPOSE_ERR_PARSE_CONFIG;
    macro["FACEPOSE_ERR_TIMEOUT"] = FACEPOSE_ERR_TIMEOUT;
  }
};

#define MACRO(container, var)                            \
  ({                                                     \
    auto _map = &(container);                            \
    std::string _state = "Unkonw";                       \
    for (const auto& _pair : *_map) {                    \
      if (_pair.second == var) {                         \
        _state = _map->begin()->first + ":" + __FILE__ + \
                 ", func:" + __FUNCTION__ +              \
                 ", line:" + std::to_string(__LINE__);   \
        break;                                           \
      }                                                  \
    }                                                    \
    _state;                                              \
  })

#define CHECK_ERROR(container, val)   \
  do {                                \
    if ((val) != FACEPOSE_OK) {      \
      AINFO << MACRO(container, val); \
    }                                 \
  } while (0)

#define CHECK_RUNTIME(container)                                       \
  do {                                                                 \
    static auto last_time = std::chrono::high_resolution_clock::now(); \
    auto now = std::chrono::high_resolution_clock::now();              \
    std::chrono::duration<double> interval = now - last_time;          \
    if ((interval.count()) > MAX_RUNTIME) {                            \
      AINFO << MACRO(container, FACEPOSE_ERR_TIMEOUT);                \
    }                                                                  \
    last_time = now;                                                   \
  } while (0)

typedef Eigen::Vector2d Vec2D;
typedef Eigen::Vector3d Vec3D;
typedef Eigen::Vector4d Vec4D;
typedef Eigen::Matrix3d Mat3D;
typedef Eigen::Matrix4d Mat4D;
typedef Eigen::MatrixXd MatXD;
typedef Eigen::VectorXd VecXD;
typedef Eigen::Quaterniond Quat;
typedef Eigen::AngleAxisd AxisD;
typedef Eigen::Matrix<double, 5, 1> Vec5D;
typedef Eigen::Matrix<double, 6, 1> Vec6D;
typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> Cloud;
typedef std::vector<Point, Eigen::aligned_allocator<Point>> PointVec;


struct Point2D {
  double x;
  double y;
  bool valid;
};

struct Face2D{
  // Point2D left_eyebrow[3];  // 左边眉毛从左到右
  // Point2D right_eyebrow[3]; // 右边眉毛从左到右  
  // Point2D left_eye[6];      // 左边眼睛从左到右,形成一个环 0-<1-2>-3-<4-5>-0, 其中<>表示中间点
  // Point2D right_eye[6];     // 右边眼睛从左到右,形成一个环 0-<1-2>-3-<4-5>-0, 其中<>表示中间点
  // Point2D nose[6];          // 鼻梁从上到下3个点,鼻尖左右3个点 
  // Point2D lips[8];          // 嘴巴内嘴唇从左上到右下,形成一个环 0-<1-2-3>-4-<5-6-7>-0, 其中<>表示中间点

  
};

struct RGB {
  unsigned char r;
  unsigned char g;
  unsigned char b;
};

struct CalibrationResult {
  Quat q;
  Vec3D t;
  double confidence;
  std::vector<bool> inliners;
  int num_inliers;
  CalibrationResult() : confidence(0.0), num_inliers(0) {}
};

FACEPOSE_END