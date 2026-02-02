#include "common.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/viz.hpp>

FACEPOSE_BEGIN

class Visualization {
public:
  Visualization() = default;
  ~Visualization() = default;
  inline void SetParameters(Mat3D &K, Vec5D &D) {
    K_ = K;
    D_ = D;
  };

  inline void SetInitVal(bool is_calibrate, bool is_reconstruct) {
    is_calibrate_ = is_calibrate;
    is_reconstruct_ = is_reconstruct;
  }

  inline void SetFrontalPose(Mat3D &R, Vec3D &t) {
    frontal_R_ = R;
    frontal_t_ = t;
  }

  inline void SetDetectKeyPoints(std::vector<Vec2D> &fixed_point2D,
                                 std::vector<Vec2D> &moving_point2D) {
    fixed_point2D_ = fixed_point2D;
    moving_point2D_ = moving_point2D;
  };

  inline void SetJointOptimization(std::vector<Vec3D> &fixed_point3D,
                                   std::vector<Vec3D> &moving_point3D, Mat3D &R,
                                   Vec3D &t) {
    fixed_point3D_opt_ = fixed_point3D;
    moving_point3D_opt_ = moving_point3D;
    opt_R_ = R;
    opt_t_ = t;
  };
  inline void SetOpencvOptimization(std::vector<Vec3D> &fixed_point3D,
                                    std::vector<Vec3D> &moving_point3D,
                                    Mat3D &R, Vec3D &t) {
    fixed_point3D_ = fixed_point3D;
    moving_point3D_ = moving_point3D;
    cv_R_ = R;
    cv_t_ = t;
  };

  inline void SetNumber(int number) { number_ = number; };

public:
  void Display(cv::Mat &img);
  void DisplayFace3D();

private:
  void DistortTransform(Mat3D &R, Vec3D &t, std::vector<Vec3D> &in,
                        std::vector<Vec2D> &out);
  void Draw3DCoordinateSystem(cv::Mat &image, const Vec3D &world_origin,
                              double axis_length, const Mat3D &R,
                              const Vec3D &t);
  void DrawAlignedPoints(cv::Mat &img, cv::Mat &img2,
                         std::vector<Vec3D> &point1,
                         std::vector<Vec3D> &point2);
  void DrawAlignedPoints(cv::Mat &img, Mat3D &R, Vec3D &t);
  void DrawFrontalVec(cv::Mat &img, Mat3D &R, double reverse = 1);
  float CalculateError(std::vector<Vec2D> p1, std::vector<Vec2D> p2);

private:
  bool is_calibrate_{false};
  bool is_reconstruct_{false};

private:
  Mat3D K_;
  Vec5D D_;
  Mat3D opt_R_;
  Vec3D opt_t_;
  Mat3D cv_R_;
  Vec3D cv_t_;
  int frame_id_{1};
  int number_{1};
  Mat3D frontal_R_;
  Vec3D frontal_t_;

private:
  std::vector<Vec2D> fixed_point2D_;
  std::vector<Vec2D> moving_point2D_;
  std::vector<Vec3D> fixed_point3D_;
  std::vector<Vec3D> moving_point3D_;
  std::vector<Vec3D> fixed_point3D_opt_;
  std::vector<Vec3D> moving_point3D_opt_;
};

FACEPOSE_END