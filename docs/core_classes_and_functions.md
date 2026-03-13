# 工程核心类与主要函数提取

本文按“职责 + 主要函数”提取该工程的核心类，便于快速定位代码入口。

## 1. `PoseEstimation`（核心主流程类）

**文件**：`include/pose_estimation.h`、`src/pose_estimation.cc`

**职责**：
- 单帧2D关键点解析与位姿估计。
- 多帧累积后的人脸3D模型重建。
- 正脸参考位姿（frontal pose）自标定。
- 结果可视化与状态管理。

**主要函数**：
- `int Init(std::string param_path)`：加载相机参数、初始位姿、3D模板点与优化约束配置。
- `int Process(std::vector<Point2D> &points)`：单帧入口；执行关键点解析、位姿/形变联合优化、标定与重建触发。
- `void GetPose(Mat3D &R, Vec3D &t)`：输出当前估计位姿。
- `void Display(cv::Mat &img)`：输出可视化叠加结果。
- `void Reset()`：重置内部状态与缓存。

**关键内部函数（建议优先阅读）**：
- `ParseFace2D(...)`：将32点映射为固定点与非刚性点。
- `SolverFace(...)`：Ceres 联合优化（位姿增量 + 非刚性点位移）。
- `CalibrateFrontedFace()`：采样历史位姿并触发正脸自标定。
- `ReconstructFace3D()`：多姿态样本累计与重建触发。
- `SolverFaceModel()` / `UpdateFaceModel()`：3D模型参数优化与迭代更新。

---

## 2. `FrontalPoseCalibrator`（正脸位姿标定类）

**文件**：`include/pose_calibrator.h`、`src/pose_calibrator.cc`

**职责**：
- 对历史旋转/平移做鲁棒估计，得到“朝前参考位姿”。

**主要函数**：
- `void Init(double angle_threshold)`：初始化 RANSAC 参数。
- `void Calibrate(std::vector<Mat3D>& rots, std::vector<Vec3D>& trans)`：执行标定主流程。
- `void GetResult(Mat3D& R, Vec3D& t)`：输出标定结果。

**关键内部函数**：
- `RansacEstimate(...)`：RANSAC 主循环。
- `ComputeQuatAverage(...)`：旋转四元数平均。
- `ComputeTransAverage(...)`：平移分量中位数估计。
- `ComputeInliers(...)`、`QuaternionDistance(...)`：内点判定与旋转距离计算。

---

## 3. `Visualization`（可视化类）

**文件**：`include/visualization.h`、`src/visualization.cc`

**职责**：
- 绘制2D重投影对比、姿态坐标轴。
- 显示3D关键点/连线模型（OpenCV viz）。

**主要函数**：
- `void SetParameters(Mat3D &K, Vec5D &D)`：设置相机参数。
- `void SetDetectKeyPoints(...)`：设置检测到的2D关键点。
- `void SetJointOptimization(...)`：设置联合优化结果。
- `void SetOpencvOptimization(...)`：设置OpenCV PnP对照结果。
- `void SetFrontalPose(...)`：设置正脸参考位姿。
- `void Display(cv::Mat &img)`：2D可视化输出。
- `void DisplayFace3D()`：3D可视化窗口。

**常用内部函数**：
- `DistortTransform(...)`：3D点到2D投影转换。
- `Draw3DCoordinateSystem(...)`、`DrawFrontalVec(...)`：姿态辅助绘制。
- `CalculateError(...)`：可视化对齐误差统计。

---

## 4. `Optimization`（单帧优化残差）

**文件**：`include/optimization.h`

**职责**：
- 定义 `SolverFace` 使用的 Ceres 误差项（重投影误差 + 方向约束）。

**主要函数**：
- `bool operator()(...) const`：残差计算核心。
- `static ceres::CostFunction* Create(...)`：构造 Ceres cost function。

---

## 5. `FaceModelOptimization`（多帧模型优化残差）

**文件**：`include/optimization.h`

**职责**：
- 定义 `SolverFaceModel` 使用的 3D模板点优化误差项。
- 支持按维度优化与对称约束。

**主要函数**：
- `bool operator()(const T* const delta, T* residuals) const`：模型点优化残差。
- `static ceres::CostFunction* Create(...)`：构造 Ceres cost function。

---

## 6. `TransConstraint`（平移约束残差类）

**文件**：`include/optimization.h`

**职责**：
- 提供非刚性点相对约束残差（当前版本为可选组件）。

**主要函数**：
- `bool operator()(const T* const delta, T* residuals) const`
- `static ceres::CostFunction* Create(...)`

---

## 快速阅读建议（按顺序）
1. `PoseEstimation::Process`（看主流程如何串起各模块）
2. `PoseEstimation::SolverFace`（看单帧联合优化）
3. `PoseEstimation::ReconstructFace3D` + `UpdateFaceModel`（看多帧重建）
4. `FrontalPoseCalibrator::Calibrate`（看正脸标定）
5. `Visualization::Display` / `DisplayFace3D`（看结果展示）
