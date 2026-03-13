# 单帧到多帧重建：数据流图

```mermaid
flowchart TD
    %% =========================
    %% 输入层
    %% =========================
    A[输入视频帧 frame_t] --> B[读取32点2D关键点]
    B --> C[ParseFace2D\n拆分固定点/非刚性点]

    %% =========================
    %% 单帧优化层
    %% =========================
    C --> D[SolverFace\nCeres联合优化]
    D --> D1[优化变量:\nΔR, Δt, moving_delta]
    D --> D2[输出:\n当前帧位姿 R_t, t_t]
    D --> D3[输出:\n当前帧3D点(固定+非刚性)]

    D2 --> E[CalculateError\n重投影误差]
    E --> F[单帧结果可视化 Display]

    %% =========================
    %% 正脸自标定分支
    %% =========================
    D2 --> G[CalibrateFrontedFace\n按间隔采样姿态]
    G --> H{采样数量 >= calibrate_num?}
    H -- 否 --> G
    H -- 是 --> I[FrontalPoseCalibrator\nRANSAC + 四元数平均 + 平移中位数]
    I --> J[输出正脸参考位姿\nfrontal_R, frontal_t]

    %% =========================
    %% 多帧重建分支
    %% =========================
    D2 --> K[CaculatePoseAmplitude\n计算相对正脸的姿态幅度]
    D3 --> L[按姿态桶累计样本\n低/中/高角度]
    K --> L

    L --> M{样本总量达到阈值?}
    M -- 否 --> N[继续处理下一帧]
    M -- 是 --> O[UpdateFaceModel]

    O --> O1[SolverFaceModel\n优化3D人脸模板点]
    O1 --> O2[约束:\n1) 左右对称\n2) 中心线点只优化 y,z\n3) 非刚性点降维]
    O2 --> O3[重估历史帧位姿 + 再次模型优化(迭代)]
    O3 --> P[输出重建后3D人脸模型]

    %% =========================
    %% 融合输出
    %% =========================
    J --> Q[统一输出]
    P --> Q
    D2 --> Q
    Q --> R[最终输出:\n1) 每帧位姿 R,t\n2) 重建3D关键点\n3) frontal_R, frontal_t\n4) 可视化与结果图]

    N --> A
    R --> A
```

## 对应代码主路径（便于定位）

- 主循环与数据读取：`src/main.cc`
- 单帧处理入口：`PoseEstimation::Process` in `src/pose_estimation.cc`
- 单帧联合优化：`PoseEstimation::SolverFace`
- 正脸自标定：`PoseEstimation::CalibrateFrontedFace` + `src/pose_calibrator.cc`
- 多帧重建：`PoseEstimation::ReconstructFace3D` + `UpdateFaceModel` + `SolverFaceModel`
- 可视化：`src/visualization.cc`
