calibration lidar and ins pose

## Prerequisites

- opencv
- ceres[已有]
- Eigen[已有]

## Compile

```shell
# mkdir build
mkdir -p build && cd build
# build
cmake .. && make j8
```
- 由于ceres库需要加入到编译过程中，编译较慢，请耐心等待

## Usage

## Data

- [data](data)
- 输入：fram_id, x1,y1,x2,y2,....,x32,y32[去完畸变的像素坐标]

- 输出：
  人脸到3D face的位姿【R， t】
  重构的人脸3D关节点
  3D face的刚性点坐标和非刚性点坐标

## 算法原理
![alt text](image.png)