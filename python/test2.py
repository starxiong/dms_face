import cv2
import numpy as np

def undistort_keep_all_pixels(img, camera_matrix, dist_coeffs):
    """
    去畸变并确保包含所有像素 - 修复版
    """
    h, w = img.shape[:2]
    
    # 1. 先计算原始图像四个角点去畸变后的归一化坐标
    corners = np.array([
        [[0, 0]],        # 左上
        [[w-1, 0]],      # 右上
        [[w-1, h-1]],    # 右下
        [[0, h-1]]       # 左下
    ], dtype=np.float32)
    
    # 使用undistortPoints计算去畸变后的归一化坐标
    undistorted_corners_norm = cv2.undistortPoints(corners, camera_matrix, dist_coeffs)
    
    # 2. 将归一化坐标转换为像素坐标
    # 需要乘以fx,fy并加上cx,cy
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    
    undistorted_corners_pixel = []
    for corner in undistorted_corners_norm:
        x = corner[0, 0] * fx + cx
        y = corner[0, 1] * fy + cy
        undistorted_corners_pixel.append([x, y])
    
    undistorted_corners_pixel = np.array(undistorted_corners_pixel)
    
    # 3. 找到边界
    min_x = undistorted_corners_pixel[:, 0].min()
    max_x = undistorted_corners_pixel[:, 0].max()
    min_y = undistorted_corners_pixel[:, 1].min()
    max_y = undistorted_corners_pixel[:, 1].max()
    
    print(f"原始图像范围: (0, 0) - ({w}, {h})")
    print(f"去畸变后范围: ({min_x:.1f}, {min_y:.1f}) - ({max_x:.1f}, {max_y:.1f})")
    
    # 4. 计算新图像尺寸
    new_w = int(np.ceil(max_x - min_x))
    new_h = int(np.ceil(max_y - min_y))
    
    print(f"新图像尺寸: {new_w} x {new_h}")
    
    # 5. 调整相机矩阵
    new_camera_matrix = camera_matrix.copy()
    new_camera_matrix[0, 2] = cx - min_x  # 调整cx
    new_camera_matrix[1, 2] = cy - min_y  # 调整cy
    
    # 6. 计算映射
    mapx, mapy = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None,
        new_camera_matrix, (new_w, new_h), cv2.CV_32FC1
    )
    
    # 7. 重映射
    result = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
    
    return result

# 测试代码
img = cv2.imread("/home/cidi/Desktop/11/微信图片_2026-01-22_114905_810.png")
if img is None:
    print("无法读取图像")
    exit()

camera_matrix = np.array([
    [1202.977997787424, 0, 948.7156],
    [0, 1201.675365074967, 778.8761],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.array([-0.424, 0.2188, -0.0055, 0.0014, -0.0615], dtype=np.float32)

# 去畸变
result = undistort_keep_all_pixels(img, camera_matrix, dist_coeffs)

# 显示
cv2.namedWindow("Original")
cv2.namedWindow("Undistorted Complete")
cv2.imshow("Original", img)
cv2.imshow("Undistorted Complete", result)
print("按任意键继续...")
cv2.waitKey(0)
cv2.destroyAllWindows()

# 保存
cv2.imwrite("undistorted_complete.jpg", result)