import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

r0 = R.from_euler('xyz', [0, 0 , 0], degrees=True)
r = R.from_euler('xyz', [0, 0 , 0], degrees=True)
r1 = R.from_euler('xyz', [24.2,-50.5,12.9], degrees=True)
r2 = R.from_euler('xyz', [0,0, 0], degrees=True)

vec = np.array([0, 0, 3])
nvec1 = r1.as_matrix() @ vec
nvec2 = r2.as_matrix() @ vec
print(nvec1)
print(nvec2)


R_mat = r.as_matrix()
R_mat1 = r1.as_matrix()
R_mat2 = r2.as_matrix()
# 原始坐标轴（单位向量）
origin = np.array([0, 0, 0])
x_axis = np.array([1, 0, 0])
y_axis = np.array([0, 1, 0])
z_axis = np.array([0, 0, 2])

# 旋转后的坐标轴
x_rot = R_mat @ x_axis
y_rot = R_mat @ y_axis
z_rot = R_mat @ z_axis

x_rot1 =  R_mat1 @ x_axis
y_rot1 =  R_mat1 @ y_axis
z_rot1 =  R_mat1 @ z_axis

x_rot2 = R_mat2 @ x_axis/2
y_rot2 = R_mat2 @ y_axis/2
z_rot2 = R_mat2 @ z_axis/2

# 创建图形
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# 绘制原始坐标轴（实线箭头）
ax.quiver(*origin, *x_axis, color='r', arrow_length_ratio=0.1, label='x (原始)')
ax.quiver(*origin, *y_axis, color='g', arrow_length_ratio=0.1, label='y (原始)')
ax.quiver(*origin, *z_axis, color='b', arrow_length_ratio=0.1, label='z (原始)')

# 绘制旋转后的坐标轴（虚线箭头，稍粗）
# ax.quiver(*origin, *x_rot, color='r', arrow_length_ratio=0.2, linewidth=1, label='x (旋转后)')
# ax.quiver(*origin, *y_rot, color='g', arrow_length_ratio=0.2, linewidth=1, label='y (旋转后)')
# ax.quiver(*origin, *z_rot, color='b', arrow_length_ratio=0.2, linewidth=1, label='z (旋转后)')


# # 绘制旋转后的坐标轴（虚线箭头，稍粗）
ax.quiver(*origin, *x_rot1, color='r', arrow_length_ratio=0.1, linestyle='dashed', linewidth=1, label='x (旋转后)')
ax.quiver(*origin, *y_rot1, color='g', arrow_length_ratio=0.1, linestyle='dashed', linewidth=1, label='y (旋转后)')
ax.quiver(*origin, *z_rot1, color='b', arrow_length_ratio=0.1, linestyle='dashed', linewidth=1, label='z (旋转后)')

# # 绘制旋转后的坐标轴（虚线箭头，稍粗）
# ax.quiver(*origin, *x_rot2, color='r', arrow_length_ratio=0.3, linestyle='dashed', linewidth=2, label='x (旋转后)')
# ax.quiver(*origin, *y_rot2, color='g', arrow_length_ratio=0.3, linestyle='dashed', linewidth=2, label='y (旋转后)')
# ax.quiver(*origin, *z_rot2, color='b', arrow_length_ratio=0.3, linestyle='dashed', linewidth=2, label='z (旋转后)')

ax.quiver(*origin, *nvec1, color='b', arrow_length_ratio=0.1, linewidth=1, label='v1')
ax.quiver(*origin, *nvec2, color='y', arrow_length_ratio=0.1, linewidth=1, label='v2')

w = -3.5
h = 3.5
x = np.linspace(w, h, 10)
z = np.linspace(w, h, 10)
X, Z = np.meshgrid(x, z)
Y = np.zeros_like(X)  
ax.plot_surface(Z, Y, X, alpha=0.5, color='cyan', edgecolor='none')
# 设置坐标轴范围
ax.set_xlim([w, h])
ax.set_ylim([w, h])
ax.set_zlim([w, h])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('旋转矩阵作用后的坐标系')
ax.legend()
plt.show()