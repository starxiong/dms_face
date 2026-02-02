import os
import numpy as np

output_dir = "output.txt"
output_dir2 = "/home/xiewei/workspace/cidi/DMS/face_pose_chin/output.txt"

file = open(output_dir, "r")
file2 = open(output_dir2, "r")

lines = file.readlines()
lines2 = file2.readlines()

angle = []
trans = []
for line in lines:
    line_split = line.split(",")
    angle.append(float(line_split[3].split(':')[1]))
    trans.append(float(line_split[4].split(':')[1].replace('\n','').replace(' ','')))
angle2 = []
trans2 = []
for line in lines2:
    line_split = line.split(",")
    angle2.append(float(line_split[3].split(':')[1]))
    trans2.append(float(line_split[4].split(':')[1].replace('\n','').replace(' ','')))

sorted_indice = [i for i, _ in sorted(enumerate(angle), key=lambda x: x[1], reverse=True)]

lines = [lines[i] for i in sorted_indice]
lines2 = [lines2[i] for i in sorted_indice]
angle = [angle[i] for i in sorted_indice]
angle2 = [angle2[i] for i in sorted_indice]

a = [print(line.replace('\n','') + ', angle_diff: ' + str(angle[i] - angle2[i])) for i, line in enumerate(lines)]

b = [angle[i] - angle2[i] for i, line in enumerate(lines)]
print(np.mean(b))


