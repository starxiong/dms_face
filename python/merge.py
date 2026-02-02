import cv2
import os
from PIL import Image
import numpy as np
import re

def natural_sort_key(filename):
    """
    生成自然排序的key
    支持：1.jpg, img_1.jpg, frame001.png 等格式
    """
    def convert(text):
        return int(text) if text.isdigit() else text.lower()
    
    return [convert(c) for c in re.split(r'(\d+)', filename)]

def get_sorted_images(image_folder):
    """获取按自然顺序排序的图像文件列表"""
    images = [img for img in os.listdir(image_folder) 
              if img.lower().endswith((".jpg", ".jpeg", ".png", ".bmp"))]
    
    # 使用自然排序
    images.sort(key=natural_sort_key)
    
    return images



def images_to_video_opencv(image_folder, output_video, fps=30):
    """
    将图像文件夹中的图片转换为视频
    
    参数:
    image_folder: 图像文件夹路径
    output_video: 输出视频路径
    fps: 帧率（每秒帧数）
    """
    # 获取所有图像文件
    # images = [img for img in os.listdir(image_folder) 
    #           if img.endswith(".png") or img.endswith(".jpg") 
    #           or img.endswith(".jpeg") or img.endswith(".bmp")]
    sorted_images = get_sorted_images(image_folder)
    # images.sort()  # 按文件名排序
    images = sorted_images  
    
    if not images:
        print("文件夹中没有找到图像文件！")
        return
    first_image = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, _ = first_image.shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
    video_writer = cv2.VideoWriter(output_video, fourcc, fps, (2*width, height)) # MP4编码
    for image_path in images:
        print(image_path)
        # 读取第一张图像获取尺寸
        first_image = cv2.imread(os.path.join(image_folder, image_path))
        second_image = cv2.imread(os.path.join(image_folder + "_chin", image_path))
        # cv2.putText(first_image, "first", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        # first_image_resize = cv2.resize(first_image, (int(width/2), int(height/2)))
        # second_image_resize = cv2.resize(second_image, (int(width/2), int(height/2)))
        image = np.concatenate((first_image, second_image), axis=1)
        # cv2.namedWindow("test")
        # cv2.imshow("test",image)
        # cv2.waitKey(0)
        cv2.imwrite(os.path.join(save_path, image_path), image)

        # 对于H.264编码，可以尝试使用：fourcc = cv2.VideoWriter_fourcc(*'avc1')
        video_writer.write(image)
    video_writer.release()
    

# 使用示例
if __name__ == "__main__":
    save_path = "/home/xiewei/workspace/cidi/data/dms_test"
    images_to_video_opencv("/media/xiewei/7BADE9BD542E5B0E/to_xiewei/all_result", "all_chin_result2.mp4", fps=20)