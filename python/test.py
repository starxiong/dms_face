import cv2
import os
from PIL import Image
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
    
    # 读取第一张图像获取尺寸
    first_image = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, _ = first_image.shape
    
    # 创建VideoWriter对象
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MP4编码
    # 对于H.264编码，可以尝试使用：fourcc = cv2.VideoWriter_fourcc(*'avc1')
    video_writer = cv2.VideoWriter(output_video, fourcc, fps, (width, height))
    
    # 逐帧写入图像
    for i, image_name in enumerate(images):
        img_path = os.path.join(image_folder, image_name)
        img = cv2.imread(img_path)
        
        # 如果图像尺寸不一致，调整到统一尺寸
        if img.shape[0] != height or img.shape[1] != width:
            img = cv2.resize(img, (width, height))
        
        video_writer.write(img)
        print(f"已处理第 {i+1}/{len(images)} 帧: {image_name}")
    
    # 释放资源
    video_writer.release()
    print(f"视频已保存到: {output_video}")

# 使用示例
if __name__ == "__main__":
    images_to_video_opencv("result", "result1.mp4", fps=20)