import cv2
import os
from PIL import Image
import numpy as np
import re
import threading
import queue
import time
from concurrent.futures import ThreadPoolExecutor
from tqdm import tqdm

def natural_sort_key(filename):
    """生成自然排序的key"""
    def convert(text):
        return int(text) if text.isdigit() else text.lower()
    return [convert(c) for c in re.split(r'(\d+)', filename)]

def get_sorted_images(image_folder):
    """获取按自然顺序排序的图像文件列表"""
    images = [img for img in os.listdir(image_folder) 
              if img.lower().endswith((".jpg", ".jpeg", ".png", ".bmp"))]
    images.sort(key=natural_sort_key)
    return images

class FastVideoWriter:
    """快速视频写入器（多线程优化）"""
    
    def __init__(self, output_video, fps, frame_size, codec='mp4v', 
                 max_queue_size=100, num_workers=4):
        """
        初始化快速视频写入器
        
        参数:
        output_video: 输出视频路径
        fps: 帧率
        frame_size: 帧尺寸 (width, height)
        codec: 编码器
        max_queue_size: 队列最大大小
        num_workers: 工作线程数
        """
        self.output_video = output_video
        self.fps = fps
        self.frame_size = frame_size
        self.codec = codec
        
        # 初始化视频写入器
        fourcc = cv2.VideoWriter_fourcc(*codec)
        self.video_writer = cv2.VideoWriter(output_video, fourcc, fps, frame_size)
        
        # 线程安全队列
        self.frame_queue = queue.Queue(maxsize=max_queue_size)
        
        # 控制标志
        self.is_running = False
        self.writer_thread = None
        self.processor_threads = []
        
        # 统计信息
        self.frames_written = 0
        self.total_frames = 0
        
        # 线程池
        self.num_workers = num_workers
        self.executor = None
        
        # 进度条
        self.progress_bar = None
        
    def start(self):
        """启动所有线程"""
        self.is_running = True
        
        # 启动写入线程
        self.writer_thread = threading.Thread(target=self._write_frames, daemon=True)
        self.writer_thread.start()
        
        # 创建线程池
        self.executor = ThreadPoolExecutor(max_workers=self.num_workers)
        
        print(f"开始处理视频，使用 {self.num_workers} 个工作线程")
        self.progress_bar = tqdm(total=self.total_frames, desc="处理进度")
        
    def process_image_pair(self, image_path, image_folder1, image_folder2, save_path=None):
        """处理单对图像（可并行执行）"""
        try:
            # 读取第一张图像
            img1_path = os.path.join(image_folder1, image_path)
            first_image = cv2.imread(img1_path)
            if first_image is None:
                print(f"警告: 无法读取图像 {img1_path}")
                return None
            
            # 读取第二张图像
            img2_path = os.path.join(image_folder2, image_path)
            second_image = cv2.imread(img2_path)
            if second_image is None:
                print(f"警告: 无法读取图像 {img2_path}")
                return None
            
            # 确保图像尺寸一致
            if first_image.shape != second_image.shape:
                # 调整第二张图像尺寸以匹配第一张
                second_image = cv2.resize(second_image, 
                                         (first_image.shape[1], first_image.shape[0]))
            
            # 水平拼接图像
            concatenated = np.concatenate((first_image, second_image), axis=1)
            
            # 可选：保存拼接后的图像
            if save_path:
                save_file = os.path.join(save_path, image_path)
                cv2.imwrite(save_file, concatenated)
            
            return concatenated
        except Exception as e:
            print(f"处理图像 {image_path} 时出错: {e}")
            return None
    
    def add_image_task(self, image_path, image_folder1, image_folder2, save_path=None):
        """添加图像处理任务"""
        if not self.is_running:
            return
            
        # 提交任务到线程池
        future = self.executor.submit(
            self.process_image_pair, 
            image_path, image_folder1, image_folder2, save_path
        )
        
        # 添加回调处理结果
        future.add_done_callback(lambda f: self._handle_processed_frame(f, image_path))
    
    def _handle_processed_frame(self, future, image_path):
        """处理完成后的回调函数"""
        try:
            processed_frame = future.result(timeout=5)
            if processed_frame is not None:
                # 将处理好的帧放入队列
                self.frame_queue.put(processed_frame, timeout=1)
        except Exception as e:
            print(f"处理图像 {image_path} 的回调出错: {e}")
    
    def _write_frames(self):
        """写入帧的线程函数"""
        while self.is_running or not self.frame_queue.empty():
            try:
                # 从队列获取帧
                frame = self.frame_queue.get(timeout=0.1)
                
                # 写入视频
                self.video_writer.write(frame)
                self.frames_written += 1
                
                # 更新进度条
                if self.progress_bar:
                    self.progress_bar.update(1)
                
                # 释放队列标记
                self.frame_queue.task_done()
                
            except queue.Empty:
                # 队列为空，继续等待
                continue
            except Exception as e:
                print(f"写入帧时出错: {e}")
    
    def stop(self):
        """停止所有线程并释放资源"""
        print("\n正在停止处理...")
        self.is_running = False
        
        # 等待队列处理完成
        if not self.frame_queue.empty():
            print(f"等待处理队列中的剩余 {self.frame_queue.qsize()} 帧...")
            self.frame_queue.join()
        
        # 关闭线程池
        if self.executor:
            self.executor.shutdown(wait=True)
        
        # 等待写入线程结束
        if self.writer_thread:
            self.writer_thread.join(timeout=5)
        
        # 释放视频写入器
        self.video_writer.release()
        
        # 关闭进度条
        if self.progress_bar:
            self.progress_bar.close()
        
        print(f"视频保存完成！共写入 {self.frames_written} 帧到 {self.output_video}")
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

def images_to_video_multithread(image_folder1, image_folder2, output_video, 
                                save_path=None, fps=30, num_workers=4):
    """
    将两个文件夹中的图像拼接后写入视频（多线程优化版本）
    
    参数:
    image_folder1: 第一个图像文件夹路径
    image_folder2: 第二个图像文件夹路径
    output_video: 输出视频路径
    save_path: 可选，保存拼接后图像的路径
    fps: 帧率
    num_workers: 工作线程数
    """
    # 获取图像列表
    images1 = get_sorted_images(image_folder1)
    images2 = get_sorted_images(image_folder2)
    
    # 确保两个文件夹图像数量相同
    min_len = min(len(images1), len(images2))
    if min_len == 0:
        print("文件夹中没有找到图像文件！")
        return
    
    # 取较少的图像数量，确保一一对应
    images = images1[:min_len]
    
    print(f"找到 {min_len} 对图像")
    
    # 读取第一张图像获取尺寸
    first_img1 = cv2.imread(os.path.join(image_folder1, images[0]))
    first_img2 = cv2.imread(os.path.join(image_folder2, images[0]))
    
    if first_img1 is None or first_img2 is None:
        print("无法读取第一张图像，请检查图像格式")
        return
    
    # 获取拼接后的图像尺寸
    height, width = first_img1.shape[:2]
    # 水平拼接，宽度加倍
    concatenated_width = width * 2
    
    # 创建快速视频写入器
    with FastVideoWriter(
        output_video=output_video,
        fps=fps,
        frame_size=(concatenated_width, height),
        codec='mp4v',
        num_workers=num_workers
    ) as video_writer:
        
        # 设置总帧数
        video_writer.total_frames = min_len
        
        # 提交所有图像处理任务
        print("开始处理图像...")
        start_time = time.time()
        
        for image_path in images:
            video_writer.add_image_task(
                image_path, 
                image_folder1, 
                image_folder2,
                save_path
            )
        
        # 等待所有任务完成
        video_writer.executor.shutdown(wait=True)
        
        end_time = time.time()
        processing_time = end_time - start_time
        
        print(f"\n图像处理完成！耗时: {processing_time:.2f} 秒")
        print(f"平均处理速度: {min_len/processing_time:.2f} 帧/秒")

# 使用示例
if __name__ == "__main__":
    # 参数配置
    image_folder1 = "/media/xiewei/7BADE9BD542E5B0E/to_xiewei/all_result"
    image_folder2 = "/media/xiewei/7BADE9BD542E5B0E/to_xiewei/all_result_chin"
    output_video = "all_chin_result_fast.mp4"
    save_path = "/home/xiewei/workspace/cidi/data/dms_test"  # 可选
    fps = 20
    num_workers = 16  # 根据CPU核心数调整
    
    # 运行处理
    print("开始多线程视频处理...")
    images_to_video_multithread(
        image_folder1=image_folder1,
        image_folder2=image_folder2,
        output_video=output_video,
        save_path=save_path,
        fps=fps,
        num_workers=num_workers
    )