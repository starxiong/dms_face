#include <dirent.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "../include/common.h"
#include "../include/pose_estimation.h"

namespace fs = std::filesystem;

void list_dir(std::string path, std::string strd,
              std::vector<std::string> &files) {
  DIR *pDir;
  struct dirent *ptr;
  if (!(pDir = opendir(path.c_str())))
    return;
  while ((ptr = readdir(pDir)) != 0) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0 &&
        strstr(ptr->d_name, strd.c_str()) != nullptr) {
      files.emplace_back(path + "/" + ptr->d_name);
    }
  }
  closedir(pDir);
}

std::vector<std::string> GetAllFolders(const std::string &path) {
  std::vector<std::string> folders;

  try {
    // 添加起始路径本身
    // 递归遍历
    for (const auto &entry : fs::recursive_directory_iterator(path)) {
      if (fs::is_directory(entry.status())) {
        std::string file_path = entry.path().string();
        bool is_folder = file_path.find("_result") == std::string::npos;
        if (is_folder)
          folders.push_back(entry.path().string());
      }
    }
  } catch (const fs::filesystem_error &e) {
    std::cerr << "遍历错误: " << e.what() << std::endl;
  }

  return folders;
}

std::vector<std::string> split_string(const std::string &str,
                                      char delimiter = ',') {
  std::vector<std::string> tokens;
  std::istringstream iss(str);
  std::string token;

  while (std::getline(iss, token, delimiter)) {
    tokens.push_back(token);
  }

  return tokens;
}

int main() {
  std::string path = "";
  facepose::PoseEstimation pose_estimator;
  pose_estimator.Init("params/face3d.yml");

  int number = 1;
  facepose::Mat3D R = facepose::Mat3D::Identity();
  facepose::Vec3D t = facepose::Vec3D::Zero();
  facepose::Mat3D next_R = facepose::Mat3D::Identity();
  facepose::Vec3D next_t = facepose::Vec3D::Zero();
  std::ofstream ofs("output.txt");

  std::vector<std::string> folders = GetAllFolders(path);

  for (int d = 0; d < folders.size(); d++) {
    std::string file_path = folders[d];
    std::vector<std::string> files;
    list_dir(file_path, ".txt", files);

    if (!std::filesystem::exists(file_path + "_result")) {
      fs::create_directories(file_path + "_result");
    }
    std::sort(files.begin(), files.end());
    pose_estimator.Reset();
    int frame_id = 0;
    for (int i = 0; i < files.size(); i++) {
      std::string filename = files[i];
      std::cout << filename << std::endl;
      std::ifstream ifs(filename);
      std::vector<std::vector<facepose::Point2D>> poses;
      std::string line;
      while (std::getline(ifs, line)) {
        std::vector<facepose::Point2D> pose;
        std::vector<std::string> line_tmp = split_string(line, ',');
        if (line_tmp.size() < 2)
          continue;
        line = line_tmp[1];
        // std::cout << line << std::endl;
        std::istringstream iss(line);
        std::string str;
        int num = 0;
        facepose::Point2D p;
        while (iss >> str) {
          if (num % 2 == 0) {
            p.x = std::stof(str);
          } else {
            p.y = std::stof(str);
            pose.emplace_back(p);
          }
          num++;
        }
        poses.emplace_back(pose);
      }

      std::string video_path = filename;
      size_t pos = video_path.find(".txt");
      if (pos != std::string::npos) {
        if (std::filesystem::exists(video_path.substr(0, pos) + ".mp4")) {
          video_path = video_path.substr(0, pos) + ".mp4";
        } else if (std::filesystem::exists(video_path.substr(0, pos) +
                                           ".avi")) {
          video_path = video_path.substr(0, pos) + ".avi";
        }
      }
      std::cout << video_path << std::endl;

      cv::VideoCapture cap(video_path);
      if (!cap.isOpened())
        continue;
      int totalFrames = (int)cap.get(cv::CAP_PROP_FRAME_COUNT);
      std::cout << "totalFrames: " << totalFrames
                << ", poses_size:" << poses.size() << std::endl;
      if (totalFrames != poses.size())
        continue;
      cv::Mat frame;
      int video_frame = 0;
      while (true) {
        bool success = cap.read(frame);
        if (!success)
          break;
        std::vector<facepose::Point2D> pose = poses[video_frame];
        if (pose.size() != 32)
          continue;
        pose_estimator.SetNumber(frame_id);
        pose_estimator.Process(pose);
        pose_estimator.Display(frame);
        pose_estimator.GetPose(next_R, next_t);
        facepose::Mat3D R_diff = next_R * R.transpose();
        facepose::AxisD angle_axis(R_diff);
        double angle_diff = angle_axis.angle();       // 弧度
        double angle_deg = angle_diff * 180.0 / M_PI; // 角度
        double t_diff = (next_t - t).norm();
        if ((t_diff > 0.5 || angle_deg > 10.0) && video_frame > 1) {
          ofs << filename << ", number: " << number
              << ", frame_id: " << frame_id << ", angle_deg: " << angle_deg
              << ", t_diff: " << t_diff << std::endl;
        }
        cv::imwrite(file_path + "_result/" + std::to_string(frame_id) + ".jpg",
                    frame);
        R = next_R, t = next_t;
        number++, frame_id++, video_frame++;
      }
    }
  }

  ofs.close();
  return 0;
}