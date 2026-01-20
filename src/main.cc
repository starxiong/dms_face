#include <dirent.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "../include/common.h"
#include "../include/pose_estimation.h"

void list_dir(std::string path, std::string strd,
                     std::vector<std::string>& files) {
  DIR* pDir;
  struct dirent* ptr;
  if (!(pDir = opendir(path.c_str()))) return;
  while ((ptr = readdir(pDir)) != 0) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0 &&
        strstr(ptr->d_name, strd.c_str()) != nullptr) {
      files.emplace_back(path + "/" + ptr->d_name);
    }
  }
  closedir(pDir);
}

std::vector<std::string> split_string(const std::string& str, char delimiter = ',') {
    std::vector<std::string> tokens;
    std::istringstream iss(str);
    std::string token;
    
    while (std::getline(iss, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}

int main() {
  std::vector<std::string> files;
  std::string path = "/home/cidi/workspace/DMS/to_xiewei";
  list_dir(path, ".txt", files);
  facepose::PoseEstimation pose_estimator;
  pose_estimator.Init("params/face3d.yml");

  for (int i = 0; i < files.size(); i++) {
    pose_estimator.Reset();
    std::string filename = files[i];
    std::cout << filename << std::endl;
    std::ifstream ifs(filename);
    std::vector<std::vector<facepose::Point2D>> poses;
    std::string line;
    while (std::getline(ifs, line)) {
      std::vector<std::string> line_tmp = split_string(line, ',');
      line = line_tmp[1];
      // std::cout << line << std::endl;
      std::istringstream iss(line);
      std::string str;
      std::vector<facepose::Point2D> pose;
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
      video_path = video_path.substr(0, pos) + ".avi";
    }
    std::cout << video_path << std::endl;

    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) return -1;
    cv::Mat frame;
    int num = 0;
    while (cap.read(frame)) {
      std::vector<facepose::Point2D> pose = poses[num];
      pose_estimator.Process(pose);
      pose_estimator.Display(frame);
      num++;
    }
  }
  return 0;
}