

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <thread>
#include <filesystem>
#include <unordered_set>

#include "read_configs.h"
#include "dataset.h"
#include "map_user.h"

namespace fs = std::filesystem;

int main(int argc, char **argv) {
  ros::init(argc, argv, "air_slam");
  ros::NodeHandle nh;

  std::string config_path, model_dir, map_root, voc_path, traj_path;
  ros::param::get("~config_path", config_path);
  ros::param::get("~model_dir", model_dir);
  ros::param::get("~map_root", map_root);
  ros::param::get("~voc_path", voc_path);
  ros::param::get("~traj_path", traj_path);

  RelocalizationConfigs configs(config_path, model_dir);
  ros::param::get("~dataroot", configs.dataroot);
  ros::param::get("~camera_config_path", configs.camera_config_path);
  std::ofstream outFile;
  outFile.open("/my_workspace/SlamDemo_0116/SlamDemo_0116/SlamDemo_Data/StreamingAssets/CamTrans.txt");
  outFile.close();

  MapUser map_user(configs, nh);
  map_user.LoadMap(map_root);
  map_user.LoadVocabulary(voc_path);

  std::unordered_set<std::string> processed_images;
  std::vector<std::pair<std::string, Eigen::Matrix4d>> trajectory;

  while (ros::ok()) {
    std::cout<<"start reloc"<<std::endl;
    for (const auto &entry : fs::directory_iterator(configs.dataroot)) {
      std::string image_path = entry.path().string();
    //   if (processed_images.find(image_path) != processed_images.end()) {
    //     continue; // 跳过已处理的图片
    //   }

    //   processed_images.insert(image_path); // 标记为已处理

      // 读取图像
      cv::Mat image = cv::imread(image_path, 0);
      if (image.empty()) {
        std::cerr << "Failed to read image: " << image_path << std::endl;
        continue;
      }

      Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
      auto before_infer = std::chrono::high_resolution_clock::now();

      std::string image_idx = "fail " + entry.path().stem().string();
      if (map_user.Relocalization(image_path, image, pose)) {
        image_idx = "success " + entry.path().stem().string();
      }

      auto after_infer = std::chrono::high_resolution_clock::now();
      auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(after_infer - before_infer).count();
      std::cout << "Processed " << image_path << " in " << cost_time << " ms." << std::endl;

      trajectory.emplace_back(std::make_pair(image_idx, pose));
      // std::cout<<pose<<std::endl;
    }

    // // 保存轨迹
    SaveTumTrajectoryToFile(traj_path, trajectory);

    std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 每隔0.15秒检查一次新图片
  }

  map_user.StopVisualization();
  ros::shutdown();

  return 0;
}
