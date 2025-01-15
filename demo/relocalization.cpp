#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <thread>
#include <filesystem>

#include "read_configs.h"
#include "dataset.h"
#include "map_user.h"


void processSingleImage(const std::string& image_path, map_user_type& map_user, std::vector<std::pair<std::string, Eigen::Matrix4d>>& trajectory) {
    cv::Mat image = cv::imread(image_path, 0);  // 读取灰度图像
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    size_t pos = image_path.find_last_of('/');
    std::string image_name = image_path.substr(pos + 1);
    std::string image_idx = "fail " + image_name;

    auto before_infer = std::chrono::high_resolution_clock::now();

    if (map_user.Relocalization(image_name, image, pose)) {
        image_idx = "success " + image_name;
    }

    auto after_infer = std::chrono::high_resolution_clock::now();
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(after_infer - before_infer).count();
    std::cout << "One Frame Processing Time: " << cost_time << " ms." << std::endl;

    trajectory.emplace_back(std::make_pair(image_idx, pose));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "air_slam");
  ros::NodeHandle nh;
  std::ofstream outFile;
  // 打开名为"example.txt"的文件
  outFile.open("/workspace/catkin/src/AirSlam/my_output/example.txt");
  outFile.close();
  std::string config_path, model_dir, map_root, voc_path, traj_path;
  ros::param::get("~config_path", config_path);
  ros::param::get("~model_dir", model_dir);
  ros::param::get("~map_root", map_root);
  ros::param::get("~voc_path", voc_path);
  ros::param::get("~traj_path", traj_path);

  RelocalizationConfigs configs(config_path, model_dir);
  ros::param::get("~dataroot", configs.dataroot);
  ros::param::get("~camera_config_path", configs.camera_config_path);

  MapUser map_user(configs, nh);
  map_user.LoadMap(map_root);
  map_user.LoadVocabulary(voc_path);

  std::string folder_path = configs.dataroot;  // 文件夹路径
  std::vector<std::pair<std::string, Eigen::Matrix4d>> trajectory;
  
  // 获取基准帧信息
  Eigen::Matrix4d base_frame_pose = map_user.GetBaseFramePose();
  double base_frame_time = map_user.GetBaseFrameTimestamp();
  trajectory.emplace_back(std::make_pair(("base " + DoubleTimeToString(base_frame_time)), base_frame_pose));

  while (ros::ok()) {
      // 获取文件夹中的图像文件
      std::vector<std::string> image_names;
      for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
          if (entry.is_regular_file() && entry.path().extension() == ".jpg") {
              image_names.push_back(entry.path().string());
          }
      }

      if (!image_names.empty()) {
          std::string latest_image = image_names[0];  // 假设文件夹里只有一张图像
          std::cout << "Found image: " << latest_image << std::endl;

          // 处理图像
          processSingleImage(latest_image, map_user, trajectory);

          // 处理完一张图像后，可以选择删除它，或等待下一次处理
          std::filesystem::remove(latest_image);  // 删除文件，防止一直处理同一张

          // 等待一段时间再进行下一轮检测
          std::this_thread::sleep_for(std::chrono::seconds(1));  // 例如每秒检测一次
      }
      else {
          // 如果没有图像文件，等待一段时间再检查
          std::this_thread::sleep_for(std::chrono::seconds(1));
      }
  }

  // 保存轨迹到文件
  SaveTumTrajectoryToFile(traj_path, trajectory);
  std::cout << "Processing complete." << std::endl;

  return 0;
}


//   std::vector<std::string> image_names;
//   GetFileNames(configs.dataroot, image_names);
//   std::sort(image_names.begin(), image_names.end());
//   size_t dataset_length = image_names.size();

//   std::vector<std::pair<std::string, Eigen::Matrix4d>> trajectory;
//   Eigen::Matrix4d base_frame_pose = map_user.GetBaseFramePose();
//   double base_frame_time = map_user.GetBaseFrameTimestamp();
//   trajectory.emplace_back(std::make_pair(("base "+DoubleTimeToString(base_frame_time)), base_frame_pose));

//   int success_num = 0;
//   for(size_t i = 0; i < dataset_length && ros::ok(); ++i){
//     std::cout << "i ====== " << i << std::endl;

//     cv::Mat image = cv::imread(ConcatenateFolderAndFileName(configs.dataroot, image_names[i]), 0);
//     Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
//     size_t pos = image_names[i].find_last_of('.');
//     std::string image_idx = "fail " + image_names[i].substr(0, pos);

//     auto before_infer = std::chrono::high_resolution_clock::now();
//     if(map_user.Relocalization(image_names[i],image, pose)){
//       image_idx = "success " + image_names[i].substr(0, pos);
//       success_num++;
//       // std::cout<<image_names[i] << " success" <<std::endl;
//       // std::cout<<"pose"<<std::endl;
//       // std::cout<<pose<<std::endl;
//     }
//     auto after_infer = std::chrono::high_resolution_clock::now();
//     auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(after_infer - before_infer).count();
//     std::cout << "One Frame Processinh Time: " << cost_time << " ms." << std::endl;

//     trajectory.emplace_back(std::make_pair(image_idx, pose));
//   }

//   SaveTumTrajectoryToFile(traj_path, trajectory);
//   std::cout << "sum_num = " << dataset_length << ", success_num = " << success_num << ", recall = " << (float)success_num / dataset_length << std::endl;

//   map_user.StopVisualization();
//   ros::shutdown();

//   return 0;
// }
