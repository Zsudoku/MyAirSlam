#include <iostream>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

void copyImages(const std::string& source_folder, const std::string& dest_folder) {
    std::vector<std::string> image_files;

    // 获取 source_folder 中的所有文件
    for (const auto& entry : fs::directory_iterator(source_folder)) {
        if (entry.is_regular_file() && entry.path().extension() == ".png") {
            image_files.push_back(entry.path().string());
        }
    }

    // 如果文件夹中有图像文件，按照文件名（时间戳顺序）排序
    if (!image_files.empty()) {
        std::sort(image_files.begin(), image_files.end());

        // 遍历图像文件，逐一复制到目标文件夹
        for (const auto& image_path : image_files) {
            std::string dest_path = dest_folder + "/" + fs::path(image_path).filename().string();
            
            // 使用 OpenCV 读取图像并保存到目标文件夹
            cv::Mat image = cv::imread(image_path);
            if (!image.empty()) {
                bool success = cv::imwrite(dest_path, image);  // 复制图像
                if (success) {
                    std::cout << "Copied image: " << image_path << " to " << dest_path << std::endl;
                } else {
                    std::cerr << "Failed to copy image: " << image_path << std::endl;
                }
            } else {
                std::cerr << "Failed to load image: " << image_path << std::endl;
            }

            // 等待一段时间后继续复制下一张图像
            std::this_thread::sleep_for(std::chrono::seconds(1));  // 每隔1秒复制一张
        }
    } else {
        std::cout << "No images found in the source folder." << std::endl;
    }
}

int main() {
    std::string source_folder = "/workspace/catkin/src/AirSlam/dataroot/cam1/data";  // 源文件夹路径
    std::string dest_folder = "/workspace/catkin/src/AirSlam/dataroot/test";      // 目标文件夹路径

    while (true) {
        std::cout << "Checking for images..." << std::endl;
        copyImages(source_folder, dest_folder);  // 执行图像复制操作

        // 等待一定时间后再次检查
        std::this_thread::sleep_for(std::chrono::minutes(1));  // 每隔1分钟检测一次文件夹
    }

    return 0;
}
