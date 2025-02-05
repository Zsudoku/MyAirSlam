#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>


// 读取位姿信息
struct Pose {
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation;
};


// 读取位姿文件
std::map<std::string, Pose> readPoses(const std::string& poseFile) {
    std::map<std::string, Pose> poses;
    std::ifstream file(poseFile);
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string status, timestamp;
        double tx, ty, tz, qx, qy, qz, qw;
        if (!(iss >> status >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
            continue;
        }
        if (status == "success") {
            Pose pose;
            pose.translation = Eigen::Vector3d(tx, ty, tz);
            pose.rotation = Eigen::Quaterniond(qw, qx, qy, qz);
            poses[timestamp] = pose;
        }
    }
    return poses;
}


// 将深度图转换为点云
pcl::PointCloud<pcl::PointXYZ>::Ptr depthToPointCloud(
    const cv::Mat& depthImage, const Pose& pose, const Eigen::Matrix3d& K) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int v = 0; v < depthImage.rows; ++v) {
        for (int u = 0; u < depthImage.cols; ++u) {
            float depth = depthImage.at<float>(v, u);
            if (depth <= 0) continue;
            Eigen::Vector3d point;
            point[2] = depth;
            point[0] = (u - K(0, 2)) * depth / K(0, 0);
            point[1] = (v - K(1, 2)) * depth / K(1, 1);
            point = pose.rotation * point + pose.translation;
            cloud->points.emplace_back(point[0], point[1], point[2]);
        }
    }
    return cloud;
}


int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <pose_file> <image_folder> <depth_folder> <output_ply>" << std::endl;
        return -1;
    }


    std::string poseFile = argv[1];
    std::string imageFolder = argv[2];
    std::string depthFolder = argv[3];
    std::string outputPly = argv[4];


    // 相机内参矩阵K，请根据您的相机参数进行设置
    Eigen::Matrix3d K;
    K << fx, 0, cx,
         0, fy, cy,
         0,  0,  1;


    // 读取位姿信息
    auto poses = readPoses(poseFile);


    pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloud(new pcl::PointCloud<pcl::PointXYZ>);


    for (const auto& entry : poses) {
        std::string timestamp = entry.first;
        Pose pose = entry.second;


        // 加载深度图像
        std::string depthPath = depthFolder + "/" + timestamp + ".pgm";
        cv::Mat depthImage = cv::imread(depthPath, cv::IMREAD_UNCHANGED);
        if (depthImage.empty()) {
            std::cerr << "Failed to load depth image: " << depthPath << std::endl;
            continue;
        }


        // 将深度图转换为点云
        auto cloud = depthToPointCloud(depthImage, pose, K);
        *globalCloud += *cloud;
    }


    // 对点云进行体素滤波以降采样
    pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
    voxelFilter.setInputCloud(globalCloud);
    voxelFilter.setLeafSize(0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxelFilter.filter(*filteredCloud);


    // 保存点云到PLY文件
    pcl::io::savePLYFileBinary(outputPly, *filteredCloud);


    std::cout << "Point cloud saved to " << outputPly << std::endl;
    return 0;
}