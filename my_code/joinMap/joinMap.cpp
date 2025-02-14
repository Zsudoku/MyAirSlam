#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <deque>
#include <Eigen/Dense>
#include <chrono>
using namespace cv;
using namespace std;
using namespace Eigen;
using namespace chrono;

// 相机内参
const Mat K = (Mat_<double>(3, 3) << 667.3003, 0, 596.8533,
                                     0, 670.0276, 395.0267,
                                     0, 0, 1);  // 请根据你的相机内参进行调整

// 读取位姿文件
bool isInVector(const std::string& value, const std::vector<std::string>& vec) {
    // 使用 std::find 查找元素
    return std::find(vec.begin(), vec.end(), value) != vec.end();
}
bool readPoseFile(const string &filename, vector<string> &imageNames, vector<Vector3d> &translations, vector<Quaterniond> &rotations) {
    ifstream infile(filename);
    if (!infile.is_open()) {
        cerr << "Failed to open file: " << filename << endl;
        return false;
    }

    string line;
    while (getline(infile, line)) {
        // 跳过空行
        if (line.empty()) {
            continue;
        }

        stringstream ss(line);
        string status, imageName, timestamp;
        double tx, ty, tz, qw, qx, qy, qz;

        // 读取每一行数据
        ss >> status >> imageName;

        // 解析位移向量 tx, ty, tz
        ss >> tx; ss.ignore();  // ss.ignore()用于跳过逗号或空格
        ss >> ty; ss.ignore();
        ss >> tz; ss.ignore();

        // 解析四元数旋转部分 qw, qx, qy, qz
        
        ss >> qx; ss.ignore();
        ss >> qy; ss.ignore();
        ss >> qz; ss.ignore();
        ss >> qw; ss.ignore();
        // 只处理"success"状态的数据
        if (status == "success" && !isInVector(imageName, imageNames)) {
            imageNames.push_back(imageName);
            translations.push_back(Vector3d(tx, ty, tz));
            rotations.push_back(Quaterniond(qw, qx, qy, qz));
        }
    }

    return true;
}

// bool readPoseFile(const string &filename, vector<string> &imageNames, vector<Vector3d> &translations, vector<Quaterniond> &rotations) {
//     ifstream infile(filename);
//     if (!infile.is_open()) {
//         cerr << "Failed to open file: " << filename << endl;
//         return false;
//     }
//     string line;
//     while (getline(infile, line)) {
//         stringstream ss(line);
//         string imageName;
//         double tx, ty, tz, qw, qx, qy, qz;
//         getline(ss, imageName, ',');
//         ss >> tx; ss.ignore();
//         ss >> ty; ss.ignore();
//         ss >> tz; ss.ignore();
//         ss >> qx; ss.ignore();
//         ss >> qy; ss.ignore();
//         ss >> qz; ss.ignore();
//         ss >> qw; ss.ignore();
//         imageNames.push_back(imageName);
//         translations.push_back(Vector3d(tx, ty, tz));
//         rotations.push_back(Quaterniond(qw, qx, qy, qz));
//     }
//     return true;

    
// }

// 从深度图像计算3D点
Point3d depthTo3D(int u, int v, double depth) {
    double x = (u - K.at<double>(0, 2)) * depth / K.at<double>(0, 0);
    double y = (v - K.at<double>(1, 2)) * depth / K.at<double>(1, 1);
    double z = depth;
    return Point3d(x, y, z);
}

// 将相机坐标系中的点转换到世界坐标系
Point3d transformToWorld(const Point3d &point, const Vector3d &translation, const Quaterniond &rotation) {
    Vector3d point_camera(point.x, point.y, point.z);
    Vector3d point_world = rotation * point_camera + translation;
    return Point3d(point_world.x(), point_world.y(), point_world.z());
}

// 计算两点之间的欧氏距离
double pointDistance(const Point3d &p1, const Point3d &p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

// 生成点云并保存为PLY格式
void savePointCloud(const vector<Point3d> &points, const string &filename) {
    ofstream ply_file(filename);
    ply_file << "ply\n";
    ply_file << "format ascii 1.0\n";
    ply_file << "element vertex " << points.size() << "\n";
    ply_file << "property float x\n";
    ply_file << "property float y\n";
    ply_file << "property float z\n";
    ply_file << "end_header\n";
    for (const auto &point : points) {
        ply_file << point.x << " " << point.y << " " << point.z << "\n";
    }
    ply_file.close();
    cout << "Point cloud saved to " << filename << endl;
}

int main() {
    string poseFile = "/my_workspace/catkin_ws/src/AirSlam/debug/relocalization.txt";  // 替换为实际文件路径
    string colorFolder = "/my_workspace/catkin_ws/src/AirSlam/dataroot/room2/cam1/data";    // 替换为实际文件夹路径
    string depthFolder = "/my_workspace/catkin_ws/src/AirSlam/dataroot/room2/depth";    // 替换为实际文件夹路径

    vector<string> imageNames;
    vector<Vector3d> translations;
    vector<Quaterniond> rotations;

    // 读取位姿文件
    if (!readPoseFile(poseFile, imageNames, translations, rotations)) {
        return -1;
    }

    vector<Point3d> pointCloud;
    deque<Point3d> recentPoints;  // 滑动窗口
    const int WINDOW_SIZE = 50000;   // 滑动窗口大小
    int my_count = 0;
    for (size_t i = 0; i < imageNames.size(); ++i) {
        auto start = high_resolution_clock::now();
        cout << "Processing image: " << i << endl;
        cout << "count:" << i << endl;
        if (my_count < 20) {
            my_count++;
            continue;
        } else {
            my_count = 0;
        }
        string colorPath = colorFolder + "/" + imageNames[i] + ".jpg";
        string depthPath = depthFolder + "/" + imageNames[i] + ".pgm";

        // 读取 color 和 depth 图像
        Mat colorImage = imread(colorPath);
        Mat depthImage = imread(depthPath, IMREAD_UNCHANGED); // depth 图通常是16位图
        if (colorImage.empty() || depthImage.empty()) {
            cerr << "Failed to read images for: " << imageNames[i] << endl;
            continue;
        }

        // 遍历每个像素生成 3D 点
        for (int v = 0; v < depthImage.rows; v += 10) {  // 每隔5行进行计算（稀疏化）
            for (int u = 0; u < depthImage.cols; u += 10) {  // 每隔5列进行计算（稀疏化）
                ushort depth_value = depthImage.at<ushort>(v, u);
                if (depth_value == 0) continue; // 忽略无效深度值

                // 转换深度图为 3D 点
                double depth = depth_value / 1000.0;  // 如果深度值单位是毫米，需要转换为米
                // std::cout<<depth<<std::endl;
                Point3d point_camera = depthTo3D(u, v, depth);

                // 应用相机的位姿，将相机坐标系转换为世界坐标系
                Point3d point_world = transformToWorld(point_camera, translations[i], rotations[i]);

                // // 去重：只检查最近点
                bool isDuplicate = false;
                for (const auto &recentPoint : recentPoints) {
                    if (pointDistance(recentPoint, point_world) < 0.01) {
                        isDuplicate = true;
                        break;
                    }
                }

                // 如果没有重复，加入点云和滑动窗口
                if (!isDuplicate) {
                    pointCloud.push_back(point_world);
                    recentPoints.push_back(point_world);

                    // 保持滑动窗口大小
                    if (recentPoints.size() > WINDOW_SIZE) {
                        recentPoints.pop_front();
                    }
                }
            }
        }
        auto end = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(end - start);
        cout << "代码运行时间: " << duration.count() << " 毫秒" << endl;
    }

    // 保存点云
    savePointCloud(pointCloud, "output_point_cloud.ply");

    return 0;
}
