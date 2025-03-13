#include "point_cloud.h"
#include "logger.h"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <vector>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstring>
#include <chrono>

PointCloudProcessor::PointCloudProcessor() : is_new_frame_(false) {}

PointCloudProcessor::~PointCloudProcessor() {}

void PointCloudProcessor::setCallback(PointCloudCallback callback) {
    callback_ = callback;
}

void PointCloudProcessor::processCloud(const PointCloud& cloud) {
    if (callback_) {
        callback_(cloud);
    }
}

bool PointCloudProcessor::saveToCSV(const PointCloud& cloud, const std::string& filename) {
    // 保留原来的CSV保存功能，但不推荐使用
    std::ofstream file(filename);
    if (!file.is_open()) {
        LD_ERROR << "Failed to open file for writing: " << filename;
        return false;
    }
    
    // 写入CSV头
    file << "x,y,z,intensity" << std::endl;
    
    // 写入点数据
    for (const auto& point : cloud.points) {
        file << std::fixed << std::setprecision(6)
             << point.x << "," 
             << point.y << "," 
             << point.z << "," 
             << point.intensity << std::endl;
    }
    
    file.close();
    LD_INFO << "Saved " << cloud.points.size() << " points to " << filename;
    return true;
}

bool PointCloudProcessor::ensureDirectoryExists(const std::string& path) {
    struct stat info;
    
    // 如果路径为空或当前目录，则不需要创建
    if (path.empty() || path == "." || path == "./") {
        return true;
    }
    
    // 检查目录是否已存在
    if (stat(path.c_str(), &info) == 0) {
        if (info.st_mode & S_IFDIR) {
            return true; // 目录存在
        }
        return false; // 存在但不是目录
    }
    
    // 尝试创建目录
    if (mkdir(path.c_str(), 0755) == 0) {
        LD_INFO << "创建目录: " << path;
        return true;
    }
    
    LD_ERROR << "创建目录失败: " << path << ", 错误: " << strerror(errno);
    return false;
}

void PointCloudProcessor::WriteCloud(const std::string& file, const pc_pkt_t* cloud, int count) {
    if (count == 0) {
        LD_ERROR << "Cloud empty!!";
        return;
    }

    static char buf[2 * 1024 * 1024]; // 2MB的静态缓冲区
    
    // 写入头部内容
    int offset = snprintf(buf, sizeof(buf), "ply\n"
                                          "format ascii 1.0\n"
                                          "element vertex %d\n"
                                          "property float32 x\n"
                                          "property float32 y\n"
                                          "property float32 z\n"
                                          "end_header\n", count);

    // 写入点数据
    for (int i = 0; i < count; i++) {
        const auto& pt = cloud[i];
        offset += snprintf(buf + offset, sizeof(buf) - offset, "%.4f %.4f %.4f\n", pt.X, pt.Y, pt.Z);
    }

    // 确保目录存在
    std::string dir_path;
    size_t last_slash = file.find_last_of("/\\");
    if (last_slash != std::string::npos) {
        dir_path = file.substr(0, last_slash);
        ensureDirectoryExists(dir_path);
    }

    // 打开文件并一次性写入所有内容
    std::ofstream pc_out(file, std::ios::out | std::ios::binary);
    if (!pc_out) {
        LD_ERROR << "Error opening file " << file;
        return;
    }

    pc_out.write(buf, offset);
    pc_out.close();
    LD_INFO << "Successfully wrote " << count << " points to " << file;
}

void PointCloudProcessor::WriteCloud(const std::string& file, const PointCloud& cloud) {
    // 过滤有效点（非零坐标）
    std::vector<pc_pkt_t> pc_data;
    pc_data.reserve(cloud.points.size()); // 预分配空间避免重分配
    
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& point = cloud.points[i];
        
        // 跳过无效点 (坐标全为0的点)
        if (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f) {
            continue;
        }
        
        pc_pkt_t pc_point;
        pc_point.X = point.x;
        pc_point.Y = point.y;
        pc_point.Z = point.z;
        pc_point.confidence = 0; // 不使用置信度值
        pc_data.push_back(pc_point);
    }
    
    WriteCloud(file, pc_data.data(), pc_data.size());
}

bool PointCloudProcessor::saveToPLY(const PointCloud& cloud, const std::string& filename) {
    // 测试帧率t1
    const auto t1 = std::chrono::steady_clock::now();
    
    if (cloud.points.empty()) {
        LD_ERROR << "点云为空，无法保存到: " << filename;
        return false;
    }
    
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        LD_ERROR << "无法创建文件: " << filename;
        return false;
    }
    
    // 写入PLY头
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "comment LDLidar point cloud\n";
    file << "element vertex " << cloud.points.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "property uchar intensity\n";
    file << "end_header\n";
    
    // 写入点数据
    for (const auto& point : cloud.points) {
        file << point.x << " " << point.y << " " << point.z << " " 
             << static_cast<int>(point.intensity) << "\n";
    }
    
    // 测试帧率t2
    const auto t2 = std::chrono::steady_clock::now();
    const auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
    LD_DEBUG << "写入PLY文件耗时: " << time_cost << " ms, 点数: " << cloud.points.size();
    
    return true;
}

bool PointCloudProcessor::saveWithTimestamp(const PointCloud& cloud, const std::string& directory) {
    // 生成带时间戳的文件名
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);
    
    std::stringstream ss;
    ss << directory << "/cloud_" 
       << std::put_time(now_tm, "%Y%m%d_%H%M%S")
       << "_" << cloud.frame_id << ".ply";
       
    std::string filename = ss.str();
    
    return saveToPLY(cloud, filename);
}

void PointCloudProcessor::clear() {
    // 不再需要清除累积的点云，因为不再累积
}