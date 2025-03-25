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
#include <config.h>

PointCloudProcessor::PointCloudProcessor() : is_new_frame_(false) {}

PointCloudProcessor::~PointCloudProcessor() {}

void PointCloudProcessor::setCallback(PointCloudCallback callback)
{
    callback_ = callback;
}



void PointCloudProcessor::processCloud(const PointCloud &cloud)
{
    // 判断是否是完整的一帧点云
    if (cloud.is_dense || (!cloud.points.empty() && cloud.width > 0))
    {
        // 获取当前时间的年月日作为子目录
        std::time_t now = std::time(nullptr);
        std::tm *now_tm = std::localtime(&now);

        std::stringstream ss;
        ss << CloudConfig::save_path << std::put_time(now_tm, "%Y%m%d");
        std::string save_dir = ss.str();

        // 保存点云到文件
        if (WriteCloud(cloud, save_dir))
        {
            LD_INFO << "成功保存完整点云帧 ID: " << cloud.frame_id;
        }
    }

    // 调用回调函数进行其他处理
    if (callback_)
    {
        callback_(cloud);
    }
}

bool PointCloudProcessor::ensureDirectoryExists(const std::string &path)
{
    struct stat info;

    // 如果路径为空或当前目录，则不需要创建
    if (path.empty() || path == "." || path == "./")
    {
        return true;
    }

    // 检查目录是否已存在
    if (stat(path.c_str(), &info) == 0)
    {
        // 路径存在，检查是否是目录
        if (info.st_mode & S_IFDIR)
        {
            return true; // 目录已存在
        }
        else
        {
            // 存在但不是目录，尝试删除并重新创建
            LD_WARN << "路径存在但不是目录，尝试删除并重新创建: " << path;
            if (remove(path.c_str()) != 0)
            {
                LD_ERROR << "无法删除非目录文件: " << path << ", 错误: " << strerror(errno);
                return false;
            }
            // 文件已删除，下面会继续尝试创建目录
        }
    }
    else if (errno != ENOENT)
    {
        // 如果错误不是"不存在"，而是其他错误（如权限问题），则返回失败
        LD_ERROR << "检查目录时出错: " << path << ", 错误: " << strerror(errno);
        return false;
    }

    // 到这里说明目录不存在或已被删除，先递归创建父目录
    size_t pos = path.find_last_of('/');
    if (pos != std::string::npos && pos > 0) // 确保pos > 0避免根目录问题
    {
        std::string parent = path.substr(0, pos);
        if (!parent.empty() && !ensureDirectoryExists(parent))
        {
            return false; // 创建父目录失败
        }
    }

    // 创建当前目录
    int result = mkdir(path.c_str(), 0755);
    if (result == 0)
    {
        LD_INFO << "创建目录: " << path;
        return true;
    }
    else
    {
        // 如果失败，但失败原因是目录已存在，仍然认为成功
        if (errno == EEXIST)
        {
            // 再次检查是否为目录
            if (stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR))
            {
                return true;
            }
        }
        LD_ERROR << "创建目录失败: " << path << ", 错误: " << strerror(errno);
        return false;
    }
}

// 实现新的WriteCloud方法
bool PointCloudProcessor::WriteCloud(const PointCloud &cloud, const std::string &directory)
{
    // 记录开始时间以测量性能
    const auto t1 = std::chrono::steady_clock::now();

    if (cloud.points.empty())
    {
        LD_ERROR << "点云为空，无法保存";
        return false;
    }

    // 确保目录存在
    if (!ensureDirectoryExists(directory))
    {
        return false;
    }

    // 生成带时间戳的文件名
    std::time_t now = std::time(nullptr);
    std::tm *now_tm = std::localtime(&now);

    std::stringstream ss;
    ss << directory << "/cloud_"
       << std::put_time(now_tm, "%Y%m%d_%H%M%S")
       << "_" << cloud.frame_id << ".ply";

    std::string filename = ss.str();

    // 打开文件并写入内容
    std::ofstream file(filename, std::ios::out);
    if (!file)
    {
        LD_ERROR << "无法创建文件: " << filename;
        return false;
    }

    // 写入PLY头
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "comment LDLidar point cloud\n";
    file << "comment Frame ID: " << cloud.frame_id << "\n";
    file << "element vertex " << cloud.points.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "property uchar intensity\n";
    file << "end_header\n";

    // 写入点数据
    int valid_count = 0;
    for (const auto &point : cloud.points)
    {
        // 跳过无效点
        if (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f)
        {
            continue;
        }

        file << point.x << " " << point.y << " " << point.z << " "
             << static_cast<int>(point.intensity) << "\n";
        valid_count++;
    }

    file.close();

    // 测量处理时间
    const auto t2 = std::chrono::steady_clock::now();
    const auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    LD_INFO << "\n[SAVE]保存点云到: " << filename << ", 有效点数: " << valid_count
            << ", 耗时: " << time_cost << " ms\n";

    return true;
}
