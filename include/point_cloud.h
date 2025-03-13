#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "lidar_types.h"
#include <functional>
#include <string>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sstream>

// 点云数据PLY格式结构体
struct pc_pkt_t {
    float X;
    float Y;
    float Z;
    float confidence;
};

// 点云处理回调函数类型
typedef std::function<void(const PointCloud&)> PointCloudCallback;

class PointCloudProcessor {
public:
    PointCloudProcessor();
    ~PointCloudProcessor();
    
    // 设置回调函数
    void setCallback(PointCloudCallback callback);
    
    // 处理点云数据 - 直接处理单帧点云, 不再累积
    void processCloud(const PointCloud& cloud);
    
    // 保存点云到CSV文件 (将被PLY格式替代)
    bool saveToCSV(const PointCloud& cloud, const std::string& filename);
    
    // 保存点云到PLY文件
    bool saveToPLY(const PointCloud& cloud, const std::string& filename);
    
    // 使用当前时间戳生成PLY文件并保存
    bool saveWithTimestamp(const PointCloud& cloud, const std::string& log_file_path);
    
    // 清除点云数据
    void clear();
    
    // 设置是否有新一帧点云的标志
    void setNewFrameFlag(bool is_new_frame) {
        is_new_frame_ = is_new_frame;
    }
    
    // 获取是否为新的一帧点云
    bool isNewFrame() const {
        return is_new_frame_;
    }
    
private:
    PointCloudCallback callback_;
    int file_index_;
    bool is_new_frame_; // 标记当前点云是否为新的一帧
    
    // 检查并创建目录
    bool ensureDirectoryExists(const std::string& path);
    
    // 写入点云数据到PLY文件 - 优化缓冲区管理
    void WriteCloud(const std::string& file, const pc_pkt_t* cloud, int count);
    void WriteCloud(const std::string& file, const PointCloud& cloud);
};

#endif // POINT_CLOUD_H