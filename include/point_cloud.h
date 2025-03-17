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
    float x;
    float y;
    float z;
    uint8_t intensity;
};

// 点云处理回调函数类型
typedef std::function<void(const PointCloud&)> PointCloudCallback;

class PointCloudProcessor {
public:
    PointCloudProcessor();
    ~PointCloudProcessor();
    
    // 设置点云处理回调
    void setCallback(PointCloudCallback callback);
    
    // 处理点云数据
    void processCloud(const PointCloud& cloud);
    
    // 设置是否有新一帧点云的标志
    void setNewFrameFlag(bool is_new_frame) {
        is_new_frame_ = is_new_frame;
    }
    
    // 获取是否为新的一帧点云
    bool isNewFrame() const {
        return is_new_frame_;
    }
    
    // 将点云写入文件
    void WriteCloud(const std::string& file, const pc_pkt_t* cloud, int count);
    
    // 将点云写入文件（重载版本，修改参数顺序）
    bool WriteCloud(const PointCloud& cloud, const std::string& directory);

private:
    PointCloudCallback callback_;
    int file_index_;
    bool is_new_frame_; // 标记当前点云是否为新的一帧
    
    // 确保目录存在
    bool ensureDirectoryExists(const std::string& path);

};

#endif // POINT_CLOUD_H