#pragma once

#include <stdint.h>
#include <vector>
#include <string>
#include <cstddef>
#include <sstream>

// 点的数据结构
struct Point3D {
    float x;
    float y;
    float z;
    uint8_t intensity;
    
    Point3D() : x(0.0f), y(0.0f), z(0.0f), intensity(0) {}
    
    Point3D(float _x, float _y, float _z, uint8_t _i) : 
        x(_x), y(_y), z(_z), intensity(_i) {}
};

// 点云数据结构
class PointCloud {
public:
    std::vector<Point3D> points;
    double timestamp;
    uint32_t height;
    uint32_t width;
    bool is_dense;
    uint32_t frame_id; // 添加帧ID，用于跟踪和显示
    
    PointCloud() : timestamp(0.0), height(1), width(0), is_dense(true), frame_id(0) {}
    
    void clear() {
        points.clear();
        width = 0;
    }
    
    void resize(size_t n) {
        points.resize(n);
        width = n;
    }
    
    size_t size() const {
        return points.size();
    }
    
    bool empty() const {
        return points.empty();
    }
    
    void push_back(const Point3D& p) {
        points.push_back(p);
        width++;
    }
};

// 雷达参数结构
struct LidarParam {
    int index;          // 雷达索引
    std::string name;   // 雷达名称
    uint32_t ipaddr;    // IP地址
    float x;            // X坐标偏移
    float y;            // Y坐标偏移
    float z;            // Z坐标偏移
    std::string topic;  // 话题名称
    
    // 默认构造函数
    LidarParam() : index(0), ipaddr(0), x(0.0f), y(0.0f), z(0.0f) {}
    
    // 转换为字符串用于日志输出
    std::string toString() const {
        return "LidarParam{name=" + name + 
               ", ipaddr=" + std::to_string(ipaddr) + 
               ", pos=(" + std::to_string(x) + "," + 
               std::to_string(y) + "," + 
               std::to_string(z) + ")}";
    }
};

// 点云处理配置
struct PointCloudConfig {
    bool save_enabled;
    std::string save_path;
    int save_interval;
    bool filter_enabled;
    float filter_threshold;
    
    PointCloudConfig() : 
        save_enabled(true),
        save_path("/usr/download/point_clouds"),
        save_interval(10),
        filter_enabled(true),
        filter_threshold(0.1f) {}
};