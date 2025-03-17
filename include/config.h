#pragma once

#include <string>
#include <vector>
#include "lidar_types.h"

// 全局配置
namespace GlobalConfig {
    const std::string Version = "1.0.0";
    constexpr int BufferSize = 2048;  // UDP缓冲区大小
}

// 雷达配置
namespace LidarConfig {
    constexpr int listenPort = 6580;  // 默认监听端口
    constexpr const char* multicastAddr = "239.255.0.1";  // 组播地址
    constexpr const char* interfaceName = "eth0";  // 接口名称
    
    // 获取配置的雷达参数列表
    std::vector<LidarParam> getLidarParams();
}

// 雷达数据包相关常量
namespace PacketConfig {
    constexpr int BIG_PACKET_SIZE = 1418;      // 标准数据包大小
    constexpr int MAXCLOUDROW = 192;           // 最大行数
    constexpr int MAXCLOUDCOL = 256 * 3;       // 最大列数
    constexpr int LD_LM_LIDAR_WIDTH = 256;     // 宽
    constexpr int LD_LM_LIDAR_HEIGHT = 192;    // 高
    constexpr int EchoNumberOfPixel = 3;       // 每个像素的回波数
}

// 点云处理配置命名空间
namespace CloudConfig {
    const bool save_enabled = true;           // 是否保存点云
    const std::string save_path = "/usr/download/point_clouds"; // 点云保存路径
    const int save_interval = 10;             // 保存间隔（帧数）
    const bool filter_enabled = true;         // 是否启用滤波
    const float filter_threshold = 0.1f;      // 滤波阈值
}