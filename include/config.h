#pragma once

#include <string>
#include <vector>
#include "lidar_types.h"

// 启用性能测试 (帧率测试)
#define ENABLE_TEST_FPS

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
    inline std::vector<LidarParam> getLidarParams() {
        std::vector<LidarParam> params;
        
        // 添加预设的雷达参数
        LidarParam param1;
        param1.index = 0;
        param1.name = "lidar_front";
        param1.topic = "points_front";
        param1.ipaddr = 10;  // 最后一个IP段
        params.push_back(param1);
        
        return params;
    }
}