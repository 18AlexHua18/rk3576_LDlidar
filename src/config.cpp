#include "../include/config.h"


namespace LidarConfig {
    std::vector<LidarParam> getLidarParams() {
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
