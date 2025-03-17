#pragma once

#include <stdint.h>
#include <vector>
#include <map>
#include <memory>
#include <set>
#include <arpa/inet.h>
#include "pktdata.h"  // 先包含数据包定义
#include "point_cloud.h"
#include "lidar_types.h"

// 算法参数结构
struct AlgorithmParam {
    int EnableEchoChose;
    int EchoNumber;
    
    AlgorithmParam() : EnableEchoChose(1), EchoNumber(5) {}
};

class PacketParser {
public:
    PacketParser();
    ~PacketParser();

    // 设置激光雷达参数
    void setLidarParam(const LidarParam& param);
    
    // 解析数据包，如果返回true表示一帧完成
    bool parsePacket(const uint8_t* data, size_t size, PointCloud& cloud);
    
    // 获取当前点云数据
    const PointCloud& getPointCloud() const { return frameCloud; }
    
    // 添加设置调试模式的功能
    void setDebugMode(bool enabled) { debugMode = enabled; }
    
    // 输出当前处理状态的诊断信息
    void printDiagnostics();

    // 添加调试工具方法
    void dumpFrameData(const std::string& filename);
    
private:
    // 解析数据包并更新点云
    void processPacket(const Gen2Packet* packet);
    
    // 检查是否为有效的消息格式
    bool isValidMessage(size_t size);
    
    // 检查是否是一帧的结束
    bool isFrameEnd(const Gen2Packet* packet);
    
    // 构建点云
    void buildPointCloud(PointCloud& cloud);
    
    // 转换点的坐标
    void transformPoint(float& x, float& y, float& z);
    
    // 算法参数
    AlgorithmParam algorithmParam;
    
    LidarParam lidarParam;  // 激光雷达参数
    PointCloud frameCloud;  // 当前帧点云
    uint32_t currentFrameId;  // 当前帧ID
    bool frameInProgress;  // 是否正在处理中的帧
    
    // 点云缓存，存储所有点
    std::vector<Point3D> points;
    
    // 当前点云的宽度、高度
    int cloudWidth;
    int cloudHeight;
    
    // 用于判断帧是否完整的计数器
    int packetCount;
    
    // 处理点的计数器
    uint64_t processed_points_;
    
    // 记录当前帧的所有subFrameId和startColId
    std::set<std::pair<uint8_t, uint8_t>> receivedSubframes;
    
    // 跟踪每帧包数
    std::map<uint32_t, int> packets_;
    
    // 3D点云数据存储 [行][列][回波]
    std::vector<std::vector<std::vector<Point3D>>> pointData_;
    
    bool debugMode = false; // 调试模式开关
    
    // 帧丢包率统计
    int totalPacketsExpected = 0;
    int totalPacketsReceived = 0;

    // 增加计数器来记录最大子帧ID和最大startColId
    uint8_t maxSubFrameId = 0;
    uint8_t maxStartColId = 0;
};