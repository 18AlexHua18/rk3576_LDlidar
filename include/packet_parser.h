#pragma once

#include <stdint.h>
#include <vector>
#include <map>
#include <memory>
#include <set>
#include "point_cloud.h"
#include "lidar_types.h"

// 从旧代码中提取的数据定义
#define MAXCLOUDROW 192
#define MAXCLOUDCOL (256 * 3)
#define LD_LM_LIDAR_WIDTH 256  // 宽
#define LD_LM_LIDAR_HEIGHT 192 // 高
#define EchoNumberOfPixel 3

#pragma pack(push, 1)

struct GPSTimeStamp {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
    uint16_t microsecond;
};

struct FrameHeader {
    uint32_t pktHead;
    uint16_t pktCnt;
    uint16_t pktLength;
    uint16_t protocolVersion;
    GPSTimeStamp gpsTime;
    uint8_t timeSyncType;
    uint8_t timeSyncStatus;
    uint16_t productId;
    uint32_t frameId;
    uint8_t subFrameId;
    uint8_t startColId;
    uint8_t endColId;
    uint8_t reserved[33];
};

typedef union {
    uint8_t label;
    struct {
        uint8_t reserved0:6;
        uint8_t echoChose:1;
        uint8_t reserved1:1;
    } BIT;
} EchoLabel;

struct Payload {
    int16_t x[3];
    int16_t y[3];
    int16_t z[3];
    uint16_t dist[3];
    uint32_t intensity[3];
    uint8_t reflectivity[3];
    EchoLabel echoLabel[3];
    uint8_t reserved;

    // 添加以下便捷方法以匹配旧代码
    int16_t GetX(uint8_t echo) const { return x[echo]; }
    int16_t GetY(uint8_t echo) const { return y[echo]; }
    int16_t GetZ(uint8_t echo) const { return z[echo]; }
    uint16_t GetDist(uint8_t echo) const { return dist[echo]; }
    uint32_t GetIntensity(uint8_t echo) const { return intensity[echo]; }
    uint8_t GetReflectivity(uint8_t echo) const { return reflectivity[echo]; }
    uint8_t GetEchoChoiseLabel(uint8_t echo) const { return echoLabel[echo].BIT.echoChose; }
};

struct FrameCheckSequence {
    uint8_t reserved[64];
};

struct Gen2Packet {
    FrameHeader head;
    Payload payload[30];
    FrameCheckSequence fcs;
};

#pragma pack(pop)

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

private:
    // 解析数据包并更新点云
    void processPacket(const Gen2Packet* packet);
    
    // 设置点云数据点
    void setCloudPoint(const Gen2Packet* packet);
    
    // 检查是否为有效的消息格式
    bool isValidMessage(size_t size);
    
    // 检查是否是一帧的结束
    bool isFrameEnd(const Gen2Packet* packet);
    
    // 构建点云
    void buildPointCloud(PointCloud& cloud);
    
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
    
    // 记录当前帧的所有subFrameId和startColId
    std::set<std::pair<uint8_t, uint8_t>> receivedSubframes;
    
    // 跟踪每帧包数
    std::map<uint32_t, int> packets_;
    
    // 3D点云数据存储 [行][列][回波]
    std::vector<std::vector<std::vector<Point3D>>> pointData_;
};