#pragma once

#include <stdint.h>
#include <arpa/inet.h>

// 定义常量，避免循环依赖
constexpr int EchoNumberOfPixel = 3;

#pragma pack(push, 1)

// GPS时间戳结构
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

// 包头结构
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
    
    // 计算时间戳方法
    uint64_t getTimestamp() const {
        uint64_t timestamp = 0;
        timestamp += static_cast<uint64_t>(gpsTime.hour) * 3600000000; // 小时转微秒
        timestamp += static_cast<uint64_t>(gpsTime.minute) * 60000000; // 分钟转微秒
        timestamp += static_cast<uint64_t>(gpsTime.second) * 1000000;  // 秒转微秒
        timestamp += ntohs(gpsTime.millisecond) * 1000;                // 毫秒转微秒
        timestamp += ntohs(gpsTime.microsecond);                       // 微秒
        return timestamp;
    }
};

// 回波标签结构
typedef union {
    uint8_t label;
    struct {
        uint8_t reserved0:6; // 保留
        uint8_t echoChose:1; // 回波选择
        uint8_t reserved1:1; // 保留
    } BIT;
} EchoLabel;

// 数据负载结构
struct Payload {
    int16_t x[EchoNumberOfPixel];
    int16_t y[EchoNumberOfPixel];
    int16_t z[EchoNumberOfPixel];
    uint16_t dist[EchoNumberOfPixel];
    uint32_t intensity[EchoNumberOfPixel];
    uint8_t reflectivity[EchoNumberOfPixel];
    EchoLabel echoLabel[EchoNumberOfPixel];
    uint8_t reserved;

    // 便捷访问方法
    int16_t GetX(uint8_t echo) const { return x[echo]; }
    int16_t GetY(uint8_t echo) const { return y[echo]; }
    int16_t GetZ(uint8_t echo) const { return z[echo]; }
    uint16_t GetDist(uint8_t echo) const { return dist[echo]; }
    uint32_t GetIntensity(uint8_t echo) const { return intensity[echo]; }
    uint8_t GetReflectivity(uint8_t echo) const { return reflectivity[echo]; }
    uint8_t GetEchoChoiseLabel(uint8_t echo) const { return echoLabel[echo].BIT.echoChose; }
};

// 帧校验结构
struct FrameCheckSequence {
    uint8_t reserved[64];
};

// 完整的数据包结构
struct Gen2Packet {
    FrameHeader head;
    Payload payload[30];
    FrameCheckSequence fcs;
};

#pragma pack(pop)