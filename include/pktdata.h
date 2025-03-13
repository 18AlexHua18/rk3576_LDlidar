#ifndef PKTDATA_H
#define PKTDATA_H

#include <stdint.h>
#include <array>
#include <arpa/inet.h>

// 为了兼容性，从old_project复制相关结构定义
constexpr int BIG_PACKET_SIZE{1418};
constexpr int EchoNumberOfPixel{3};

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

// 包头
struct FrameHeader {
    uint32_t pktHead;
    uint16_t pktCnt;
    uint16_t pktLength;
    uint16_t protocolVersion;
    GPSTimeStamp gpsTime;
    uint8_t timeSyncType;
    uint8_t timeSyncStatus;
    uint16_t productId; // 默认0x01
    uint32_t frameId;
    uint8_t subFrameId; // OFFSET 28
    uint8_t startColId;
    uint8_t endColId;
    uint8_t reserved[33];

    unsigned long long getTimestamp() const
    {
        unsigned long long timestamp;
        timestamp = gpsTime.hour;
        timestamp = timestamp * 60 + gpsTime.minute;
        timestamp = timestamp * 60 + gpsTime.second;
        timestamp = timestamp * 1000 + ntohs((gpsTime.millisecond));
        timestamp = timestamp * 1000 + ntohs((gpsTime.microsecond));
        return timestamp;
    }
};

// 包尾
struct FrameCheckSequence {
    uint8_t reserved[64];
};

// 回波标签
typedef union {
    uint8_t label;
    struct {
        uint8_t reseved0:6; // 保留
        uint8_t echoChose:1; // 回波选择
        uint8_t reseved1:1; // 保留
    } BIT;
} EchoLable;

// 净荷
struct Payload {
    int16_t x[3];
    int16_t y[3];
    int16_t z[3];
    uint16_t dist[3];
    uint32_t intensity[3];
    uint8_t reflectivity[3];
    EchoLable echoLabel[3];
    uint8_t reseved;

    int16_t GetX(uint8_t echo) const
    {
        return x[echo];
    }

    int16_t GetY(uint8_t echo) const
    {
        return y[echo];
    }

    int16_t GetZ(uint8_t echo) const
    {
        return z[echo];
    }

    uint16_t GetDist(uint8_t echo) const
    {
        return dist[echo];
    }

    uint32_t GetIntensity(uint8_t echo) const
    {
        return intensity[echo];
    }

    uint8_t GetReflectivity(uint8_t echo) const
    {
        return reflectivity[echo];
    }

    uint8_t GetEchoChoiseLabel(uint8_t echo) const
    {
        return echoLabel[echo].BIT.echoChose;
    }
};

// 雷达数据包结构定义
struct Gen2Packet {
    FrameHeader head;
    Payload payload[30];
    FrameCheckSequence fcs;
};

#endif // PKTDATA_H