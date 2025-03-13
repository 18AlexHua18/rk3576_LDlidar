#include "packet_parser.h"
#include "logger.h"
#include <arpa/inet.h>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <cstddef>  // 添加对size_t的支持
#include <netinet/in.h>

// 包大小常量
constexpr int BIG_PACKET_SIZE = 1418;

PacketParser::PacketParser() 
    : currentFrameId(0), frameInProgress(false), 
      cloudWidth(LD_LM_LIDAR_WIDTH), cloudHeight(LD_LM_LIDAR_HEIGHT),
      packetCount(0)
{
    // 初始化算法参数
    algorithmParam.EnableEchoChose = 1;
    algorithmParam.EchoNumber = 5;
    
    // 初始化点云存储空间
    points.resize(cloudWidth * cloudHeight * EchoNumberOfPixel);
    
    // 初始化3D点云数据结构
    pointData_.resize(cloudHeight);
    for (int i = 0; i < cloudHeight; i++) {
        pointData_[i].resize(cloudWidth);
        for (int j = 0; j < cloudWidth; j++) {
            pointData_[i][j].resize(EchoNumberOfPixel);
        }
    }
}

PacketParser::~PacketParser() {
    // 清理资源
}

void PacketParser::setLidarParam(const LidarParam& param) {
    this->lidarParam = param;
    LD_INFO << "设置雷达参数: " << param.toString();
}

bool PacketParser::parsePacket(const uint8_t* data, size_t size, PointCloud& cloud) {
    // 检查是否为有效的雷达数据包
    if (!isValidMessage(size)) {
        return false;
    }

    const Gen2Packet* packet = reinterpret_cast<const Gen2Packet*>(data);
    
    // 检查帧ID，如果是新帧则重置
    if (!frameInProgress || packet->head.frameId != currentFrameId) {
        currentFrameId = packet->head.frameId;
        frameInProgress = true;
        packets_[currentFrameId] = 0;
        frameCloud.points.clear();
    }
    
    // 增加当前帧的包计数
    packets_[currentFrameId]++;
    
    // 添加日志以检查包的关键信息
    LD_DEBUG << "处理数据包: 子帧ID=" << (int)packet->head.subFrameId 
             << ", 起始列=" << (int)packet->head.startColId 
             << ", 帧ID=" << packet->head.frameId;

    // 处理有效的雷达数据包
    processPacket(packet);
    
    // 检查是否为当前帧的结束包
    bool frame_end = isFrameEnd(packet);
    if (frame_end) {
        LD_INFO << "检测到帧结束标记，帧ID: " << packet->head.frameId
                << ", 包数: " << packets_[currentFrameId];
        
        // 检查帧完整性 - 确保我们收到了足够多的包
        if (packets_[currentFrameId] >= 1600) { // 一个完整帧通常有约1664个包
            // 构建点云
            buildPointCloud(cloud);
            
            // 检查点云大小
            if (cloud.points.empty()) {
                LD_WARN << "帧完整，但点云构建后为空！检查点云过滤条件。";
                
                // 添加额外诊断信息
                int validPointCount = 0;
                for (int row = 0; row < cloudHeight; ++row) {
                    for (int col = 0; col < cloudWidth; ++col) {
                        for (int echo = 0; echo < 3; ++echo) {
                            // 获取对应位置的点
                            Point3D& point = pointData_[row][col][echo];
                            // 计算有效点(非零点)数量
                            if (point.x != 0.0f || point.y != 0.0f || point.z != 0.0f) {
                                validPointCount++;
                            }
                        }
                    }
                }
                LD_INFO << "有效点(非零坐标)数量: " << validPointCount;
            } else {
                LD_INFO << "帧完整，点云大小: " << cloud.points.size();
            }

            // 重置当前帧
            frameInProgress = false;
            return true;
        } else {
            LD_WARN << "帧数据不完整，接收到 " << packets_[currentFrameId] 
                    << " 个包，预期至少 1600 个";
            // 重置当前帧
            frameInProgress = false;
            return false;
        }
    }
    
    return false;
}

void PacketParser::buildPointCloud(PointCloud& cloud) {
    cloud.points.clear();
    cloud.frame_id = currentFrameId;
    
    // 添加诊断日志
    LD_DEBUG << "开始构建点云，帧ID: " << currentFrameId;
    
    // 计数有效点和无效点
    int validPoints = 0;
    int invalidPoints = 0;
    
    for (int row = 0; row < cloudHeight; ++row) {
        for (int col = 0; col < cloudWidth; ++col) {
            for (int echo = 0; echo < EchoNumberOfPixel; ++echo) { // 每个像素有3个回波
                // 获取对应位置的点
                Point3D& point = pointData_[row][col][echo];
                
                // 跳过无效点 (坐标全为0的点)
                if (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f) {
                    invalidPoints++;
                    continue;
                }
                
                // 添加有效点到点云
                cloud.points.push_back(point);
                validPoints++;
            }
        }
    }
    
    LD_DEBUG << "点云构建完成，有效点: " << validPoints << ", 无效点: " << invalidPoints;
}

bool PacketParser::isValidMessage(size_t size) {
    return size >= sizeof(Gen2Packet);
}

bool PacketParser::isFrameEnd(const Gen2Packet* packet) {
    // 帧的最后一个包: subFrameId = 31, startColId = 255
    return (packet->head.subFrameId == 31 && packet->head.startColId == 255);
}

void PacketParser::processPacket(const Gen2Packet* packet) {
    uint8_t subFrameId = packet->head.subFrameId;
    uint8_t startCol = packet->head.startColId;
    int curRow = 0;
    int curCol = 0;
    int colNum = (startCol == 255) ? 1 : 5; // 根据老代码逻辑调整
    
    // 添加调试日志
    LD_DEBUG << "处理子帧: " << (int)subFrameId << ", 起始列: " << (int)startCol;
    
    // 遍历数据包中的有效载荷
    for (int col = 0; col < colNum; ++col) {
        for (int row = 0; row < 6; ++row) {
            const Payload& payload = packet->payload[col * 6 + row];
            curRow = subFrameId * 6 + row;
            curCol = startCol + col;
            
            // 确保索引在有效范围内
            if (curRow >= 0 && curRow < cloudHeight && 
                curCol >= 0 && curCol < cloudWidth) {
                
                // 处理每个回波
                for (int echoId = 0; ++echoId < EchoNumberOfPixel; ++echoId) {
                    // 参照旧项目中的点云提取逻辑，从payload中获取xyz和intensity
                    int16_t x = ntohs(payload.GetX(echoId));
                    int16_t y = ntohs(payload.GetY(echoId));
                    int16_t z = ntohs(payload.GetZ(echoId));
                    uint8_t intensity = payload.GetReflectivity(echoId); 
                    
                    // 将点数据保存到点云数组
                    Point3D& point = pointData_[curRow][curCol][echoId];
                    point.x = static_cast<float>(x) / 512.0f;
                    point.y = static_cast<float>(y) / 512.0f;
                    point.z = static_cast<float>(z) / 512.0f;
                    point.intensity = intensity;
                    
                    // 根据回波选择算法判断是否保留该点
                    if (algorithmParam.EnableEchoChose && payload.GetEchoChoiseLabel(echoId) == 0) {
                        // 置为零点（无效点）
                        point.x = 0.0f;
                        point.y = 0.0f;
                        point.z = 0.0f;
                        point.intensity = 0;
                    }
                }
            }
        }
    }
}

void PacketParser::setCloudPoint(const Gen2Packet* packet) {
    uint8_t subFrameId = packet->head.subFrameId;
    uint8_t startCol = packet->head.startColId;
    int colNum = (startCol == 255) ? 1 : 5;
    
    for (int col = 0; col < colNum; ++col) {
        for (int row = 0; row < 6; ++row) {
            const Payload& payload = packet->payload[col * 6 + row];
            int curRow = subFrameId * 6 + row;
            int curCol = startCol + col;
            
            // 确保索引有效
            if (curRow >= MAXCLOUDROW || curCol >= LD_LM_LIDAR_WIDTH) {
                continue;
            }
            
            for (int echoId = 0; echoId < EchoNumberOfPixel; ++echoId) {
                int16_t x = ntohs(payload.x[echoId]);
                int16_t y = ntohs(payload.y[echoId]);
                int16_t z = ntohs(payload.z[echoId]);
                uint8_t intensity = payload.reflectivity[echoId];
                
                // 计算点的索引
                int globalIndex = (curRow * LD_LM_LIDAR_WIDTH + curCol) * EchoNumberOfPixel + echoId;
                
                // 确保不越界
                if (globalIndex >= points.size()) {
                    continue;
                }
                
                // 转换坐标
                float x_f = static_cast<float>(x / 512.0);
                float y_f = static_cast<float>(y / 512.0);
                float z_f = static_cast<float>(z / 512.0);
                
                // 应用回波选择算法
                bool validPoint = true;
                if (algorithmParam.EnableEchoChose == 1) {
                    if (payload.echoLabel[echoId].BIT.echoChose == 0) {
                        validPoint = false;
                    }
                } else if (algorithmParam.EchoNumber == 1 || 
                          algorithmParam.EchoNumber == 2 || 
                          algorithmParam.EchoNumber == 3) {
                    if (echoId != algorithmParam.EchoNumber - 1) {
                        validPoint = false;
                    }
                } else if (algorithmParam.EchoNumber == 4) {
                    if (echoId > 2) {
                        validPoint = false;
                    }
                }
                
                // 如果是有效点，则添加到当前帧的点云中
                if (validPoint && (x_f != 0 || y_f != 0 || z_f != 0)) {
                    // 变换坐标
                    float transformed_x = x_f + lidarParam.x;
                    float transformed_y = y_f + lidarParam.y;
                    float transformed_z = z_f + lidarParam.z;
                    
                    Point3D point;
                    point.x = transformed_x;
                    point.y = transformed_y;
                    point.z = transformed_z;
                    point.intensity = intensity;
                    
                    frameCloud.points.push_back(point);
                }
            }
        }
    }
}