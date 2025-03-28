#include "packet_parser.h"
#include "logger.h"
#include "config.h"
#include <cmath>
#include <cstring>
#include <cstddef>
#include <arpa/inet.h>

// 检查是否定义了点云过滤宏
#ifndef ENABLE_POINT_FILTERING
#define ENABLE_POINT_FILTERING 0
#endif

PacketParser::PacketParser()
    : currentFrameId(0), frameInProgress(false),
      cloudWidth(PacketConfig::LD_LM_LIDAR_WIDTH),
      cloudHeight(PacketConfig::LD_LM_LIDAR_HEIGHT),
      packetCount(0), processed_points_(0)
{
    // 初始化算法参数
    algorithmParam.EnableEchoChose = 1;
    algorithmParam.EchoNumber = 5;

    // 初始化点云存储空间
    points.resize(cloudWidth * cloudHeight * PacketConfig::EchoNumberOfPixel);

    // 初始化3D点云数据结构
    pointData_.resize(cloudHeight);
    for (int i = 0; i < cloudHeight; i++)
    {
        pointData_[i].resize(cloudWidth);
        for (int j = 0; j < cloudWidth; j++)
        {
            pointData_[i][j].resize(PacketConfig::EchoNumberOfPixel);
        }
    }
}

PacketParser::~PacketParser()
{
    // 清理资源
}

void PacketParser::setLidarParam(const LidarParam &param)
{
    lidarParam = param;
    LD_INFO << "设置雷达参数: " << param.toString();
}

bool PacketParser::isValidMessage(size_t size)
{
    if (size != PacketConfig::BIG_PACKET_SIZE)
    {
        LD_ERROR << "无效的数据包大小: " << size << " 字节, 预期: "
                 << PacketConfig::BIG_PACKET_SIZE << " 字节";
        return false;
    }
    return true;
}

bool PacketParser::parsePacket(const uint8_t *data, size_t size, PointCloud &cloud)
{
    // 检查是否为有效的雷达数据包
    if (!isValidMessage(size))
    {
        return false;
    }

    // 重新映射数据以便访问
    const Gen2Packet *packet = reinterpret_cast<const Gen2Packet *>(data);

    // 检查是否是新的帧ID
    bool isNewFrame = false;
    if (frameInProgress && ntohl(packet->head.frameId) != currentFrameId)
    {
        LD_WARN << "检测到新帧ID(" << ntohl(packet->head.frameId)
                << ")，但上一帧(" << currentFrameId

        << ")仅接收到" << packets_[currentFrameId] << "个包，强制结束上一帧";

        // 处理未完成的上一帧数据
        if (packets_[currentFrameId] > 0)
        {
            buildPointCloud(cloud);
            LD_WARN << "强制结束上一帧，由" << packets_[currentFrameId] << "个包构建，点云大小：" << cloud.points.size();
            isNewFrame = true;
        }

        // 重置为新帧
        frameInProgress = false;
    }

    // 如果不是在处理一个帧或者是新帧，则初始化新帧
    if (!frameInProgress || ntohl(packet->head.frameId) != currentFrameId)
    {
        currentFrameId = ntohl(packet->head.frameId);
        frameInProgress = true;
        packets_[currentFrameId] = 0;
        frameCloud.points.clear();
        receivedSubframes.clear(); // 清除子帧记录
        LD_INFO << "开始新帧: 帧ID=" << currentFrameId;
    }

    // 增加当前帧的包计数
    packets_[currentFrameId]++;

    // 记录已收到的子帧和起始列
    receivedSubframes.insert(std::make_pair(packet->head.subFrameId, packet->head.startColId));

    // 处理有效的雷达数据包
    processPacket(packet);

    // 如果前面已经处理了未完成的旧帧，则直接返回
    if (isNewFrame)
    {
        return true;
    }

    // 判断帧是否结束，简化为检测是否达到1664个包
    bool frame_end = (packets_[currentFrameId] >= 1664);

    if (frame_end)
    {
        LD_INFO << "检测到帧结束，帧ID: " << ntohl(packet->head.frameId)
                << ", 包数: " << packets_[currentFrameId];

        // 构建点云
        buildPointCloud(cloud);

        LD_WARN << "！！！点云构建完成，由" << packets_[currentFrameId] << "个包构建，点云大小：" << cloud.points.size();

        // 检查点云大小
        if (cloud.points.empty())
        {
            LD_WARN << "帧完整，但点云构建后为空！检查点云过滤条件。";

            // 添加额外诊断信息
            int validPointCount = 0;
            for (int row = 0; row < cloudHeight; ++row)
            {
                for (int col = 0; col < cloudWidth; ++col)
                {
                    for (int echo = 0; echo < 3; ++echo)
                    {
                        Point3D &point = pointData_[row][col][echo];
                        if (point.x != 0.0f || point.y != 0.0f || point.z != 0.0f)
                        {
                            cloud.points.push_back(point);
                            validPointCount++;
                        }
                    }
                }
            }

            if (validPointCount > 0)
            {
                LD_INFO << "手动添加有效点后，点云大小: " << cloud.points.size();
            }
        }
        else
        {
            LD_INFO << "帧完整，点云大小: " << cloud.points.size();
        }

        // 重置当前帧
        frameInProgress = false;
        return true;
    }

    return false;
}

void PacketParser::transformPoint(float &x, float &y, float &z)
{
    float temp_x = x;
    float temp_y = y;
    float temp_z = z;

    // 使用lidarParam进行点云变换
    x = temp_x;
    y = temp_y;
    z = temp_z;

    // 如果lidarParam有坐标偏移，则应用它
    if (lidarParam.x != 0.0f || lidarParam.y != 0.0f || lidarParam.z != 0.0f)
    {
        x += lidarParam.x;
        y += lidarParam.y;
        z += lidarParam.z;
    }

    // 这里可以添加更复杂的变换，如旋转等
}

// 构建点云
void PacketParser::buildPointCloud(PointCloud &cloud)
{
    LD_DEBUG << "开始构建点云，帧ID: " << currentFrameId;

    cloud.clear();
    cloud.frame_id = currentFrameId;

    int validPointCount = 0;
    int invalidPointCount = 0;
    int zeroPointCount = 0;

    // 预先分配空间以提高效率
    cloud.points.reserve(cloudWidth * cloudHeight * EchoNumberOfPixel / 2);

    // 遍历点云数据
    for (int row = 0; row < cloudHeight; ++row)
    {
        for (int col = 0; col < cloudWidth; ++col)
        {
            for (int echo = 0; echo < EchoNumberOfPixel; ++echo)
            {
                Point3D &point = pointData_[row][col][echo];

                // 筛选有效点
                if (point.x != 0.0f || point.y != 0.0f || point.z != 0.0f)
                {
                    cloud.points.push_back(point);
                    validPointCount++;
                }
                else
                {
                    zeroPointCount++;
                }
            }
        }
    }

    // 更新点云元数据
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;

    LD_INFO << "点云构建完成，有效点: " << validPointCount
            << ", 无效点: " << invalidPointCount
            << ", 原始零点: " << zeroPointCount;
}

// 检查是否是一帧的结束
bool PacketParser::isFrameEnd(const Gen2Packet *packet)
{
    // 帧结束标志：子帧ID为31且起始列为255
    bool isEnd = (packet->head.subFrameId == 31 && packet->head.startColId == 255);

    if (isEnd)
    {
        LD_INFO << "检测到帧结束标记(sub=31): FrameId=" << ntohl(packet->head.frameId);
    }
    return isEnd;
}

// 处理数据包
void PacketParser::processPacket(const Gen2Packet *packet)
{
    // 提取子帧ID和起始列ID
    uint8_t subFrameId = packet->head.subFrameId;
    uint8_t startColId = packet->head.startColId;

    // 更新最大子帧ID和起始列ID
    if (subFrameId > maxSubFrameId)
        maxSubFrameId = subFrameId;
    if (startColId > maxStartColId)
        maxStartColId = startColId;

    LD_DEBUG << "处理子帧: " << (int)subFrameId << ", 起始列: " << (int)startColId;

    // 获取列数
    int colNum = (startColId == 255) ? 1 : 5;

    // 处理每一列和每一行的数据
    for (int col = 0; col < colNum; ++col)
    {
        for (int row = 0; row < 6; ++row)
        {
            const Payload &payload = packet->payload[col * 6 + row];

            // 计算当前行和列的全局索引
            int curRow = subFrameId * 6 + row;
            int curCol = startColId + col;

            // 确保不越界
            if (curRow >= cloudHeight || curCol >= cloudWidth)
            {
                continue;
            }

            // 处理每个回波的数据
            for (int echoId = 0; echoId < EchoNumberOfPixel; ++echoId)
            {
                // 从payload中读取原始坐标数据
                int16_t x = ntohs(payload.GetX(echoId));
                int16_t y = ntohs(payload.GetY(echoId));
                int16_t z = ntohs(payload.GetZ(echoId));
                uint8_t intensity = payload.GetReflectivity(echoId);

                // 计算点的索引
                int globalIndex = (curRow * cloudWidth + curCol) * EchoNumberOfPixel + echoId;

                // 确保不越界
                if (globalIndex >= points.size())
                {
                    continue;
                }

                // 转换坐标
                float x_f = static_cast<float>(x / 512.0);
                float y_f = static_cast<float>(y / 512.0);
                float z_f = static_cast<float>(z / 512.0);

                // 保存坐标和强度
                Point3D &point = pointData_[curRow][curCol][echoId];

#if ENABLE_POINT_FILTERING
                // 应用回波选择算法（仅在启用过滤时）
                bool validPoint = true;
                if (algorithmParam.EnableEchoChose == 1)
                {
                    if (payload.GetEchoChoiseLabel(echoId) == 0)
                    {
                        validPoint = false;
                    }
                }
                else if (algorithmParam.EchoNumber >= 1 && algorithmParam.EchoNumber <= 3)
                {
                    if (echoId != algorithmParam.EchoNumber - 1)
                    {
                        validPoint = false;
                    }
                }
                else if (algorithmParam.EchoNumber == 4)
                {
                    if (echoId > 2)
                    {
                        validPoint = false;
                    }
                }

                if (validPoint)
                {
                    // 应用坐标变换
                    float new_x = x_f;
                    float new_y = y_f;
                    float new_z = z_f;

                    transformPoint(new_x, new_y, new_z);

                    point.x = new_x;
                    point.y = new_y;
                    point.z = new_z;
                    point.intensity = intensity;
                    processed_points_++;
                }
                else
                {
                    // 无效点设置为零
                    point.x = 0.0f;
                    point.y = 0.0f;
                    point.z = 0.0f;
                    point.intensity = 0;
                }
#else
                // 禁用过滤，所有点都保留
                // 应用坐标变换
                float new_x = x_f;
                float new_y = y_f;
                float new_z = z_f;

                transformPoint(new_x, new_y, new_z);

                point.x = new_x;
                point.y = new_y;
                point.z = new_z;
                point.intensity = intensity;
                processed_points_++;
#endif
            }
        }
    }
}

// 输出诊断信息
void PacketParser::printDiagnostics()
{
    LD_INFO << "诊断信息：已处理点数=" << processed_points_
            << ", 当前帧ID=" << currentFrameId
            << ", 当前帧包数=" << (packets_.count(currentFrameId) ? packets_[currentFrameId] : 0)
            << ", 最大子帧ID=" << (int)maxSubFrameId
            << ", 最大起始列ID=" << (int)maxStartColId;
}
