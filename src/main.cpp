#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <string>
#include <thread>
#include <chrono>
#include <map>
#include <vector>
#include <atomic>
#include <sys/stat.h>
#include <iomanip>
#include <sstream>
#include <cstring>

#include "config.h"
#include "logger.h"
#include "lidar_types.h"
#include "ring_buffer.h"
#include "packet_parser.h"
#include "point_cloud.h"
#include "pktdata.h"

// 全局变量
std::atomic<bool> g_running(true);
RingBuffer<std::vector<uint8_t>> g_packet_buffer(5000);
std::map<uint32_t, PacketParser *> g_parsers;
PointCloudProcessor g_processor;
int g_socket_fd = -1;

// 监控计数器
std::atomic<uint64_t> g_dropped_packets(0);
std::atomic<uint64_t> g_received_packets(0);
std::map<uint32_t, std::chrono::steady_clock::time_point> g_last_frame_times;

// 信号处理函数
void signalHandler(int signum)
{
    LD_INFO << "接收到信号 " << signum << "，正在退出...";
    g_running = false;

    // 关闭套接字以中断recvfrom()阻塞
    if (g_socket_fd >= 0)
    {
        close(g_socket_fd);
        g_socket_fd = -1;
    }
}

// 处理线程函数
void processThread()
{
    LD_INFO << "点云处理线程启动";

    std::vector<uint8_t> packet_data;
    while (g_running)
    {
        if (g_packet_buffer.pop(packet_data))
        {
            // 提取发送者IP地址 (假设前4字节包含IP信息)
            uint32_t ipaddr = 0;
            if (packet_data.size() >= 4)
            {
                ipaddr = packet_data[0];
            }

            // 创建或获取对应的解析器用于多雷达测试
            if (g_parsers.find(ipaddr) == g_parsers.end())
            {
                LD_INFO << "新检测到雷达，IP后缀: " << ipaddr;

                // 查找配置中是否有对应IP的雷达参数
                LidarParam param;
                bool found = false;

                // 获取预设的雷达参数
                auto lidarParams = LidarConfig::getLidarParams();
                for (const auto &p : lidarParams)
                {
                    if (p.ipaddr == ipaddr)
                    {
                        param = p;
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    param.ipaddr = ipaddr;
                    param.name = "lidar_" + std::to_string(ipaddr);
                }

                g_parsers[ipaddr] = new PacketParser();
                g_parsers[ipaddr]->setLidarParam(param);

                LD_INFO << "初始化雷达参数: " << param.toString();
            }

            //TODO 在这里可以检查每一个点云处理的时间，如果太长可以考虑写一个自动扩增buffer的机制

            // 解析数据包
            PointCloud cloud;
            if (g_parsers[ipaddr]->parsePacket(packet_data.data() + 4,
                                               packet_data.size() - 4,
                                               cloud))
            {
                // 处理点云
                g_processor.processCloud(cloud);
            }
        }
    }

    LD_INFO << "点云处理线程退出";
}

// 点云回调函数，展示点云信息并可选保存
void cloudCallback(const PointCloud &cloud)
{

}

int main(int argc, char **argv)
{
    LD_INFO << "RK3576 激光雷达点云处理工具 v" << GlobalConfig::Version;

    // 在程序开始时创建保存目录
    PointCloudProcessor::ensureDirectoryExists(CloudConfig::save_path);

    // 设置信号处理
    signal(SIGINT, signalHandler);

    signal(SIGTERM, signalHandler);

    // 解析命令行参数 (端口等)
    int port = LidarConfig::listenPort;
    if (argc > 1)
    {
        port = atoi(argv[1]);
    }
    LD_INFO << "监听端口: " << port;

    // 设置点云回调
    g_processor.setCallback(cloudCallback);

    // 启动处理线程
    std::thread proc_thread(processThread);

    // 创建UDP套接字
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        LD_FATAL << "socket() 失败: " << strerror(errno);
        return 1;
    }

    // 保存到全局变量，以便信号处理函数使用
    g_socket_fd = fd;

    // 设置套接字选项
    int reuse = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
    {
        LD_ERROR << "setsockopt(SO_REUSEADDR) 失败: " << strerror(errno);
    }

    // 设置套接字超时，防止recvfrom无限阻塞
    struct timeval tv;
    tv.tv_sec = 1; // 1秒超时
    tv.tv_usec = 0;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
        LD_ERROR << "setsockopt(SO_RCVTIMEO) 失败: " << strerror(errno);
    }

    // 绑定套接字
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        LD_FATAL << "bind() 失败: " << strerror(errno);
        close(fd);
        return 1;
    }

    // 如果配置了组播地址，则加入组播
    const char *multiaddr = LidarConfig::multicastAddr;
    if (multiaddr && multiaddr[0] != '\0')
    {
        struct ip_mreqn mreqn;
        inet_pton(AF_INET, multiaddr, &mreqn.imr_multiaddr.s_addr);
        inet_pton(AF_INET, "0.0.0.0", &mreqn.imr_address.s_addr);
        mreqn.imr_ifindex = if_nametoindex(LidarConfig::interfaceName);

        if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreqn, sizeof(mreqn)) < 0)
        {
            LD_ERROR << "加入组播组失败: " << strerror(errno);
        }
        else
        {
            LD_INFO << "已加入组播组: " << multiaddr;
        }
    }

    // 接收数据包
    uint8_t udp_buffer[GlobalConfig::BufferSize];
    struct sockaddr_in client;
    socklen_t client_len = sizeof(client);

    LD_INFO << "开始接收数据...";

    while (g_running)
    {
        // 接收一个UDP数据包
        int recvlen = recvfrom(fd, udp_buffer, sizeof(udp_buffer), 0,
                               (struct sockaddr *)&client, &client_len);

        if (recvlen < 0)
        {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // 被信号中断或超时，检查g_running标志
                continue;
            }
            if (!g_running)
            {
                // 程序已经设置为退出状态
                break;
            }
            LD_ERROR << "recvfrom() 失败: " << strerror(errno);
            break;
        }

        // 计数接收的包
        g_received_packets++;

        // 提取IP地址的最后一个字节(IPv4地址最后一段)
        uint32_t ipaddr = ntohl(client.sin_addr.s_addr) & 0xFF; // 只取最后一位

        // 构建包含IP和数据的缓冲区
        std::vector<uint8_t> packet_data(recvlen + 4);
        // 在第一个字节保存IP地址的最后一段，其余三个字节保留为0
        packet_data[0] = (uint8_t)ipaddr;
        packet_data[1] = 0;
        packet_data[2] = 0;
        packet_data[3] = 0;
        // 复制数据包内容
        memcpy(packet_data.data() + 4, udp_buffer, recvlen);

        // 将数据包放入缓冲区
        if (!g_packet_buffer.push(packet_data))
        {
            g_dropped_packets++; // 计数丢弃的包
            LD_WARN << "缓冲区已满，丢弃数据包";
        }
    }

    // 清理资源
    if (g_socket_fd >= 0)
    {
        close(g_socket_fd);
        g_socket_fd = -1;
    }

    // 确保缓冲区不再阻塞处理线程
    g_packet_buffer.setExit(true);

    // 等待处理线程结束
    if (proc_thread.joinable())
    {
        LD_INFO << "等待点云处理线程退出...";
        proc_thread.join();
    }

    // 清理解析器
    for (auto &pair : g_parsers)
    {
        delete pair.second;
    }
    g_parsers.clear();

    // 在程序退出时输出包处理统计信息
    LD_INFO << "程序运行期间接收了 " << g_received_packets.load()
            << " 个数据包，丢弃了 " << g_dropped_packets.load() << " 个数据包";

    LD_INFO << "程序正常退出";
    return 0;
}