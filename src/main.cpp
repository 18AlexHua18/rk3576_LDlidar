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
#include <sys/stat.h>  // 添加此头文件以支持 mkdir
#include <iomanip>  // 添加此头文件以支持 std::put_time
#include <sstream>  // 添加此头文件以支持 std::stringstream

#include "config.h"
#include "logger.h"
#include "lidar_types.h"
#include "ring_buffer.h"
#include "packet_parser.h"
#include "point_cloud.h"
#include <cstring>  // 添加此头文件以支持 memset, memcpy 和 strerror

// 全局变量
std::atomic<bool> g_running(true);
// 增加缓冲区大小到5000，可以容纳更多的UDP数据包
// 假设10Hz帧率，每帧30个包，需要300包/秒
// 5000包可缓存约16秒数据，足够系统处理延迟或瞬时高负载
RingBuffer<std::vector<uint8_t>> g_packet_buffer(5000);
std::map<uint32_t, PacketParser*> g_parsers;
PointCloudProcessor g_processor;
int g_socket_fd = -1; // 添加全局套接字文件描述符

// 添加缓冲区监控计数器
std::atomic<uint64_t> g_dropped_packets(0);
std::atomic<uint64_t> g_received_packets(0);

// 添加上一帧时间戳记录映射，用于计算帧率
std::map<uint32_t, std::chrono::steady_clock::time_point> g_last_frame_times;

// 信号处理函数
void signalHandler(int signum) {
    LD_INFO << "接收到信号 " << signum << "，正在退出...";
    g_running = false;
    
    // 关闭套接字以中断recvfrom()阻塞
    if (g_socket_fd >= 0) {
        close(g_socket_fd);
        g_socket_fd = -1;
    }
}

// 处理线程函数
void processThread() {
    LD_INFO << "点云处理线程启动";
    
    // 在处理线程开始时确保点云目录存在
    struct stat info;
    std::string pointCloudDir = "/usr/download/point_clouds";
    if (stat(pointCloudDir.c_str(), &info) != 0) {
        // 目录不存在，创建目录
        if (mkdir(pointCloudDir.c_str(), 0777) != 0) {
            LD_ERROR << "创建点云目录失败: " << pointCloudDir << ", 错误: " << strerror(errno);
        } else {
            LD_INFO << "创建点云目录: " << pointCloudDir;
        }
    }
    chmod(pointCloudDir.c_str(), 0777); // 确保目录权限
    
    // 每个解析器的最后保存时间，避免频繁保存
    std::map<uint32_t, std::chrono::steady_clock::time_point> lastSaveTime;
    // 每个解析器的帧计数器
    std::map<uint32_t, int> frameCounters;
    
    std::vector<uint8_t> packet_data;
    
    // 添加缓冲区使用率监控
    auto last_status_time = std::chrono::steady_clock::now();
    
    while (g_running) {
        if (g_packet_buffer.pop(packet_data)) {
            // 提取发送者IP地址 (从第一个字节获取IP地址的最后一段)
            uint32_t ipaddr = 0;
            if (packet_data.size() >= 4) {
                ipaddr = packet_data[0]; // IP地址的最后一个字节
            }
            
            // 创建或获取对应的解析器
            if (g_parsers.find(ipaddr) == g_parsers.end()) {
                LD_INFO << "新检测到雷达，IP后缀: " << ipaddr;
                
                // 查找配置中是否有对应IP的雷达参数
                LidarParam param;
                bool found = false;
                for (const auto& p : LidarConfig::getLidarParams()) {
                    if (p.ipaddr == ipaddr) {
                        param = p;
                        found = true;
                        break;
                    }
                }
                
                if (!found) {
                    param.ipaddr = ipaddr;
                    param.name = "lidar_" + std::to_string(ipaddr);
                }
                
                g_parsers[ipaddr] = new PacketParser();
                g_parsers[ipaddr]->setLidarParam(param);
                
                LD_INFO << "初始化雷达参数: " << param.toString();
                
                // 初始化帧计数器和保存时间
                frameCounters[ipaddr] = 0;
                lastSaveTime[ipaddr] = std::chrono::steady_clock::now();
            }
            
            // 解析数据包
            PointCloud cloud;
            bool frame_completed = false;
            
            if (packet_data.size() > 4) {
                frame_completed = g_parsers[ipaddr]->parsePacket(
                    packet_data.data() + 4, 
                    packet_data.size() - 4, 
                    cloud);
            }
            
            // 只有当帧完成时才处理点云
            if (frame_completed && !cloud.empty()) {
                // 更新帧计数器
                frameCounters[ipaddr]++;
                
                // 测试帧率 - 计算当前帧与上一帧之间的时间间隔
                auto current_time = std::chrono::steady_clock::now();
                
                // 如果存在上一帧的时间戳，计算帧率
                if (g_last_frame_times.find(ipaddr) != g_last_frame_times.end()) {
                    auto time_since_last_frame = std::chrono::duration_cast<std::chrono::milliseconds>(
                        current_time - g_last_frame_times[ipaddr]).count();
                    
                    double frame_rate = 1000.0 / time_since_last_frame; // 帧率 (帧/秒)
                    LD_INFO << "雷达 " << ipaddr << " 帧间隔: " << time_since_last_frame 
                           << " ms, 帧率: " << frame_rate << " fps";
                }
                
                // 更新该IP的最后帧时间
                g_last_frame_times[ipaddr] = current_time;
                
                // 修改: 对每一个完整的帧直接保存文件，不再基于计数或时间间隔
                // 测试帧率t1 - 文件保存开始时间
                const auto t1 = std::chrono::steady_clock::now();
                
                // 使用当前时间戳生成文件名
                std::time_t current_time_t = std::time(nullptr);
                std::tm* now_tm = std::localtime(&current_time_t);
                std::stringstream ss;
                ss << pointCloudDir << "/lidar_" << ipaddr << "_" 
                   << std::put_time(now_tm, "%Y%m%d_%H%M%S") << "_" 
                   << frameCounters[ipaddr] << ".ply";
                std::string filename = ss.str();
                
                // 检查点云是否有足够的点
                if (cloud.points.size() > 10) { // 降低点数阈值，便于调试
                    bool save_success = g_processor.saveToPLY(cloud, filename);
                    
                    // 测试帧率t2 - 文件保存结束时间
                    const auto t2 = std::chrono::steady_clock::now();
                    const auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
                    
                    if (save_success) {
                        LD_INFO << "点云数据成功保存到: " << filename 
                               << ", 点数: " << cloud.points.size()
                               << ", 保存耗时: " << time_cost << " ms";
                        
                        // 更新最后保存时间 (仍然保留这个记录，可能用于其他目的)
                        lastSaveTime[ipaddr] = current_time;
                    } else {
                        LD_ERROR << "保存点云数据失败: " << filename
                                << ", 尝试保存耗时: " << time_cost << " ms";
                    }
                } else {
                    LD_WARN << "点云数据点数不足，跳过保存: " << cloud.points.size() << " 点";
                }
                
                // 处理点云数据
                g_processor.processCloud(cloud);
            } else if (frame_completed) {
                // 帧已完成但点云为空
                LD_WARN << "检测到空点云帧，帧ID: " << cloud.frame_id;
            }
            
            // 添加缓冲区使用率监控
            auto current_time = std::chrono::steady_clock::now();
            auto time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                current_time - last_status_time).count();
            
            // 每30秒输出一次缓冲区使用情况
            if (time_elapsed >= 30) {
                size_t buffer_size = g_packet_buffer.size();
                size_t buffer_capacity = g_packet_buffer.capacity();
                float usage_percent = (buffer_size * 100.0f) / buffer_capacity;
                
                LD_INFO << "缓冲区使用情况: " << buffer_size << "/" << buffer_capacity 
                       << " (" << usage_percent << "%), 丢弃包数: " 
                       << g_dropped_packets.load() << "/" << g_received_packets.load();
                
                last_status_time = current_time;
            }
        }
    }
    
    LD_INFO << "点云处理线程退出";
}

// 点云回调函数，只保存完整的点云帧
void cloudCallback(const PointCloud& cloud) {
    static int counter = 0;
    counter++;
    
    // 每10帧输出一次信息
    if (counter % 10 == 0) {
        LD_INFO << "已处理点云帧: " << counter << ", 点数: " << cloud.points.size();
    }
    
    // 原有代码可能仍需保留，但要确保不是重复保存
    if (g_processor.isNewFrame() && cloud.points.size() > 100) {
        LD_INFO << "保存完整点云帧，帧ID: " << cloud.frame_id << ", 点数: " << cloud.points.size();
        g_processor.saveWithTimestamp(cloud, "./point_clouds");
    }
}

int main(int argc, char** argv) {
    LD_INFO << "RK3576 激光雷达点云处理工具 v" << GlobalConfig::Version;
    
    // 创建输出目录
    mkdir("./point_clouds", 0755);
    
    // 设置信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 解析命令行参数 (端口等)
    int port = LidarConfig::listenPort;
    if (argc > 1) {
        port = atoi(argv[1]);
    }
    LD_INFO << "监听端口: " << port;
    
    // 设置点云回调
    g_processor.setCallback(cloudCallback);
    
    // 启动处理线程
    std::thread proc_thread(processThread);
    
    // 创建UDP套接字
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        LD_FATAL << "socket() 失败: " << strerror(errno);
        return 1;
    }
    
    // 保存到全局变量，以便信号处理函数使用
    g_socket_fd = fd;
    
    // 设置套接字选项
    int reuse = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        LD_ERROR << "setsockopt(SO_REUSEADDR) 失败: " << strerror(errno);
    }
    
    // 设置套接字超时，防止recvfrom无限阻塞
    struct timeval tv;
    tv.tv_sec = 1;  // 1秒超时
    tv.tv_usec = 0;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        LD_ERROR << "setsockopt(SO_RCVTIMEO) 失败: " << strerror(errno);
    }
    
    // 绑定套接字
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        LD_FATAL << "bind() 失败: " << strerror(errno);
        close(fd);
        return 1;
    }
    
    // 如果配置了组播地址，则加入组播
    const char* multiaddr = LidarConfig::multicastAddr;
    if (multiaddr && multiaddr[0] != '\0') {
        struct ip_mreqn mreqn;
        inet_pton(AF_INET, multiaddr, &mreqn.imr_multiaddr.s_addr);
        inet_pton(AF_INET, "0.0.0.0", &mreqn.imr_address.s_addr);
        mreqn.imr_ifindex = if_nametoindex(LidarConfig::interfaceName);
        
        if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreqn, sizeof(mreqn)) < 0) {
            LD_ERROR << "加入组播组失败: " << strerror(errno);
        } else {
            LD_INFO << "已加入组播组: " << multiaddr;
        }
    }
    
    // 接收数据包
    uint8_t buffer[GlobalConfig::BufferSize];
    struct sockaddr_in client;
    socklen_t client_len = sizeof(client);
    
    LD_INFO << "开始接收数据...";
    
    while (g_running) {
        int recvlen = recvfrom(fd, buffer, sizeof(buffer), 0, 
                              (struct sockaddr*)&client, &client_len);
                              
        if (recvlen < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                // 被信号中断或超时，检查g_running标志
                continue;
            }
            if (!g_running) {
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
        memcpy(packet_data.data() + 4, buffer, recvlen);
        
        // 将数据包放入缓冲区
        if (!g_packet_buffer.push(packet_data)) {
            g_dropped_packets++; // 计数丢弃的包
            LD_WARN << "缓冲区已满，丢弃数据包";
        }
    }
    
    // 清理资源
    if (g_socket_fd >= 0) {
        close(g_socket_fd);
        g_socket_fd = -1;
    }
    
    // 确保缓冲区不再阻塞处理线程
    g_packet_buffer.setExit(true);
    
    // 等待处理线程结束
    if (proc_thread.joinable()) {
        LD_INFO << "等待点云处理线程退出...";
        proc_thread.join();
    }
    
    // 清理解析器
    for (auto& pair : g_parsers) {
        delete pair.second;
    }
    g_parsers.clear();
    
    // 在程序退出时输出包处理统计信息
    LD_INFO << "程序运行期间接收了 " << g_received_packets.load() 
           << " 个数据包，丢弃了 " << g_dropped_packets.load() << " 个数据包";
           
    LD_INFO << "程序正常退出";
    return 0;
}