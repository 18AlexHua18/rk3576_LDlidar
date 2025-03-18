# RK3576 雷达点云处理

## 项目说明

该项目用于接收和处理来自亮道雷达的点云数据。项目通过UDP接收数据包，解析帧数据并构建点云。

## 编译选项

构建说明
使用以下命令构建Debug版本（用于gdbserver调试）：
```sh
mkdir -p build_debug && cd build_debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```
使用以下命令构建Release版本（用于生产环境）：
```sh
mkdir -p build_release && cd build_release
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```
Debug版本包含完整的调试信息，可以与gdbserver一起使用，而Release版本进行了代码优化以提高性能。

### 点云过滤功能

项目提供了对点云数据进行过滤的功能，默认情况下该功能是关闭的，以保证捕获所有可能的点。

#### 启用/禁用点云过滤

通过CMake选项控制点云过滤功能：

- 禁用点云过滤（默认）：
  ```bash
  cmake ..
  ```
  或
  ```bash
  cmake -DENABLE_POINT_FILTERING=OFF ..
  ```

- 启用点云过滤：
  ```bash
  cmake -DENABLE_POINT_FILTERING=ON ..
  ```

#### 过滤原理

当启用点云过滤时，会根据以下规则过滤点：

1. 如果`algorithmParam.EnableEchoChose = 1`，将根据点的回波标签`echoChose`进行过滤
2. 如果`algorithmParam.EchoNumber`为1-3，则只保留对应索引的回波
3. 如果`algorithmParam.EchoNumber = 4`，则保留前3个回波

#### 调试建议

如果点云数据看起来不完整或有丢失的点：
- 确保点云过滤功能已禁用（默认状态）
- 检查帧判定逻辑是否合理，避免帧数据未完全接收就结束处理
- 通过日志信息分析接收到的包数量和子帧覆盖率

## 其他配置

请参考代码中的其他配置选项，如端口设置、保存路径等。
雷达的IP地址为`192.168.5.10`，打开测试demo前可以先ping通雷达保证通讯正常
必须将设备设置为一个网段，否则需要修改子网掩码用于查看到雷达网段的广播
可以使用的指令推荐：`ifconfig eth0 192.168.5.36 netmask 255.255.255.0`，*仅用于临时测试，若需要长时间开机自动运行需要让ETH端口设置固定IP*
