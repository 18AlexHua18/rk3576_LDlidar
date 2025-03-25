#!/bin/zsh

# debug bridge
export PATH=$PATH:/mnt/c/toolchain/adb_platform-tools_r33.0.1-windows/platform-tools/
alias adb='/mnt/c/toolchain/adb_platform-tools_r33.0.1-windows/platform-tools/adb.exe'

# 设备时间同步
adb shell "date $(date +%m%d%H%M%Y.%S)"

# setting IP
adb shell ifconfig eth0 192.168.5.36 netmask 255.255.255.0

# **杀死已有的 rk3576_LDlidar 进程**
echo "正在检查并终止已有的 rk3576_LDlidar 进程..."
adb shell "ps | grep rk3576_LDlidar | awk '{print \$2}' | grep -E '^[0-9]+$' | xargs -r adb shell kill -9"
echo "旧进程已终止"

# 询问用户选择文件夹
echo "请选择要发送的可执行文件:"
echo "1) 打开过滤算法版本"
echo "2) 关闭过滤算法版本"
read "?请输入1或2: " choice  # zsh 语法，"?" 使其在同一行显示

# 根据选择设置文件夹路径
if [ "$choice" = "1" ]; then
    LOCAL_DIR="/home/alex/Project/rk3576_LDlidar/cmake-build-debug-withalgrithm/bin/rk3576_LDlidar"
elif [ "$choice" = "2" ]; then
    LOCAL_DIR="/home/alex/Project/rk3576_LDlidar/cmake-build-debug-withoutFilter/bin/rk3576_LDlidar"
else
    echo "无效输入，退出"
    exit 1
fi

REMOTE_DIR="/usr/download/"  # 远端目标目录

# 1. 推送文件到远端
adb push "$LOCAL_DIR" "$REMOTE_DIR"

# 2. 赋予所有可执行文件权限
adb shell "chmod -R +x $REMOTE_DIR"

# 3. 运行程序并监听端口
adb shell "pkill -f gdbserver"
adb shell "gdbserver :12345 $REMOTE_DIR/rk3576_LDlidar &"

adb forward tcp:12345 tcp:12345

echo "操作完成"
