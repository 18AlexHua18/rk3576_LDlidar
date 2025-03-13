#include "../include/logger.h"

// 定义并初始化静态成员变量
LogLevel Logger::level = INFO;  // 默认日志级别设为 INFO
LogOutputType Logger::output_type = SYSLOG_OUTPUT;  // 默认输出到系统日志
