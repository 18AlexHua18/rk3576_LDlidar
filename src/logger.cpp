#include "../include/logger.h"

// 定义并初始化静态成员变量
LogLevel Logger::level = INFO;  // 默认日志级别设为 INFO
LogOutputType Logger::output_type = SYSLOG_OUTPUT;  // 默认输出到系统日志

Logger::~Logger() {
    // 只有当当前消息级别大于等于设置的级别时才输出
    if (level_ < Logger::level) {
        return;
    }
    
    std::stringstream ss;
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    
    ss << "[" << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S") << "] "
       << "[" << getLevelString(level_) << "] "
       << stream_.str();
       
    std::string log_message = ss.str();
    
    // 根据输出类型选择输出目标
    if (Logger::output_type == CONSOLE_OUTPUT || Logger::output_type == BOTH_OUTPUT) {
        std::cout << log_message << std::endl;
    }
    
    // 如果是严重错误或设置为输出到系统日志，则输出到系统日志
    if ((level_ >= ERROR || Logger::output_type == SYSLOG_OUTPUT || Logger::output_type == BOTH_OUTPUT)) {
        // 转换日志级别
        int syslog_level;
        switch (level_) {
            case TRACE: syslog_level = LOG_DEBUG; break;
            case DEBUG: syslog_level = LOG_DEBUG; break;
            case INFO:  syslog_level = LOG_INFO; break;
            case WARN:  syslog_level = LOG_WARNING; break;
            case ERROR: syslog_level = LOG_ERR; break;
            case FATAL: syslog_level = LOG_CRIT; break;
            default:    syslog_level = LOG_NOTICE; break;
        }
        
        // 发送到 syslog
        syslog(syslog_level, "%s", log_message.c_str());
    }
}

std::string Logger::getLevelString(LogLevel level) {
    switch (level) {
        case TRACE: return "TRACE";
        case DEBUG: return "DEBUG";
        case INFO:  return "INFO ";
        case WARN:  return "WARN ";
        case ERROR: return "ERROR";
        case FATAL: return "FATAL";
        default:    return "UNKN ";
    }
}
