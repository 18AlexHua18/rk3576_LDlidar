#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <iomanip>
#include <syslog.h> // 添加 syslog 头文件

enum LogLevel {
    TRACE,
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

// 日志输出目标类型
enum LogOutputType {
    CONSOLE_OUTPUT,  // 控制台输出
    SYSLOG_OUTPUT,   // 系统日志输出
    BOTH_OUTPUT      // 同时输出到控制台和系统日志
};

// 日志类
class Logger {
public:
    Logger(LogLevel level) : level_(level) {}
    
    ~Logger() {
        std::time_t now = std::time(nullptr);
        std::tm* now_tm = std::localtime(&now);
        
        // 构建日志消息
        std::string log_message = stream_.str();
        std::stringstream time_stream;
        time_stream << "[" << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S") << "] [" 
                   << getLevelString(level_) << "] " << log_message;
        std::string full_message = time_stream.str();
        
        // 根据输出类型发送日志
        if (output_type == CONSOLE_OUTPUT || output_type == BOTH_OUTPUT) {
            std::cout << full_message << std::endl;
        }
        
        if (output_type == SYSLOG_OUTPUT || output_type == BOTH_OUTPUT) {
            // 将日志级别转换为 syslog 级别
            int syslog_level = LOG_INFO;
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
    
    template <typename T>
    Logger& operator<<(const T& msg) {
        stream_ << msg;
        return *this;
    }
    
    // 静态成员及方法
    static LogLevel level;
    static LogOutputType output_type;
    
    // 设置日志级别
    static void setLogLevel(LogLevel level) { Logger::level = level; }
    static LogLevel getLogLevel() { return Logger::level; }
    
    // 设置日志输出类型
    static void setLogOutput(LogOutputType type) { Logger::output_type = type; }
    static LogOutputType getLogOutput() { return Logger::output_type; }
    
    // 初始化 syslog
    static void initSyslog(const char* ident = "rk3576_LDlidar", int option = LOG_PID, int facility = LOG_USER) {
        openlog(ident, option, facility);
    }
    
    // 关闭 syslog
    static void closeSyslog() {
        closelog();
    }
    
private:
    LogLevel level_;
    std::ostringstream stream_;
    
    std::string getLevelString(LogLevel level) {
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
};

#define LD_TRACE Logger(TRACE)
#define LD_DEBUG Logger(DEBUG)
#define LD_INFO  Logger(INFO)
#define LD_WARN  Logger(WARN)
#define LD_ERROR Logger(ERROR)
#define LD_FATAL Logger(FATAL)