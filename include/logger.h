#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <syslog.h>

enum LogLevel {
    TRACE,
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

enum LogOutputType {
    CONSOLE_OUTPUT,
    SYSLOG_OUTPUT,
    BOTH_OUTPUT
};

// 日志类
class Logger {
public:
    Logger(LogLevel level) : level_(level) {}
    
    ~Logger();
    
    template<typename T>
    Logger& operator<<(const T& value) {
        stream_ << value;
        return *this;
    }
    
    // 静态成员：全局日志级别和输出类型
    static LogLevel level;
    static LogOutputType output_type;
    
    // 设置全局日志级别
    static void setLogLevel(LogLevel level) {
        Logger::level = level;
    }
    
    // 设置日志输出类型
    static void setOutputType(LogOutputType type) {
        Logger::output_type = type;
    }
    
private:
    LogLevel level_;
    std::ostringstream stream_;
    
    std::string getLevelString(LogLevel level);
};

#define LD_TRACE Logger(TRACE)
#define LD_DEBUG Logger(DEBUG)
#define LD_INFO  Logger(INFO)
#define LD_WARN  Logger(WARN)
#define LD_ERROR Logger(ERROR)
#define LD_FATAL Logger(FATAL)