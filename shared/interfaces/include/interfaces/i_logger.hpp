#pragma once

#include <string>
#include <memory>
#include <vector>

namespace radar {
namespace interfaces {

enum class LogLevel {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
    CRITICAL = 5,
    OFF = 6
};

struct LogConfig {
    LogLevel level{LogLevel::INFO};         // Minimum log level
    std::string pattern{"%Y-%m-%d %H:%M:%S [%l] %n: %v"}; // Log pattern
    bool console_output{true};              // Enable console output
    bool file_output{true};                 // Enable file output
    std::string log_file{"radar_system.log"}; // Log file path
    size_t max_file_size{10 * 1024 * 1024}; // Maximum file size (10MB)
    size_t max_files{5};                    // Maximum number of log files
    bool async_logging{true};               // Enable asynchronous logging
    size_t async_queue_size{8192};          // Async queue size
    bool flush_on_error{true};              // Flush logs on error
    std::string logger_name{"radar"};       // Logger name
};

class ILogger {
public:
    virtual ~ILogger() = default;
    
    // Configuration
    virtual bool configure(const LogConfig& config) = 0;
    virtual LogConfig getConfiguration() const = 0;
    
    // Core logging methods
    virtual void trace(const std::string& component, const std::string& message) = 0;
    virtual void debug(const std::string& component, const std::string& message) = 0;
    virtual void info(const std::string& component, const std::string& message) = 0;
    virtual void warn(const std::string& component, const std::string& message) = 0;
    virtual void error(const std::string& component, const std::string& message) = 0;
    virtual void critical(const std::string& component, const std::string& message) = 0;
    
    // Formatted logging methods
    virtual void trace(const std::string& component, const char* format, ...) = 0;
    virtual void debug(const std::string& component, const char* format, ...) = 0;
    virtual void info(const std::string& component, const char* format, ...) = 0;
    virtual void warn(const std::string& component, const char* format, ...) = 0;
    virtual void error(const std::string& component, const char* format, ...) = 0;
    virtual void critical(const std::string& component, const char* format, ...) = 0;
    
    // Log level control
    virtual void setLevel(LogLevel level) = 0;
    virtual LogLevel getLevel() const = 0;
    virtual bool shouldLog(LogLevel level) const = 0;
    
    // Log management
    virtual void flush() = 0;
    virtual void rotate() = 0;
    virtual size_t getQueueSize() const = 0;
    
    // Logger name
    virtual void setName(const std::string& name) = 0;
    virtual std::string getName() const = 0;
};

} // namespace interfaces
} // namespace radar