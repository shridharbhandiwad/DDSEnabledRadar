#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <queue>
#include <fstream>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include "interfaces/i_logger.hpp"

namespace radar {
namespace logging {

struct LogMessage {
    interfaces::LogLevel level;
    std::string component;
    std::string message;
    std::chrono::system_clock::time_point timestamp;
    std::thread::id thread_id;
};

class Logger : public interfaces::ILogger {
public:
    Logger();
    explicit Logger(const std::string& name);
    ~Logger() override;
    
    // ILogger interface implementation
    bool configure(const interfaces::LogConfig& config) override;
    interfaces::LogConfig getConfiguration() const override;
    
    // Core logging methods
    void trace(const std::string& component, const std::string& message) override;
    void debug(const std::string& component, const std::string& message) override;
    void info(const std::string& component, const std::string& message) override;
    void warn(const std::string& component, const std::string& message) override;
    void error(const std::string& component, const std::string& message) override;
    void critical(const std::string& component, const std::string& message) override;
    
    // Formatted logging methods
    void trace(const std::string& component, const char* format, ...) override;
    void debug(const std::string& component, const char* format, ...) override;
    void info(const std::string& component, const char* format, ...) override;
    void warn(const std::string& component, const char* format, ...) override;
    void error(const std::string& component, const char* format, ...) override;
    void critical(const std::string& component, const char* format, ...) override;
    
    // Log level control
    void setLevel(interfaces::LogLevel level) override;
    interfaces::LogLevel getLevel() const override;
    bool shouldLog(interfaces::LogLevel level) const override;
    
    // Log management
    void flush() override;
    void rotate() override;
    size_t getQueueSize() const override;
    
    // Logger name
    void setName(const std::string& name) override;
    std::string getName() const override;
    
    // Static factory methods
    static std::shared_ptr<Logger> create();
    static std::shared_ptr<Logger> create(const std::string& name);
    static std::shared_ptr<Logger> create(const interfaces::LogConfig& config);
    
private:
    // Configuration
    interfaces::LogConfig config_;
    mutable std::mutex config_mutex_;
    
    // State management
    std::atomic<bool> initialized_{false};
    std::atomic<bool> shutdown_{false};
    
    // Asynchronous logging
    std::queue<LogMessage> message_queue_;
    mutable std::mutex queue_mutex_;  // Made mutable for const methods
    std::condition_variable queue_condition_;
    std::thread worker_thread_;
    
    // File output
    std::ofstream log_file_;
    std::mutex file_mutex_;
    size_t current_file_size_{0};
    size_t current_file_index_{0};
    
    // Internal methods
    void log(interfaces::LogLevel level, const std::string& component, const std::string& message);
    void logFormatted(interfaces::LogLevel level, const std::string& component, const char* format, va_list args);
    void workerThread();
    void processMessage(const LogMessage& message);
    
    // Output methods
    void writeToConsole(const LogMessage& message);
    void writeToFile(const LogMessage& message);
    std::string formatMessage(const LogMessage& message) const;
    
    // File management
    bool openLogFile();
    void closeLogFile();
    void rotateLogFile();
    bool needsRotation() const;
    std::string generateLogFileName(size_t index = 0) const;
    
    // Utility methods
    std::string levelToString(interfaces::LogLevel level) const;
    std::string formatTimestamp(const std::chrono::system_clock::time_point& timestamp) const;
    bool validateConfig(const interfaces::LogConfig& config) const;
    void initializeLogging();
    void shutdownLogging();
    
    // Cleanup methods
    void cleanupOldLogFiles();
    std::vector<std::string> getExistingLogFiles() const;
};

// Convenience macros for logging
#define LOG_TRACE(logger, component, message) \
    do { if (logger && logger->shouldLog(radar::interfaces::LogLevel::TRACE)) \
         logger->trace(component, message); } while(0)

#define LOG_DEBUG(logger, component, message) \
    do { if (logger && logger->shouldLog(radar::interfaces::LogLevel::DEBUG)) \
         logger->debug(component, message); } while(0)

#define LOG_INFO(logger, component, message) \
    do { if (logger && logger->shouldLog(radar::interfaces::LogLevel::INFO)) \
         logger->info(component, message); } while(0)

#define LOG_WARN(logger, component, message) \
    do { if (logger && logger->shouldLog(radar::interfaces::LogLevel::WARN)) \
         logger->warn(component, message); } while(0)

#define LOG_ERROR(logger, component, message) \
    do { if (logger && logger->shouldLog(radar::interfaces::LogLevel::ERROR)) \
         logger->error(component, message); } while(0)

#define LOG_CRITICAL(logger, component, message) \
    do { if (logger && logger->shouldLog(radar::interfaces::LogLevel::CRITICAL)) \
         logger->critical(component, message); } while(0)

} // namespace logging
} // namespace radar