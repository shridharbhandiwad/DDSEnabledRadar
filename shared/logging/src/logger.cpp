#include "logging/logger.hpp"
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <thread>

namespace radar {
namespace logging {

Logger::Logger() : Logger("DefaultLogger") {
}

Logger::Logger(const std::string& name) {
    config_.logger_name = name;
    config_.level = interfaces::LogLevel::INFO;
    config_.console_output = true;
    config_.file_output = false;
    config_.async_logging = true;
    config_.max_file_size = 10 * 1024 * 1024; // 10MB
    config_.max_files = 5;
    
    initializeLogging();
}

Logger::~Logger() {
    shutdownLogging();
}

bool Logger::configure(const interfaces::LogConfig& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    if (!validateConfig(config)) {
        return false;
    }
    
    // Stop current logging
    shutdownLogging();
    
    // Update configuration
    config_ = config;
    
    // Restart with new configuration
    initializeLogging();
    
    return true;
}

interfaces::LogConfig Logger::getConfiguration() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

void Logger::trace(const std::string& component, const std::string& message) {
    log(interfaces::LogLevel::TRACE, component, message);
}

void Logger::debug(const std::string& component, const std::string& message) {
    log(interfaces::LogLevel::DEBUG, component, message);
}

void Logger::info(const std::string& component, const std::string& message) {
    log(interfaces::LogLevel::INFO, component, message);
}

void Logger::warn(const std::string& component, const std::string& message) {
    log(interfaces::LogLevel::WARN, component, message);
}

void Logger::error(const std::string& component, const std::string& message) {
    log(interfaces::LogLevel::ERROR, component, message);
}

void Logger::critical(const std::string& component, const std::string& message) {
    log(interfaces::LogLevel::CRITICAL, component, message);
}

void Logger::trace(const std::string& component, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logFormatted(interfaces::LogLevel::TRACE, component, format, args);
    va_end(args);
}

void Logger::debug(const std::string& component, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logFormatted(interfaces::LogLevel::DEBUG, component, format, args);
    va_end(args);
}

void Logger::info(const std::string& component, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logFormatted(interfaces::LogLevel::INFO, component, format, args);
    va_end(args);
}

void Logger::warn(const std::string& component, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logFormatted(interfaces::LogLevel::WARN, component, format, args);
    va_end(args);
}

void Logger::error(const std::string& component, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logFormatted(interfaces::LogLevel::ERROR, component, format, args);
    va_end(args);
}

void Logger::critical(const std::string& component, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logFormatted(interfaces::LogLevel::CRITICAL, component, format, args);
    va_end(args);
}

void Logger::setLevel(interfaces::LogLevel level) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_.level = level;
}

interfaces::LogLevel Logger::getLevel() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_.level;
}

bool Logger::shouldLog(interfaces::LogLevel level) const {
    return static_cast<int>(level) >= static_cast<int>(getLevel());
}

void Logger::flush() {
    if (config_.file_output) {
        std::lock_guard<std::mutex> lock(file_mutex_);
        if (log_file_.is_open()) {
            log_file_.flush();
        }
    }
}

void Logger::rotate() {
    if (config_.file_output && needsRotation()) {
        rotateLogFile();
    }
}

size_t Logger::getQueueSize() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return message_queue_.size();
}

void Logger::setName(const std::string& name) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_.logger_name = name;
}

std::string Logger::getName() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_.logger_name;
}

std::shared_ptr<Logger> Logger::create() {
    return std::make_shared<Logger>();
}

std::shared_ptr<Logger> Logger::create(const std::string& name) {
    return std::make_shared<Logger>(name);
}

std::shared_ptr<Logger> Logger::create(const interfaces::LogConfig& config) {
    auto logger = std::make_shared<Logger>();
    logger->configure(config);
    return logger;
}

// Private methods implementation
void Logger::log(interfaces::LogLevel level, const std::string& component, const std::string& message) {
    if (!shouldLog(level) || shutdown_.load()) {
        return;
    }
    
    LogMessage log_msg;
    log_msg.level = level;
    log_msg.component = component;
    log_msg.message = message;
    log_msg.timestamp = std::chrono::system_clock::now();
    log_msg.thread_id = std::this_thread::get_id();
    
    if (config_.async_logging) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            message_queue_.push(log_msg);
        }
        queue_condition_.notify_one();
    } else {
        processMessage(log_msg);
    }
}

void Logger::logFormatted(interfaces::LogLevel level, const std::string& component, const char* format, va_list args) {
    if (!shouldLog(level)) {
        return;
    }
    
    char buffer[4096];
    vsnprintf(buffer, sizeof(buffer), format, args);
    log(level, component, std::string(buffer));
}

void Logger::workerThread() {
    while (!shutdown_.load()) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_condition_.wait(lock, [this] { return !message_queue_.empty() || shutdown_.load(); });
        
        while (!message_queue_.empty()) {
            LogMessage message = message_queue_.front();
            message_queue_.pop();
            lock.unlock();
            
            processMessage(message);
            
            lock.lock();
        }
    }
}

void Logger::processMessage(const LogMessage& message) {
    if (config_.console_output) {
        writeToConsole(message);
    }
    
    if (config_.file_output) {
        writeToFile(message);
    }
}

void Logger::writeToConsole(const LogMessage& message) {
    std::string formatted = formatMessage(message);
    
    // Use different colors for different log levels
    if (message.level >= interfaces::LogLevel::ERROR) {
        std::cerr << formatted << std::endl;
    } else {
        std::cout << formatted << std::endl;
    }
}

void Logger::writeToFile(const LogMessage& message) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    
    if (!log_file_.is_open()) {
        if (!openLogFile()) {
            return;
        }
    }
    
    std::string formatted = formatMessage(message);
    log_file_ << formatted << std::endl;
    
    current_file_size_ += formatted.length() + 1; // +1 for newline
    
    if (needsRotation()) {
        rotateLogFile();
    }
}

std::string Logger::formatMessage(const LogMessage& message) const {
    std::ostringstream oss;
    
    // Format: [TIMESTAMP] [LEVEL] [COMPONENT] MESSAGE
    oss << "[" << formatTimestamp(message.timestamp) << "] "
        << "[" << levelToString(message.level) << "] "
        << "[" << message.component << "] "
        << message.message;
    
    return oss.str();
}

bool Logger::openLogFile() {
    if (config_.log_file.empty()) {
        return false;
    }
    
    // Create directory if it doesn't exist
    std::filesystem::path file_path(config_.log_file);
    std::filesystem::path dir_path = file_path.parent_path();
    
    if (!dir_path.empty() && !std::filesystem::exists(dir_path)) {
        std::filesystem::create_directories(dir_path);
    }
    
    log_file_.open(generateLogFileName(), std::ios::app);
    if (log_file_.is_open()) {
        // Get current file size
        log_file_.seekp(0, std::ios::end);
        current_file_size_ = log_file_.tellp();
        return true;
    }
    
    return false;
}

void Logger::closeLogFile() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
    current_file_size_ = 0;
}

void Logger::rotateLogFile() {
    closeLogFile();
    
    // Move current log files
    for (size_t i = config_.max_files - 1; i > 0; --i) {
        std::string old_name = generateLogFileName(i - 1);
        std::string new_name = generateLogFileName(i);
        
        if (std::filesystem::exists(old_name)) {
            std::filesystem::rename(old_name, new_name);
        }
    }
    
    // Remove oldest log file if it exists
    std::string oldest = generateLogFileName(config_.max_files - 1);
    if (std::filesystem::exists(oldest)) {
        std::filesystem::remove(oldest);
    }
    
    current_file_index_ = 0;
    openLogFile();
}

bool Logger::needsRotation() const {
    return config_.max_file_size > 0 && current_file_size_ >= config_.max_file_size;
}

std::string Logger::generateLogFileName(size_t index) const {
    if (index == 0) {
        return config_.log_file;
    } else {
        std::filesystem::path path(config_.log_file);
        std::string stem = path.stem().string();
        std::string extension = path.extension().string();
        std::string dir = path.parent_path().string();
        
        return dir + "/" + stem + "." + std::to_string(index) + extension;
    }
}

std::string Logger::levelToString(interfaces::LogLevel level) const {
    switch (level) {
        case interfaces::LogLevel::TRACE: return "TRACE";
        case interfaces::LogLevel::DEBUG: return "DEBUG";
        case interfaces::LogLevel::INFO: return "INFO";
        case interfaces::LogLevel::WARN: return "WARN";
        case interfaces::LogLevel::ERROR: return "ERROR";
        case interfaces::LogLevel::CRITICAL: return "CRITICAL";
        default: return "UNKNOWN";
    }
}

std::string Logger::formatTimestamp(const std::chrono::system_clock::time_point& timestamp) const {
    auto time_t = std::chrono::system_clock::to_time_t(timestamp);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        timestamp.time_since_epoch()) % 1000;
    
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    oss << "." << std::setfill('0') << std::setw(3) << ms.count();
    
    return oss.str();
}

bool Logger::validateConfig(const interfaces::LogConfig& config) const {
    // Basic validation
    if (config.logger_name.empty()) {
        return false;
    }
    
    if (config.file_output && config.log_file.empty()) {
        return false;
    }
    
    return true;
}

void Logger::initializeLogging() {
    if (initialized_.load()) {
        return;
    }
    
    shutdown_ = false;
    
    if (config_.file_output) {
        openLogFile();
    }
    
    if (config_.async_logging) {
        worker_thread_ = std::thread(&Logger::workerThread, this);
    }
    
    initialized_ = true;
}

void Logger::shutdownLogging() {
    if (!initialized_.load()) {
        return;
    }
    
    shutdown_ = true;
    
    if (config_.async_logging && worker_thread_.joinable()) {
        queue_condition_.notify_all();
        worker_thread_.join();
    }
    
    // Process any remaining messages
    while (!message_queue_.empty()) {
        processMessage(message_queue_.front());
        message_queue_.pop();
    }
    
    if (config_.file_output) {
        closeLogFile();
    }
    
    initialized_ = false;
}

void Logger::cleanupOldLogFiles() {
    // Simple implementation - no-op for now
}

std::vector<std::string> Logger::getExistingLogFiles() const {
    return std::vector<std::string>();
}

} // namespace logging
} // namespace radar