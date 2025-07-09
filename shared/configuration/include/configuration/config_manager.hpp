#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include <thread>
#include "interfaces/i_logger.hpp"

namespace radar {
namespace configuration {

// Forward declarations - applications will need to include specific headers
// when they use the specific config methods

struct ConfigManagerSettings {
    std::string config_file_path;           // Main configuration file path
    bool auto_reload{true};                 // Enable automatic configuration reloading
    double reload_check_interval_s{5.0};   // Configuration file check interval
    bool validate_on_load{true};            // Validate configuration on load
    bool backup_on_change{true};            // Backup configuration before changes
    std::string backup_directory{"config/backups"};
    size_t max_backups{10};                 // Maximum number of backup files
};

class ConfigManager {
public:
    ConfigManager();
    explicit ConfigManager(const ConfigManagerSettings& settings);
    ~ConfigManager();
    
    // Configuration loading and management
    bool loadConfiguration(const std::string& config_file);
    bool reloadConfiguration();
    bool saveConfiguration(const std::string& config_file = "");
    bool hasConfigurationChanged() const;
    
    // Configuration retrieval methods
    interfaces::LogConfig getLoggingConfig() const;
    
    // Configuration update methods
    bool updateLoggingConfig(const interfaces::LogConfig& config);
    
    // Generic parameter access
    template<typename T>
    T getParameter(const std::string& section, const std::string& key, const T& default_value = T{}) const;
    
    template<typename T>
    bool setParameter(const std::string& section, const std::string& key, const T& value);
    
    // Configuration validation
    bool validateConfiguration() const;
    std::vector<std::string> getValidationErrors() const;
    
    // File monitoring
    void startFileMonitoring();
    void stopFileMonitoring();
    bool isMonitoring() const;
    
    // Configuration metadata
    std::string getConfigFilePath() const;
    std::chrono::system_clock::time_point getLastModifiedTime() const;
    std::chrono::system_clock::time_point getLastLoadTime() const;
    size_t getConfigurationHash() const;
    
    // Backup management
    bool createBackup();
    std::vector<std::string> getAvailableBackups() const;
    bool restoreFromBackup(const std::string& backup_file);
    void cleanupOldBackups();
    
private:
    // Internal implementation
    struct Impl;
    std::unique_ptr<Impl> pImpl_;
    
    // Settings
    ConfigManagerSettings settings_;
    std::mutex settings_mutex_;
    
    // Configuration data
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> config_data_;
    mutable std::mutex config_mutex_;
    
    // File monitoring
    std::atomic<bool> monitoring_{false};
    std::thread monitor_thread_;
    std::atomic<bool> config_changed_{false};
    
    // File metadata
    std::string current_config_file_;
    std::chrono::system_clock::time_point last_modified_time_;
    std::chrono::system_clock::time_point last_load_time_;
    size_t configuration_hash_{0};
    
    // Validation
    std::vector<std::string> validation_errors_;
    mutable std::mutex validation_mutex_;
    
    // Internal methods
    bool loadFromFile(const std::string& file_path);
    bool saveToFile(const std::string& file_path);
    void monitoringWorkerThread();
    bool checkFileModified() const;
    
    // Parsing methods
    bool parseYamlFile(const std::string& file_path);
    bool parseJsonFile(const std::string& file_path);
    bool parseIniFile(const std::string& file_path);
    
    // Configuration building methods
    interfaces::LogConfig buildLoggingConfig() const;
    
    // Validation methods
    bool validateLoggingConfig(const interfaces::LogConfig& config) const;
    
    // Utility methods
    std::string getConfigString(const std::string& section, const std::string& key, const std::string& default_value = "") const;
    int getConfigInt(const std::string& section, const std::string& key, int default_value = 0) const;
    double getConfigDouble(const std::string& section, const std::string& key, double default_value = 0.0) const;
    bool getConfigBool(const std::string& section, const std::string& key, bool default_value = false) const;
    
    void setConfigString(const std::string& section, const std::string& key, const std::string& value);
    void setConfigInt(const std::string& section, const std::string& key, int value);
    void setConfigDouble(const std::string& section, const std::string& key, double value);
    void setConfigBool(const std::string& section, const std::string& key, bool value);
    
    size_t calculateConfigurationHash() const;
    std::string generateBackupFileName() const;
    std::string getFileExtension(const std::string& file_path) const;
    bool fileExists(const std::string& file_path) const;
    bool createDirectoryIfNotExists(const std::string& directory) const;
};

// Template method implementations
template<typename T>
T ConfigManager::getParameter(const std::string& section, const std::string& key, const T& default_value) const {
    static_assert(std::is_same_v<T, std::string> || 
                  std::is_same_v<T, int> || 
                  std::is_same_v<T, double> || 
                  std::is_same_v<T, bool>, 
                  "Unsupported parameter type");
    
    if constexpr (std::is_same_v<T, std::string>) {
        return getConfigString(section, key, default_value);
    } else if constexpr (std::is_same_v<T, int>) {
        return getConfigInt(section, key, default_value);
    } else if constexpr (std::is_same_v<T, double>) {
        return getConfigDouble(section, key, default_value);
    } else if constexpr (std::is_same_v<T, bool>) {
        return getConfigBool(section, key, default_value);
    }
}

template<typename T>
bool ConfigManager::setParameter(const std::string& section, const std::string& key, const T& value) {
    static_assert(std::is_same_v<T, std::string> || 
                  std::is_same_v<T, int> || 
                  std::is_same_v<T, double> || 
                  std::is_same_v<T, bool>, 
                  "Unsupported parameter type");
    
    try {
        if constexpr (std::is_same_v<T, std::string>) {
            setConfigString(section, key, value);
        } else if constexpr (std::is_same_v<T, int>) {
            setConfigInt(section, key, value);
        } else if constexpr (std::is_same_v<T, double>) {
            setConfigDouble(section, key, value);
        } else if constexpr (std::is_same_v<T, bool>) {
            setConfigBool(section, key, value);
        }
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace configuration
} // namespace radar