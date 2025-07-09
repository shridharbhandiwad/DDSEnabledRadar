#pragma once

#include <memory>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include "common/types.hpp"
#include "interfaces/i_logger.hpp"

namespace radar {
namespace rsp {

struct SignalProcessorConfig {
    double sampling_rate{1000.0};          // Hz
    double threshold{0.1};                  // Detection threshold
    size_t window_size{1024};              // Processing window size
    size_t overlap{512};                   // Overlap between windows
    bool enable_cfar{true};                // Constant False Alarm Rate
    double cfar_threshold{3.0};            // CFAR threshold
    size_t max_detections_per_scan{500};   // Maximum detections per scan
    
    // Performance settings
    size_t thread_count{4};                // Number of processing threads
    size_t buffer_size{10000};             // Internal buffer size
    double processing_timeout_ms{50.0};    // Processing timeout
};

class SignalProcessor {
public:
    SignalProcessor();
    ~SignalProcessor();
    
    // Configuration
    bool configure(const SignalProcessorConfig& config);
    SignalProcessorConfig getConfiguration() const;
    
    // Lifecycle management
    bool start();
    void stop();
    bool isRunning() const;
    
    // Core processing functionality
    std::vector<common::Detection> processSignals();
    
    // Status and monitoring
    void sendStatusUpdate();
    common::PerformanceMetrics getPerformanceMetrics() const;
    
    // Advanced processing
    bool performCFAR(const std::vector<double>& signal_data);
    std::vector<common::Detection> extractDetections(const std::vector<double>& processed_data);
    
private:
    // Internal implementation
    struct Impl;
    std::unique_ptr<Impl> pImpl_;
    
    // Configuration
    SignalProcessorConfig config_;
    mutable std::mutex config_mutex_;
    
    // State management
    std::atomic<bool> running_{false};
    std::atomic<bool> configured_{false};
    
    // Processing threads
    std::vector<std::thread> processing_threads_;
    
    // Metrics
    mutable std::mutex metrics_mutex_;
    common::PerformanceMetrics metrics_;
    
    // Internal methods
    void processingWorkerThread();
    void updateMetrics(const common::PerformanceMetrics& new_metrics);
    bool validateConfiguration(const SignalProcessorConfig& config) const;
    void initializeProcessingPipeline();
    void shutdownProcessingPipeline();
};

} // namespace rsp
} // namespace radar