#pragma once

#include <memory>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>
#include "common/types.hpp"
#include "interfaces/i_logger.hpp"

namespace radar {
namespace rsp {

struct DetectionProcessorConfig {
    double min_snr{3.0};                    // Minimum signal-to-noise ratio
    double max_range{50000.0};             // Maximum detection range (meters)
    double min_range{100.0};               // Minimum detection range (meters)
    double doppler_threshold{0.5};         // Doppler processing threshold
    size_t max_detections{1000};           // Maximum detections per processing cycle
    
    // Quality control
    bool enable_quality_check{true};       // Enable detection quality checking
    double confidence_threshold{0.7};      // Minimum confidence for valid detection
    
    // Output settings
    size_t output_buffer_size{5000};       // Output buffer size
    double output_rate_hz{50.0};           // Output rate to data processor
    bool enable_compression{false};        // Enable data compression
    
    // Communication settings
    std::string data_processor_topic{"detections"};
    double heartbeat_interval_ms{1000.0};  // Heartbeat interval
};

class DetectionProcessor {
public:
    DetectionProcessor();
    ~DetectionProcessor();
    
    // Configuration
    bool configure(const DetectionProcessorConfig& config);
    DetectionProcessorConfig getConfiguration() const;
    
    // Lifecycle management
    bool start();
    void stop();
    bool isRunning() const;
    
    // Core processing functionality
    std::vector<common::Detection> processDetections(const std::vector<common::Detection>& raw_detections);
    
    // Quality control
    bool validateDetection(const common::Detection& detection) const;
    std::vector<common::Detection> filterDetections(const std::vector<common::Detection>& detections);
    
    // Output management
    void sendToDataProcessor();
    size_t getOutputBufferSize() const;
    void clearOutputBuffer();
    
    // Monitoring and diagnostics
    common::PerformanceMetrics getPerformanceMetrics() const;
    size_t getProcessedDetectionCount() const;
    double getDetectionRate() const;
    
private:
    // Internal implementation
    struct Impl;
    std::unique_ptr<Impl> pImpl_;
    
    // Configuration
    DetectionProcessorConfig config_;
    std::mutex config_mutex_;
    
    // State management
    std::atomic<bool> running_{false};
    std::atomic<bool> configured_{false};
    
    // Detection queues
    std::queue<common::Detection> input_queue_;
    std::queue<common::Detection> output_queue_;
    std::mutex input_queue_mutex_;
    std::mutex output_queue_mutex_;
    
    // Processing threads
    std::thread processing_thread_;
    std::thread output_thread_;
    
    // Statistics
    std::atomic<size_t> processed_count_{0};
    std::atomic<size_t> filtered_count_{0};
    std::atomic<size_t> sent_count_{0};
    
    // Metrics
    mutable std::mutex metrics_mutex_;
    common::PerformanceMetrics metrics_;
    
    // Internal methods
    void processingWorkerThread();
    void outputWorkerThread();
    void updateMetrics();
    bool validateConfiguration(const DetectionProcessorConfig& config) const;
    
    // Quality control methods
    bool checkSNR(const common::Detection& detection) const;
    bool checkRange(const common::Detection& detection) const;
    bool checkDoppler(const common::Detection& detection) const;
    double calculateConfidence(const common::Detection& detection) const;
    
    // Communication methods
    void initializeCommunication();
    void shutdownCommunication();
    bool sendDetectionBatch(const std::vector<common::Detection>& detections);
};

} // namespace rsp
} // namespace radar