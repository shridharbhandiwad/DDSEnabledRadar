#include "detection_processor.hpp"
#include <algorithm>
#include <cmath>

namespace radar {
namespace rsp {

// Pimpl implementation
struct DetectionProcessor::Impl {
    // Add future implementation details here
};

DetectionProcessor::DetectionProcessor() : pImpl_(std::make_unique<Impl>()) {
    // Initialize with default configuration
    config_.min_snr = 3.0;
    config_.max_range = 50000.0;
    config_.min_range = 100.0;
    config_.doppler_threshold = 0.5;
    config_.max_detections = 1000;
    config_.enable_quality_check = true;
    config_.confidence_threshold = 0.7;
    config_.output_buffer_size = 5000;
    config_.output_rate_hz = 50.0;
    config_.enable_compression = false;
    config_.data_processor_topic = "detections";
    config_.heartbeat_interval_ms = 1000.0;
}

DetectionProcessor::~DetectionProcessor() {
    stop();
}

bool DetectionProcessor::configure(const DetectionProcessorConfig& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    if (!validateConfiguration(config)) {
        return false;
    }
    
    // Stop processing if running
    bool was_running = isRunning();
    if (was_running) {
        stop();
    }
    
    // Update configuration
    config_ = config;
    configured_ = true;
    
    // Restart if it was running
    if (was_running) {
        start();
    }
    
    return true;
}

DetectionProcessorConfig DetectionProcessor::getConfiguration() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

bool DetectionProcessor::start() {
    if (running_.load()) {
        return true;
    }
    
    if (!configured_.load()) {
        return false;
    }
    
    initializeCommunication();
    
    // Start worker threads
    processing_thread_ = std::thread(&DetectionProcessor::processingWorkerThread, this);
    output_thread_ = std::thread(&DetectionProcessor::outputWorkerThread, this);
    
    running_ = true;
    return true;
}

void DetectionProcessor::stop() {
    if (!running_.load()) {
        return;
    }
    
    running_ = false;
    
    // Wait for threads to finish
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    if (output_thread_.joinable()) {
        output_thread_.join();
    }
    
    shutdownCommunication();
}

bool DetectionProcessor::isRunning() const {
    return running_.load();
}

std::vector<common::Detection> DetectionProcessor::processDetections(const std::vector<common::Detection>& raw_detections) {
    if (!running_.load()) {
        return {};
    }
    
    std::vector<common::Detection> processed_detections;
    processed_detections.reserve(raw_detections.size());
    
    for (const auto& detection : raw_detections) {
        if (validateDetection(detection)) {
            // Create a copy and potentially modify it
            common::Detection processed = detection;
            
            // Update confidence based on our criteria
            processed.confidence = calculateConfidence(detection);
            
            // Add to processed list if it meets quality standards
            if (!config_.enable_quality_check || processed.confidence >= config_.confidence_threshold) {
                processed_detections.push_back(processed);
            }
        }
        
        if (processed_detections.size() >= config_.max_detections) {
            break;
        }
    }
    
    // Add to input queue for further processing
    {
        std::lock_guard<std::mutex> lock(input_queue_mutex_);
        for (const auto& detection : processed_detections) {
            if (input_queue_.size() < config_.output_buffer_size) {
                input_queue_.push(detection);
            }
        }
    }
    
    // Update statistics
    processed_count_ += processed_detections.size();
    filtered_count_ += (raw_detections.size() - processed_detections.size());
    
    updateMetrics();
    
    return processed_detections;
}

bool DetectionProcessor::validateDetection(const common::Detection& detection) const {
    if (!checkSNR(detection)) {
        return false;
    }
    
    if (!checkRange(detection)) {
        return false;
    }
    
    if (!checkDoppler(detection)) {
        return false;
    }
    
    return true;
}

std::vector<common::Detection> DetectionProcessor::filterDetections(const std::vector<common::Detection>& detections) {
    std::vector<common::Detection> filtered;
    filtered.reserve(detections.size());
    
    for (const auto& detection : detections) {
        if (validateDetection(detection)) {
            filtered.push_back(detection);
        }
    }
    
    return filtered;
}

void DetectionProcessor::sendToDataProcessor() {
    std::vector<common::Detection> detections_to_send;
    
    // Get detections from output queue
    {
        std::lock_guard<std::mutex> lock(output_queue_mutex_);
        while (!output_queue_.empty() && detections_to_send.size() < 100) { // Batch size limit
            detections_to_send.push_back(output_queue_.front());
            output_queue_.pop();
        }
    }
    
    if (!detections_to_send.empty()) {
        // In a real implementation, this would send via DDS/network
        if (sendDetectionBatch(detections_to_send)) {
            sent_count_ += detections_to_send.size();
        }
    }
}

size_t DetectionProcessor::getOutputBufferSize() const {
    std::lock_guard<std::mutex> lock(output_queue_mutex_);
    return output_queue_.size();
}

void DetectionProcessor::clearOutputBuffer() {
    std::lock_guard<std::mutex> lock(output_queue_mutex_);
    while (!output_queue_.empty()) {
        output_queue_.pop();
    }
}

common::PerformanceMetrics DetectionProcessor::getPerformanceMetrics() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return metrics_;
}

size_t DetectionProcessor::getProcessedDetectionCount() const {
    return processed_count_.load();
}

double DetectionProcessor::getDetectionRate() const {
    // Calculate detections per second
    auto now = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - metrics_.timestamp).count();
    
    if (elapsed > 0) {
        return static_cast<double>(processed_count_.load()) / elapsed;
    }
    
    return 0.0;
}

// Private methods implementation
void DetectionProcessor::processingWorkerThread() {
    while (running_.load()) {
        try {
            std::vector<common::Detection> detections_to_process;
            
            // Get detections from input queue
            {
                std::lock_guard<std::mutex> lock(input_queue_mutex_);
                while (!input_queue_.empty() && detections_to_process.size() < 50) {
                    detections_to_process.push_back(input_queue_.front());
                    input_queue_.pop();
                }
            }
            
            // Process detections and move to output queue
            for (const auto& detection : detections_to_process) {
                {
                    std::lock_guard<std::mutex> lock(output_queue_mutex_);
                    if (output_queue_.size() < config_.output_buffer_size) {
                        output_queue_.push(detection);
                    }
                }
            }
            
            // Sleep for a short time
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
        } catch (const std::exception& e) {
            // Log error and continue
            updateMetrics();
        }
    }
}

void DetectionProcessor::outputWorkerThread() {
    auto next_output_time = std::chrono::steady_clock::now();
    const auto output_interval = std::chrono::milliseconds(
        static_cast<int>(1000.0 / config_.output_rate_hz));
    
    while (running_.load()) {
        try {
            auto current_time = std::chrono::steady_clock::now();
            
            if (current_time >= next_output_time) {
                sendToDataProcessor();
                next_output_time = current_time + output_interval;
            }
            
            // Sleep until next output time or check interval
            auto sleep_time = std::min(
                std::chrono::duration_cast<std::chrono::milliseconds>(next_output_time - current_time),
                std::chrono::milliseconds(50)
            );
            
            if (sleep_time > std::chrono::milliseconds(0)) {
                std::this_thread::sleep_for(sleep_time);
            }
            
        } catch (const std::exception& e) {
            // Log error and continue
            updateMetrics();
        }
    }
}

void DetectionProcessor::updateMetrics() {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_.detections_processed = processed_count_.load();
    metrics_.processing_time_ms = 5.0; // Simulate processing time
    metrics_.cpu_usage = 25.0; // Simulate CPU usage
    metrics_.memory_usage = 1024 * 1024 * 32; // 32MB
    metrics_.timestamp = std::chrono::system_clock::now();
    metrics_.errors_count = 0; // No errors for now
}

bool DetectionProcessor::validateConfiguration(const DetectionProcessorConfig& config) const {
    if (config.min_snr <= 0.0) {
        return false;
    }
    
    if (config.max_range <= config.min_range) {
        return false;
    }
    
    if (config.min_range <= 0.0) {
        return false;
    }
    
    if (config.max_detections == 0) {
        return false;
    }
    
    if (config.output_rate_hz <= 0.0) {
        return false;
    }
    
    if (config.confidence_threshold < 0.0 || config.confidence_threshold > 1.0) {
        return false;
    }
    
    return true;
}

bool DetectionProcessor::checkSNR(const common::Detection& detection) const {
    return detection.snr >= config_.min_snr;
}

bool DetectionProcessor::checkRange(const common::Detection& detection) const {
    return detection.range >= config_.min_range && detection.range <= config_.max_range;
}

bool DetectionProcessor::checkDoppler(const common::Detection& detection) const {
    return std::abs(detection.doppler_velocity) >= config_.doppler_threshold;
}

double DetectionProcessor::calculateConfidence(const common::Detection& detection) const {
    // Simple confidence calculation based on SNR and range
    double snr_factor = std::min(1.0, detection.snr / 20.0); // Normalize to 20dB max
    double range_factor = 1.0 - (detection.range / config_.max_range); // Closer is better
    
    // Combine factors
    double confidence = (snr_factor * 0.7 + range_factor * 0.3);
    
    return std::max(0.0, std::min(1.0, confidence));
}

void DetectionProcessor::initializeCommunication() {
    // In a real implementation, this would:
    // 1. Initialize DDS publisher
    // 2. Set up network connections
    // 3. Configure data serialization
}

void DetectionProcessor::shutdownCommunication() {
    // In a real implementation, this would:
    // 1. Cleanup DDS publisher
    // 2. Close network connections
    // 3. Release communication resources
}

bool DetectionProcessor::sendDetectionBatch(const std::vector<common::Detection>& detections) {
    // In a real implementation, this would:
    // 1. Serialize detection data
    // 2. Send via DDS/network to data processor
    // 3. Handle transmission errors
    
    // For now, just simulate successful transmission
    return !detections.empty();
}

} // namespace rsp
} // namespace radar