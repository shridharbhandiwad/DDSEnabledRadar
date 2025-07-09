#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <signal.h>
#include <atomic>

#include "signal_processor.hpp"
#include "detection_processor.hpp"
#include "common/types.hpp"
#include "logging/logger.hpp"
#include "configuration/config_manager.hpp"

using namespace radar;
using namespace std::chrono_literals;

// Global shutdown flag
std::atomic<bool> g_shutdown{false};

// Signal handler for graceful shutdown
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ". Initiating graceful shutdown..." << std::endl;
    g_shutdown = true;
}

class RadarSignalProcessorApp {
private:
    std::unique_ptr<rsp::SignalProcessor> signal_processor_;
    std::unique_ptr<rsp::DetectionProcessor> detection_processor_;
    std::shared_ptr<interfaces::ILogger> logger_;
    std::shared_ptr<configuration::ConfigManager> config_manager_;
    
    std::thread processing_thread_;
    std::thread output_thread_;
    
    common::PerformanceMetrics performance_metrics_;
    
public:
    RadarSignalProcessorApp() = default;
    ~RadarSignalProcessorApp() = default;

    bool initialize(const std::string& config_file) {
        try {
            // Initialize configuration manager
            config_manager_ = std::make_shared<configuration::ConfigManager>();
            if (!config_manager_->loadConfiguration(config_file)) {
                std::cerr << "Failed to load configuration from: " << config_file << std::endl;
                return false;
            }

            // Initialize logger
            logger_ = std::make_shared<logging::Logger>();
            auto log_config = config_manager_->getLoggingConfig();
            if (!logger_->configure(log_config)) {
                std::cerr << "Failed to configure logger" << std::endl;
                return false;
            }

            logger_->info("Radar Signal Processor", "Initializing application...");

            // Initialize signal processor
            signal_processor_ = std::make_unique<rsp::SignalProcessor>();
            auto sp_config = buildSignalProcessorConfig();
            if (!signal_processor_->configure(sp_config)) {
                logger_->error("Radar Signal Processor", "Failed to configure signal processor");
                return false;
            }

            // Initialize detection processor
            detection_processor_ = std::make_unique<rsp::DetectionProcessor>();
            auto dp_config = buildDetectionProcessorConfig();
            if (!detection_processor_->configure(dp_config)) {
                logger_->error("Radar Signal Processor", "Failed to configure detection processor");
                return false;
            }

            logger_->info("Radar Signal Processor", "Application initialized successfully");
            return true;

        } catch (const std::exception& e) {
            std::cerr << "Exception during initialization: " << e.what() << std::endl;
            return false;
        }
    }

    bool start() {
        try {
            logger_->info("Radar Signal Processor", "Starting application...");

            // Start signal processor
            if (!signal_processor_->start()) {
                logger_->error("Radar Signal Processor", "Failed to start signal processor");
                return false;
            }

            // Start detection processor
            if (!detection_processor_->start()) {
                logger_->error("Radar Signal Processor", "Failed to start detection processor");
                return false;
            }

            // Start processing threads
            processing_thread_ = std::thread(&RadarSignalProcessorApp::processingLoop, this);
            output_thread_ = std::thread(&RadarSignalProcessorApp::outputLoop, this);

            logger_->info("Radar Signal Processor", "Application started successfully");
            return true;

        } catch (const std::exception& e) {
            logger_->error("Radar Signal Processor", "Exception during startup: " + std::string(e.what()));
            return false;
        }
    }

    void stop() {
        logger_->info("Radar Signal Processor", "Stopping application...");

        g_shutdown = true;

        // Stop processing threads
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        if (output_thread_.joinable()) {
            output_thread_.join();
        }

        // Stop processors
        if (detection_processor_) {
            detection_processor_->stop();
        }
        if (signal_processor_) {
            signal_processor_->stop();
        }

        logger_->info("Radar Signal Processor", "Application stopped");
    }

    void run() {
        logger_->info("Radar Signal Processor", "Running main loop...");
        
        auto last_status_time = std::chrono::steady_clock::now();
        const auto status_interval = 10s;

        while (!g_shutdown) {
            auto current_time = std::chrono::steady_clock::now();
            
            // Periodic status reporting
            if (current_time - last_status_time >= status_interval) {
                reportStatus();
                last_status_time = current_time;
            }

            // Check for configuration updates
            if (config_manager_->hasConfigurationChanged()) {
                logger_->info("Radar Signal Processor", "Configuration changed, reloading...");
                reloadConfiguration();
            }

            std::this_thread::sleep_for(100ms);
        }
    }

private:
    void processingLoop() {
        logger_->info("Processing Thread", "Starting processing loop");
        
        while (!g_shutdown) {
            try {
                auto start_time = std::chrono::high_resolution_clock::now();

                // Process incoming radar signals (simulated)
                auto raw_data = signal_processor_->processSignals();
                
                // Perform detection processing
                auto detections = detection_processor_->processDetections(raw_data);
                
                // Calculate processing time
                auto end_time = std::chrono::high_resolution_clock::now();
                auto processing_time = std::chrono::duration<double, std::milli>(end_time - start_time);
                
                // Update performance metrics
                updatePerformanceMetrics(processing_time.count(), detections.size());
                
                // Log periodic processing statistics
                static size_t process_count = 0;
                if (++process_count % 100 == 0) {
                    logger_->debug("Processing Thread", 
                        "Processed " + std::to_string(detections.size()) + 
                        " detections in " + std::to_string(processing_time.count()) + " ms");
                }

            } catch (const std::exception& e) {
                logger_->error("Processing Thread", "Exception in processing loop: " + std::string(e.what()));
            }

            // Control processing rate
            std::this_thread::sleep_for(100ms);
        }
        
        logger_->info("Processing Thread", "Processing loop terminated");
    }

    void outputLoop() {
        logger_->info("Output Thread", "Starting output loop");
        
        while (!g_shutdown) {
            try {
                // Send processed data to Data Processor
                detection_processor_->sendToDataProcessor();
                
                // Send status updates
                signal_processor_->sendStatusUpdate();
                
            } catch (const std::exception& e) {
                logger_->error("Output Thread", "Exception in output loop: " + std::string(e.what()));
            }

            std::this_thread::sleep_for(50ms);
        }
        
        logger_->info("Output Thread", "Output loop terminated");
    }

    void updatePerformanceMetrics(double processing_time_ms, size_t detection_count) {
        performance_metrics_.timestamp = std::chrono::high_resolution_clock::now();
        performance_metrics_.processing_time_ms = processing_time_ms;
        performance_metrics_.total_detections = detection_count;
        
        // Calculate CPU and memory usage (simplified)
        performance_metrics_.cpu_usage = 45.0; // TODO: Implement actual CPU monitoring
        performance_metrics_.memory_usage = 128.0 * 1024 * 1024; // 128MB in bytes: TODO: Implement actual memory monitoring
    }

    void reportStatus() {
        logger_->info("Status", 
            "Processing time: " + std::to_string(performance_metrics_.processing_time_ms) + " ms, " +
            "Detections: " + std::to_string(performance_metrics_.total_detections) + ", " +
            "CPU: " + std::to_string(performance_metrics_.cpu_usage) + "%, " +
            "Memory: " + std::to_string(performance_metrics_.memory_usage / (1024.0 * 1024.0)) + " MB");
    }

    void reloadConfiguration() {
        try {
            config_manager_->reloadConfiguration();
            
            // Update component configurations
            if (signal_processor_) {
                auto sp_config = buildSignalProcessorConfig();
                signal_processor_->configure(sp_config);
            }
            
            if (detection_processor_) {
                auto dp_config = buildDetectionProcessorConfig();
                detection_processor_->configure(dp_config);
            }
            
            logger_->info("Radar Signal Processor", "Configuration reloaded successfully");
            
        } catch (const std::exception& e) {
            logger_->error("Radar Signal Processor", 
                "Failed to reload configuration: " + std::string(e.what()));
        }
    }
    
    rsp::SignalProcessorConfig buildSignalProcessorConfig() {
        rsp::SignalProcessorConfig config;
        config.sampling_rate = config_manager_->getParameter<double>("signal_processor", "sampling_rate", 1000.0);
        config.threshold = config_manager_->getParameter<double>("signal_processor", "threshold", 0.1);
        config.window_size = config_manager_->getParameter<int>("signal_processor", "window_size", 1024);
        config.overlap = config_manager_->getParameter<int>("signal_processor", "overlap", 512);
        config.enable_cfar = config_manager_->getParameter<bool>("signal_processor", "enable_cfar", true);
        config.cfar_threshold = config_manager_->getParameter<double>("signal_processor", "cfar_threshold", 3.0);
        config.max_detections_per_scan = config_manager_->getParameter<int>("signal_processor", "max_detections_per_scan", 500);
        config.thread_count = config_manager_->getParameter<int>("signal_processor", "thread_count", 4);
        config.buffer_size = config_manager_->getParameter<int>("signal_processor", "buffer_size", 10000);
        config.processing_timeout_ms = config_manager_->getParameter<double>("signal_processor", "processing_timeout_ms", 50.0);
        return config;
    }
    
    rsp::DetectionProcessorConfig buildDetectionProcessorConfig() {
        rsp::DetectionProcessorConfig config;
        config.min_snr = config_manager_->getParameter<double>("detection_processor", "min_snr", 3.0);
        config.max_range = config_manager_->getParameter<double>("detection_processor", "max_range", 50000.0);
        config.min_range = config_manager_->getParameter<double>("detection_processor", "min_range", 100.0);
        config.doppler_threshold = config_manager_->getParameter<double>("detection_processor", "doppler_threshold", 0.5);
        config.max_detections = config_manager_->getParameter<int>("detection_processor", "max_detections", 1000);
        config.enable_quality_check = config_manager_->getParameter<bool>("detection_processor", "enable_quality_check", true);
        config.confidence_threshold = config_manager_->getParameter<double>("detection_processor", "confidence_threshold", 0.7);
        config.output_buffer_size = config_manager_->getParameter<int>("detection_processor", "output_buffer_size", 5000);
        config.output_rate_hz = config_manager_->getParameter<double>("detection_processor", "output_rate_hz", 50.0);
        config.enable_compression = config_manager_->getParameter<bool>("detection_processor", "enable_compression", false);
        config.data_processor_topic = config_manager_->getParameter<std::string>("detection_processor", "data_processor_topic", "detections");
        config.heartbeat_interval_ms = config_manager_->getParameter<double>("detection_processor", "heartbeat_interval_ms", 1000.0);
        return config;
    }
};

int main(int argc, char* argv[]) {
    // Install signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "Radar Signal Processor v1.0.0" << std::endl;
    std::cout << "Defense Radar Tracking System" << std::endl;
    std::cout << "=============================" << std::endl;

    // Parse command line arguments
    std::string config_file = "config/rsp_config.yaml";
    if (argc > 1) {
        config_file = argv[1];
    }

    std::cout << "Using configuration file: " << config_file << std::endl;

    try {
        // Create and initialize application
        RadarSignalProcessorApp app;
        
        if (!app.initialize(config_file)) {
            std::cerr << "Failed to initialize application" << std::endl;
            return 1;
        }

        if (!app.start()) {
            std::cerr << "Failed to start application" << std::endl;
            return 1;
        }

        // Run main loop
        app.run();

        // Graceful shutdown
        app.stop();

        std::cout << "Radar Signal Processor terminated successfully" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Fatal exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown fatal exception occurred" << std::endl;
        return 1;
    }
}