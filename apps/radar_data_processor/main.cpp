#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <signal.h>
#include <atomic>

#include "data_processor.hpp"
#include "tracking_manager.hpp"
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

class RadarDataProcessorApp {
private:
    std::unique_ptr<rdp::DataProcessor> data_processor_;
    std::unique_ptr<rdp::TrackingManager> tracking_manager_;
    std::shared_ptr<interfaces::ILogger> logger_;
    std::shared_ptr<configuration::ConfigManager> config_manager_;
    
    std::thread processing_thread_;
    std::thread tracking_thread_;
    std::thread output_thread_;
    std::thread management_thread_;
    
    common::PerformanceMetrics performance_metrics_;
    std::atomic<size_t> active_tracks_{0};
    std::atomic<size_t> total_detections_processed_{0};
    
public:
    RadarDataProcessorApp() = default;
    ~RadarDataProcessorApp() = default;

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

            logger_->info("Radar Data Processor", "Initializing application...");

            // Initialize data processor
            data_processor_ = std::make_unique<rdp::DataProcessor>();
            auto dp_config = config_manager_->getDataProcessorConfig();
            if (!data_processor_->configure(dp_config)) {
                logger_->error("Radar Data Processor", "Failed to configure data processor");
                return false;
            }

            // Initialize tracking manager
            tracking_manager_ = std::make_unique<rdp::TrackingManager>();
            auto tm_config = config_manager_->getTrackingManagerConfig();
            if (!tracking_manager_->configure(tm_config)) {
                logger_->error("Radar Data Processor", "Failed to configure tracking manager");
                return false;
            }

            logger_->info("Radar Data Processor", "Application initialized successfully");
            return true;

        } catch (const std::exception& e) {
            std::cerr << "Exception during initialization: " << e.what() << std::endl;
            return false;
        }
    }

    bool start() {
        try {
            logger_->info("Radar Data Processor", "Starting application...");

            // Start data processor
            if (!data_processor_->start()) {
                logger_->error("Radar Data Processor", "Failed to start data processor");
                return false;
            }

            // Start tracking manager
            if (!tracking_manager_->start()) {
                logger_->error("Radar Data Processor", "Failed to start tracking manager");
                return false;
            }

            // Start processing threads
            processing_thread_ = std::thread(&RadarDataProcessorApp::processingLoop, this);
            tracking_thread_ = std::thread(&RadarDataProcessorApp::trackingLoop, this);
            output_thread_ = std::thread(&RadarDataProcessorApp::outputLoop, this);
            management_thread_ = std::thread(&RadarDataProcessorApp::managementLoop, this);

            logger_->info("Radar Data Processor", "Application started successfully");
            return true;

        } catch (const std::exception& e) {
            logger_->error("Radar Data Processor", "Exception during startup: " + std::string(e.what()));
            return false;
        }
    }

    void stop() {
        logger_->info("Radar Data Processor", "Stopping application...");

        g_shutdown = true;

        // Stop processing threads
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        if (tracking_thread_.joinable()) {
            tracking_thread_.join();
        }
        if (output_thread_.joinable()) {
            output_thread_.join();
        }
        if (management_thread_.joinable()) {
            management_thread_.join();
        }

        // Stop components
        if (tracking_manager_) {
            tracking_manager_->stop();
        }
        if (data_processor_) {
            data_processor_->stop();
        }

        logger_->info("Radar Data Processor", "Application stopped");
    }

    void run() {
        logger_->info("Radar Data Processor", "Running main loop...");
        
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
                logger_->info("Radar Data Processor", "Configuration changed, reloading...");
                reloadConfiguration();
            }

            // Check system health
            checkSystemHealth();

            std::this_thread::sleep_for(100ms);
        }
    }

private:
    void processingLoop() {
        logger_->info("Processing Thread", "Starting data processing loop");
        
        while (!g_shutdown) {
            try {
                auto start_time = std::chrono::high_resolution_clock::now();

                // Receive detections from Signal Processor
                auto detection_batch = data_processor_->receiveDetections();
                
                if (!detection_batch.empty()) {
                    total_detections_processed_ += detection_batch.size();
                    
                    // Perform clustering
                    auto clusters = data_processor_->performClustering(detection_batch);
                    
                    // Perform association
                    auto associations = data_processor_->performAssociation(clusters);
                    
                    // Pass to tracking manager
                    tracking_manager_->processAssociations(associations);
                    
                    // Calculate processing time
                    auto end_time = std::chrono::high_resolution_clock::now();
                    auto processing_time = std::chrono::duration<double, std::milli>(end_time - start_time);
                    
                    // Update performance metrics
                    updatePerformanceMetrics(processing_time.count(), detection_batch.size());
                    
                    // Log periodic processing statistics
                    static size_t process_count = 0;
                    if (++process_count % 50 == 0) {
                        logger_->debug("Processing Thread", 
                            "Processed " + std::to_string(detection_batch.size()) + 
                            " detections in " + std::to_string(processing_time.count()) + " ms");
                    }
                }

            } catch (const std::exception& e) {
                logger_->error("Processing Thread", "Exception in processing loop: " + std::string(e.what()));
            }

            // Control processing rate
            std::this_thread::sleep_for(50ms);
        }
        
        logger_->info("Processing Thread", "Processing loop terminated");
    }

    void trackingLoop() {
        logger_->info("Tracking Thread", "Starting tracking loop");
        
        while (!g_shutdown) {
            try {
                auto start_time = std::chrono::high_resolution_clock::now();

                // Update all active tracks
                auto tracks = tracking_manager_->updateTracks();
                active_tracks_ = tracks.size();
                
                // Perform prediction for tracks without detections
                tracking_manager_->predictTracks();
                
                // Calculate tracking time
                auto end_time = std::chrono::high_resolution_clock::now();
                auto tracking_time = std::chrono::duration<double, std::milli>(end_time - start_time);
                
                // Log periodic tracking statistics
                static size_t track_count = 0;
                if (++track_count % 100 == 0) {
                    logger_->debug("Tracking Thread", 
                        "Updated " + std::to_string(tracks.size()) + 
                        " tracks in " + std::to_string(tracking_time.count()) + " ms");
                }

            } catch (const std::exception& e) {
                logger_->error("Tracking Thread", "Exception in tracking loop: " + std::string(e.what()));
            }

            // Control tracking rate
            std::this_thread::sleep_for(100ms);
        }
        
        logger_->info("Tracking Thread", "Tracking loop terminated");
    }

    void outputLoop() {
        logger_->info("Output Thread", "Starting output loop");
        
        while (!g_shutdown) {
            try {
                // Send tracks to HMI
                data_processor_->sendToHMI();
                
                // Send tracks to Fusion system
                data_processor_->sendToFusion();
                
                // Send performance metrics
                data_processor_->sendPerformanceMetrics(performance_metrics_);
                
            } catch (const std::exception& e) {
                logger_->error("Output Thread", "Exception in output loop: " + std::string(e.what()));
            }

            std::this_thread::sleep_for(200ms);
        }
        
        logger_->info("Output Thread", "Output loop terminated");
    }

    void managementLoop() {
        logger_->info("Management Thread", "Starting management loop");
        
        while (!g_shutdown) {
            try {
                // Perform track lifecycle management
                tracking_manager_->manageTrackLifecycle();
                
                // Clean up deleted tracks
                tracking_manager_->cleanupDeletedTracks();
                
                // Validate track quality
                tracking_manager_->validateTrackQuality();
                
                // Check for track merging/splitting opportunities
                tracking_manager_->checkTrackMergingSplitting();
                
            } catch (const std::exception& e) {
                logger_->error("Management Thread", "Exception in management loop: " + std::string(e.what()));
            }

            // Management runs less frequently
            std::this_thread::sleep_for(1s);
        }
        
        logger_->info("Management Thread", "Management loop terminated");
    }

    void updatePerformanceMetrics(double processing_time_ms, size_t detection_count) {
        performance_metrics_.timestamp = std::chrono::high_resolution_clock::now();
        performance_metrics_.processing_time_ms = processing_time_ms;
        performance_metrics_.total_detections = detection_count;
        performance_metrics_.active_tracks = active_tracks_;
        
        // Calculate CPU and memory usage (simplified)
        performance_metrics_.cpu_usage = 65.0; // TODO: Implement actual CPU monitoring
        performance_metrics_.memory_usage_mb = 256.0; // TODO: Implement actual memory monitoring
        
        // Calculate detection rate
        static auto last_time = std::chrono::high_resolution_clock::now();
        static size_t last_detections = 0;
        
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_diff = std::chrono::duration<double>(current_time - last_time).count();
        
        if (time_diff >= 1.0) { // Update every second
            auto detection_diff = total_detections_processed_ - last_detections;
            performance_metrics_.detection_rate = detection_diff / time_diff;
            
            last_time = current_time;
            last_detections = total_detections_processed_;
        }
    }

    void reportStatus() {
        logger_->info("Status", 
            "Active tracks: " + std::to_string(active_tracks_) + ", " +
            "Processing time: " + std::to_string(performance_metrics_.processing_time_ms) + " ms, " +
            "Detection rate: " + std::to_string(performance_metrics_.detection_rate) + " Hz, " +
            "Total detections: " + std::to_string(total_detections_processed_) + ", " +
            "CPU: " + std::to_string(performance_metrics_.cpu_usage) + "%, " +
            "Memory: " + std::to_string(performance_metrics_.memory_usage_mb) + " MB");
    }

    void checkSystemHealth() {
        // Simple health checks
        bool healthy = true;
        
        // Check if processing is keeping up
        if (performance_metrics_.processing_time_ms > 50.0) {
            logger_->warn("Health Check", "Processing time exceeding 50ms threshold");
            healthy = false;
        }
        
        // Check memory usage
        if (performance_metrics_.memory_usage_mb > 1024.0) {
            logger_->warn("Health Check", "Memory usage exceeding 1GB threshold");
            healthy = false;
        }
        
        // Check number of active tracks
        if (active_tracks_ > 500) {
            logger_->warn("Health Check", "Number of active tracks exceeding 500");
            healthy = false;
        }
        
        if (!healthy) {
            logger_->warn("Health Check", "System health degraded");
        }
    }

    void reloadConfiguration() {
        try {
            config_manager_->reloadConfiguration();
            
            // Update component configurations
            if (data_processor_) {
                auto dp_config = config_manager_->getDataProcessorConfig();
                data_processor_->configure(dp_config);
            }
            
            if (tracking_manager_) {
                auto tm_config = config_manager_->getTrackingManagerConfig();
                tracking_manager_->configure(tm_config);
            }
            
            logger_->info("Radar Data Processor", "Configuration reloaded successfully");
            
        } catch (const std::exception& e) {
            logger_->error("Radar Data Processor", 
                "Failed to reload configuration: " + std::string(e.what()));
        }
    }
};

int main(int argc, char* argv[]) {
    // Install signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "Radar Data Processor v1.0.0" << std::endl;
    std::cout << "Defense Radar Tracking System" << std::endl;
    std::cout << "=============================" << std::endl;

    // Parse command line arguments
    std::string config_file = "config/rdp_config.yaml";
    if (argc > 1) {
        config_file = argv[1];
    }

    std::cout << "Using configuration file: " << config_file << std::endl;

    try {
        // Create and initialize application
        RadarDataProcessorApp app;
        
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

        std::cout << "Radar Data Processor terminated successfully" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Fatal exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown fatal exception occurred" << std::endl;
        return 1;
    }
}