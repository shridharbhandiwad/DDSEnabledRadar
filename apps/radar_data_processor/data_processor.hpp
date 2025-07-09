#pragma once

#include <memory>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>
#include "common/types.hpp"
#include "interfaces/i_logger.hpp"
#include "interfaces/i_clustering.hpp"
#include "interfaces/i_association.hpp"

namespace radar {
namespace rdp {

struct DataProcessorConfig {
    // Clustering configuration
    common::ClusteringAlgorithm clustering_algorithm{common::ClusteringAlgorithm::DBSCAN};
    double clustering_epsilon{5.0};         // DBSCAN epsilon parameter
    size_t clustering_min_points{3};        // DBSCAN minimum points
    size_t max_clusters{100};               // Maximum clusters per scan
    
    // Association configuration
    common::AssociationAlgorithm association_algorithm{common::AssociationAlgorithm::GLOBAL_NEAREST_NEIGHBOR};
    double association_gate{9.0};           // Association gate (Mahalanobis distance)
    double association_threshold{0.8};      // Association probability threshold
    
    // Processing settings
    size_t input_buffer_size{10000};        // Input detection buffer size
    size_t processing_threads{4};           // Number of processing threads
    double processing_timeout_ms{100.0};    // Processing timeout
    
    // Output settings
    double output_rate_hz{25.0};            // Output rate to HMI/Fusion
    size_t max_tracks_per_message{500};     // Maximum tracks per output message
    bool enable_track_prediction{true};     // Enable track prediction
    
    // Communication settings
    std::string hmi_topic{"tracks_hmi"};
    std::string fusion_topic{"tracks_fusion"};
    std::string metrics_topic{"performance_metrics"};
};

class DataProcessor {
public:
    DataProcessor();
    ~DataProcessor();
    
    // Configuration
    bool configure(const DataProcessorConfig& config);
    DataProcessorConfig getConfiguration() const;
    
    // Lifecycle management
    bool start();
    void stop();
    bool isRunning() const;
    
    // Core processing functionality
    std::vector<common::Detection> receiveDetections();
    std::vector<common::DetectionCluster> performClustering(const std::vector<common::Detection>& detections);
    std::vector<common::Association> performAssociation(const std::vector<common::DetectionCluster>& clusters);
    
    // Output management
    void sendToHMI();
    void sendToFusion();
    void sendPerformanceMetrics(const common::PerformanceMetrics& metrics);
    
    // Data management
    void processDetectionBatch(const std::vector<common::Detection>& detections);
    size_t getQueuedDetectionCount() const;
    void clearInputBuffer();
    
    // Monitoring and diagnostics
    common::PerformanceMetrics getPerformanceMetrics() const;
    size_t getProcessedDetectionCount() const;
    size_t getClusterCount() const;
    size_t getAssociationCount() const;
    
private:
    // Internal implementation
    struct Impl;
    std::unique_ptr<Impl> pImpl_;
    
    // Configuration
    DataProcessorConfig config_;
    mutable std::mutex config_mutex_;
    
    // State management
    std::atomic<bool> running_{false};
    std::atomic<bool> configured_{false};
    
    // Processing components
    std::unique_ptr<interfaces::IClustering> clustering_engine_;
    std::unique_ptr<interfaces::IAssociation> association_engine_;
    
    // Data queues
    std::queue<common::Detection> input_queue_;
    std::queue<common::DetectionCluster> cluster_queue_;
    std::queue<common::Association> association_queue_;
    mutable std::mutex input_queue_mutex_;
    std::mutex cluster_queue_mutex_;
    std::mutex association_queue_mutex_;
    
    // Processing threads
    std::vector<std::thread> processing_threads_;
    std::thread output_thread_;
    std::thread communication_thread_;
    
    // Statistics
    std::atomic<size_t> detections_processed_{0};
    std::atomic<size_t> clusters_created_{0};
    std::atomic<size_t> associations_created_{0};
    std::atomic<size_t> tracks_output_{0};
    
    // Metrics
    mutable std::mutex metrics_mutex_;
    common::PerformanceMetrics metrics_;
    
    // Internal methods
    void processingWorkerThread();
    void outputWorkerThread();
    void communicationWorkerThread();
    void updateMetrics();
    bool validateConfiguration(const DataProcessorConfig& config) const;
    
    // Processing methods
    void initializeProcessingComponents();
    void shutdownProcessingComponents();
    std::vector<common::DetectionCluster> clusterDetections(const std::vector<common::Detection>& detections);
    std::vector<common::Association> associateClusters(const std::vector<common::DetectionCluster>& clusters);
    
    // Communication methods
    void initializeCommunication();
    void shutdownCommunication();
    bool sendTrackBatch(const std::vector<common::Track>& tracks, const std::string& topic);
    void publishMetrics(const common::PerformanceMetrics& metrics);
    
    // Data conversion methods
    std::vector<common::Track> convertAssociationsToTracks(const std::vector<common::Association>& associations);
    common::Track createTrackFromAssociation(const common::Association& association);
};

} // namespace rdp
} // namespace radar