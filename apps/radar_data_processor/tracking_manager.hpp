#pragma once

#include <memory>
#include <vector>
#include <unordered_map>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include "common/types.hpp"
#include "interfaces/i_logger.hpp"
#include "interfaces/i_filter.hpp"

namespace radar {
namespace rdp {

struct TrackingManagerConfig {
    // Track management parameters
    uint32_t max_tracks{1000};              // Maximum number of active tracks
    uint32_t min_hits_for_confirmation{3};  // Minimum hits to confirm a track
    uint32_t max_coasts{5};                 // Maximum coast cycles before deletion
    double track_deletion_timeout_s{30.0}; // Track deletion timeout
    
    // Track initiation
    double initiation_threshold{0.8};       // Track initiation threshold
    double confirmation_threshold{0.9};     // Track confirmation threshold
    size_t max_new_tracks_per_cycle{50};    // Maximum new tracks per processing cycle
    
    // Track prediction
    bool enable_prediction{true};           // Enable track prediction
    double prediction_interval_s{0.1};     // Prediction time interval
    double max_prediction_time_s{1.0};     // Maximum prediction time
    
    // Filter settings
    common::FilterType default_filter{common::FilterType::KALMAN};
    double process_noise{0.1};              // Process noise standard deviation
    double measurement_noise{1.0};          // Measurement noise standard deviation
    
    // Quality control
    double min_track_quality{0.5};         // Minimum track quality threshold
    bool enable_track_merging{true};        // Enable track merging
    bool enable_track_splitting{false};     // Enable track splitting
    double merge_distance_threshold{10.0};  // Distance threshold for track merging
    
    // Performance settings
    size_t processing_threads{2};           // Number of processing threads
    double processing_timeout_ms{200.0};    // Processing timeout
};

class TrackingManager {
public:
    TrackingManager();
    ~TrackingManager();
    
    // Configuration
    bool configure(const TrackingManagerConfig& config);
    TrackingManagerConfig getConfiguration() const;
    
    // Lifecycle management
    bool start();
    void stop();
    bool isRunning() const;
    
    // Core tracking functionality
    void processAssociations(const std::vector<common::Association>& associations);
    std::vector<common::Track> updateTracks();
    void predictTracks();
    
    // Track management
    std::vector<common::Track> getActiveTracks() const;
    std::vector<common::Track> getTracksByStatus(common::TrackStatus status) const;
    common::Track* getTrack(common::TrackId track_id);
    bool deleteTrack(common::TrackId track_id);
    
    // Track lifecycle management
    void manageTrackLifecycle();
    void cleanupDeletedTracks();
    void validateTrackQuality();
    void checkTrackMergingSplitting();
    
    // Track creation and management
    common::TrackId createNewTrack(const common::Detection& detection);
    bool updateTrackWithDetection(common::TrackId track_id, const common::Detection& detection);
    void promoteTrackToConfirmed(common::TrackId track_id);
    void markTrackForDeletion(common::TrackId track_id);
    
    // Statistics and monitoring
    size_t getActiveTrackCount() const;
    size_t getTentativeTrackCount() const;
    size_t getConfirmedTrackCount() const;
    size_t getCoastingTrackCount() const;
    common::PerformanceMetrics getPerformanceMetrics() const;
    
private:
    // Internal implementation
    struct Impl;
    std::unique_ptr<Impl> pImpl_;
    
    // Configuration
    TrackingManagerConfig config_;
    std::mutex config_mutex_;
    
    // State management
    std::atomic<bool> running_{false};
    std::atomic<bool> configured_{false};
    
    // Track storage
    std::unordered_map<common::TrackId, std::unique_ptr<common::Track>> tracks_;
    mutable std::mutex tracks_mutex_;
    
    // Track ID management
    std::atomic<common::TrackId> next_track_id_{1};
    
    // Processing threads
    std::vector<std::thread> processing_threads_;
    std::thread management_thread_;
    
    // Filters
    std::unordered_map<common::TrackId, std::unique_ptr<interfaces::IFilter>> filters_;
    std::mutex filters_mutex_;
    
    // Statistics
    std::atomic<size_t> tracks_created_{0};
    std::atomic<size_t> tracks_deleted_{0};
    std::atomic<size_t> tracks_confirmed_{0};
    std::atomic<size_t> associations_processed_{0};
    
    // Metrics
    mutable std::mutex metrics_mutex_;
    common::PerformanceMetrics metrics_;
    
    // Internal methods
    void processingWorkerThread();
    void managementWorkerThread();
    void updateMetrics();
    bool validateConfiguration(const TrackingManagerConfig& config) const;
    
    // Track management methods
    void initializeTrackingSystem();
    void shutdownTrackingSystem();
    common::TrackId generateNewTrackId();
    void updateTrackStatistics();
    
    // Track processing methods
    void processNewDetections(const std::vector<common::Detection>& unassociated_detections);
    void updateExistingTracks(const std::vector<common::Association>& associations);
    void predictUnassociatedTracks();
    void manageTrackTransitions();
    
    // Track quality methods
    double calculateTrackQuality(const common::Track& track) const;
    bool shouldDeleteTrack(const common::Track& track) const;
    bool shouldConfirmTrack(const common::Track& track) const;
    
    // Filter management
    std::unique_ptr<interfaces::IFilter> createFilter(common::FilterType type);
    bool initializeTrackFilter(common::TrackId track_id, const common::Detection& initial_detection);
    void updateTrackFilter(common::TrackId track_id, const common::Detection& detection);
    void predictTrackFilter(common::TrackId track_id, double time_delta);
    
    // Track merging and splitting
    std::vector<std::pair<common::TrackId, common::TrackId>> findMergeCandidates();
    bool mergeTracks(common::TrackId track1_id, common::TrackId track2_id);
    std::vector<common::TrackId> findSplitCandidates();
    bool splitTrack(common::TrackId track_id);
    
    // Utilities
    double calculateDistance(const common::Track& track1, const common::Track& track2) const;
    std::chrono::duration<double> getTimeSinceLastUpdate(const common::Track& track) const;
    void updateTrackHistory(common::Track& track, const common::Detection& detection);
};

} // namespace rdp
} // namespace radar