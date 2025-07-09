#pragma once

#include "interfaces/i_clustering.hpp"
#include <vector>
#include <chrono>

namespace radar {
namespace processing {
namespace clustering {

/**
 * @brief DBSCAN (Density-Based Spatial Clustering of Applications with Noise) implementation
 * 
 * This class implements the DBSCAN clustering algorithm which groups together points
 * that are closely packed while marking points in low-density areas as outliers.
 * It's particularly effective for radar data as it can handle noise and varying cluster shapes.
 */
class DBSCAN : public interfaces::IClustering {
private:
    /**
     * @brief Point classification types for DBSCAN algorithm
     */
    enum class PointType {
        UNCLASSIFIED,   // Point not yet processed
        CORE,           // Core point with sufficient neighbors
        BORDER,         // Border point (neighbor of core point)
        NOISE           // Noise point (outlier)
    };

    common::AlgorithmConfig config_;
    common::PerformanceMetrics performance_metrics_;

public:
    DBSCAN();
    ~DBSCAN() override = default;

    // IClustering interface implementation
    std::vector<common::DetectionCluster> cluster(
        const std::vector<common::Detection>& detections) override;

    bool configure(const common::AlgorithmConfig& config) override;
    common::AlgorithmConfig getConfiguration() const override;
    
    std::unordered_map<std::string, std::string> getParameterDescriptions() const override;
    bool validateConfiguration(const common::AlgorithmConfig& config) const override;
    
    common::ClusteringAlgorithm getAlgorithmType() const override;
    std::string getName() const override;
    std::string getVersion() const override;
    
    void reset() override;
    common::PerformanceMetrics getPerformanceMetrics() const override;

private:
    /**
     * @brief Find all neighbors within epsilon distance of a point
     * @param detections All detection points
     * @param point_index Index of the query point
     * @param epsilon Maximum distance for neighborhood
     * @return Vector of neighbor indices
     */
    std::vector<size_t> findNeighbors(
        const std::vector<common::Detection>& detections,
        size_t point_index,
        double epsilon) const;

    /**
     * @brief Expand cluster by adding density-reachable points
     * @param detections All detection points
     * @param point_index Starting core point index
     * @param neighbors Initial neighbors of the core point
     * @param cluster_id Current cluster ID
     * @param epsilon Distance threshold
     * @param min_points Minimum points for core point
     * @param point_types Point classification array
     * @param cluster_assignments Cluster assignment array
     */
    void expandCluster(
        const std::vector<common::Detection>& detections,
        size_t point_index,
        std::vector<size_t>& neighbors,
        int cluster_id,
        double epsilon,
        int min_points,
        std::vector<PointType>& point_types,
        std::vector<int>& cluster_assignments);

    /**
     * @brief Create cluster objects from assignments
     * @param detections Original detection points
     * @param cluster_assignments Point-to-cluster assignments
     * @param num_clusters Number of clusters found
     * @return Vector of detection clusters
     */
    std::vector<common::DetectionCluster> createClusters(
        const std::vector<common::Detection>& detections,
        const std::vector<int>& cluster_assignments,
        int num_clusters) const;
};

} // namespace clustering
} // namespace processing
} // namespace radar