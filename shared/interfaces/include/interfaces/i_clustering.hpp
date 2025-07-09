#pragma once

#include "common/types.hpp"
#include <memory>
#include <vector>

namespace radar {
namespace interfaces {

/**
 * @brief Abstract interface for clustering algorithms
 * 
 * This interface defines the contract for clustering algorithms used in
 * radar data processing. Implementations can include DBSCAN, K-Means,
 * hierarchical clustering, and custom algorithms.
 */
class IClustering {
public:
    virtual ~IClustering() = default;

    /**
     * @brief Perform clustering on a set of detections
     * @param detections Input detections to cluster
     * @return Vector of detection clusters
     */
    virtual std::vector<common::DetectionCluster> cluster(
        const std::vector<common::Detection>& detections) = 0;

    /**
     * @brief Configure the clustering algorithm
     * @param config Configuration parameters
     * @return true if configuration was successful
     */
    virtual bool configure(const common::AlgorithmConfig& config) = 0;

    /**
     * @brief Get the current configuration
     * @return Current algorithm configuration
     */
    virtual common::AlgorithmConfig getConfiguration() const = 0;

    /**
     * @brief Get algorithm-specific parameters
     * @return Map of parameter names to descriptions
     */
    virtual std::unordered_map<std::string, std::string> getParameterDescriptions() const = 0;

    /**
     * @brief Validate configuration parameters
     * @param config Configuration to validate
     * @return true if configuration is valid
     */
    virtual bool validateConfiguration(const common::AlgorithmConfig& config) const = 0;

    /**
     * @brief Get the algorithm type
     * @return Algorithm type identifier
     */
    virtual common::ClusteringAlgorithm getAlgorithmType() const = 0;

    /**
     * @brief Get algorithm name
     * @return Human-readable algorithm name
     */
    virtual std::string getName() const = 0;

    /**
     * @brief Get algorithm version
     * @return Algorithm version string
     */
    virtual std::string getVersion() const = 0;

    /**
     * @brief Reset algorithm state
     */
    virtual void reset() = 0;

    /**
     * @brief Get performance metrics from last clustering operation
     * @return Performance metrics
     */
    virtual common::PerformanceMetrics getPerformanceMetrics() const = 0;

protected:
    // Common utility functions for derived classes
    
    /**
     * @brief Calculate distance between two detections
     * @param det1 First detection
     * @param det2 Second detection
     * @return Euclidean distance
     */
    static double calculateDistance(const common::Detection& det1, 
                                  const common::Detection& det2);

    /**
     * @brief Calculate cluster centroid
     * @param detections Detections in the cluster
     * @return Centroid point
     */
    static common::Point3D calculateCentroid(
        const std::vector<common::Detection>& detections);

    /**
     * @brief Calculate cluster covariance matrix
     * @param detections Detections in the cluster
     * @param centroid Cluster centroid
     * @return Covariance matrix
     */
    static common::Matrix3d calculateCovariance(
        const std::vector<common::Detection>& detections,
        const common::Point3D& centroid);
};

/**
 * @brief Factory interface for creating clustering algorithms
 */
class IClusteringFactory {
public:
    virtual ~IClusteringFactory() = default;

    /**
     * @brief Create a clustering algorithm instance
     * @param algorithm Algorithm type to create
     * @param config Initial configuration
     * @return Unique pointer to clustering algorithm
     */
    virtual std::unique_ptr<IClustering> create(
        common::ClusteringAlgorithm algorithm,
        const common::AlgorithmConfig& config = {}) = 0;

    /**
     * @brief Get list of supported algorithms
     * @return Vector of supported algorithm types
     */
    virtual std::vector<common::ClusteringAlgorithm> getSupportedAlgorithms() const = 0;

    /**
     * @brief Check if algorithm is supported
     * @param algorithm Algorithm type to check
     * @return true if algorithm is supported
     */
    virtual bool isSupported(common::ClusteringAlgorithm algorithm) const = 0;

    /**
     * @brief Get default configuration for an algorithm
     * @param algorithm Algorithm type
     * @return Default configuration
     */
    virtual common::AlgorithmConfig getDefaultConfiguration(
        common::ClusteringAlgorithm algorithm) const = 0;
};

} // namespace interfaces
} // namespace radar