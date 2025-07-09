#pragma once

#include "interfaces/i_clustering.hpp"
#include <vector>
#include <random>

namespace radar {
namespace processing {
namespace clustering {

/**
 * @brief K-Means clustering algorithm implementation
 * 
 * This class implements the K-Means clustering algorithm which partitions detections
 * into k clusters by minimizing the within-cluster sum of squares. It's effective
 * for radar data when the number of expected targets is approximately known.
 */
class KMeans : public interfaces::IClustering {
private:
    /**
     * @brief Centroid information for K-Means algorithm
     */
    struct Centroid {
        common::Point3D position;
        std::vector<size_t> assigned_points;
        double sum_squared_distances{0.0};
        
        Centroid() = default;
        Centroid(const common::Point3D& pos) : position(pos) {}
    };

    common::AlgorithmConfig config_;
    common::PerformanceMetrics performance_metrics_;
    mutable std::mt19937 random_generator_;

public:
    KMeans();
    ~KMeans() override = default;

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

    /**
     * @brief Set random seed for reproducible results
     * @param seed Random seed value
     */
    void setSeed(uint32_t seed);

private:
    /**
     * @brief Initialize centroids using various methods
     * @param detections Input detection points
     * @param k Number of clusters
     * @param method Initialization method
     * @return Vector of initial centroids
     */
    std::vector<Centroid> initializeCentroids(
        const std::vector<common::Detection>& detections,
        int k,
        const std::string& method = "kmeans++") const;

    /**
     * @brief Initialize centroids randomly
     * @param detections Input detection points
     * @param k Number of clusters
     * @return Vector of random centroids
     */
    std::vector<Centroid> initializeRandomCentroids(
        const std::vector<common::Detection>& detections,
        int k) const;

    /**
     * @brief Initialize centroids using K-Means++ method
     * @param detections Input detection points
     * @param k Number of clusters
     * @return Vector of K-Means++ centroids
     */
    std::vector<Centroid> initializeKMeansPlusPlusCentroids(
        const std::vector<common::Detection>& detections,
        int k) const;

    /**
     * @brief Assign each detection to the nearest centroid
     * @param detections Input detection points
     * @param centroids Current centroids
     * @return Vector of cluster assignments for each detection
     */
    std::vector<int> assignPointsToCentroids(
        const std::vector<common::Detection>& detections,
        const std::vector<Centroid>& centroids) const;

    /**
     * @brief Update centroid positions based on assigned points
     * @param detections Input detection points
     * @param assignments Point-to-cluster assignments
     * @param centroids Centroids to update
     * @return True if centroids changed significantly
     */
    bool updateCentroids(
        const std::vector<common::Detection>& detections,
        const std::vector<int>& assignments,
        std::vector<Centroid>& centroids) const;

    /**
     * @brief Calculate total within-cluster sum of squares
     * @param detections Input detection points
     * @param assignments Point-to-cluster assignments
     * @param centroids Current centroids
     * @return Total WCSS value
     */
    double calculateWCSS(
        const std::vector<common::Detection>& detections,
        const std::vector<int>& assignments,
        const std::vector<Centroid>& centroids) const;

    /**
     * @brief Determine optimal number of clusters using elbow method
     * @param detections Input detection points
     * @param max_k Maximum number of clusters to test
     * @return Optimal number of clusters
     */
    int determineOptimalK(
        const std::vector<common::Detection>& detections,
        int max_k) const;

    /**
     * @brief Create cluster objects from final assignments
     * @param detections Original detection points
     * @param assignments Point-to-cluster assignments
     * @param centroids Final centroids
     * @return Vector of detection clusters
     */
    std::vector<common::DetectionCluster> createClusters(
        const std::vector<common::Detection>& detections,
        const std::vector<int>& assignments,
        const std::vector<Centroid>& centroids) const;

    /**
     * @brief Check convergence criteria
     * @param old_centroids Previous centroid positions
     * @param new_centroids New centroid positions
     * @param tolerance Convergence tolerance
     * @return True if converged
     */
    bool hasConverged(
        const std::vector<Centroid>& old_centroids,
        const std::vector<Centroid>& new_centroids,
        double tolerance) const;
};

} // namespace clustering
} // namespace processing
} // namespace radar