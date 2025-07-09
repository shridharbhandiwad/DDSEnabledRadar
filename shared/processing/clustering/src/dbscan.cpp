#include "clustering/dbscan.hpp"
#include <cmath>
#include <unordered_set>
#include <queue>
#include <algorithm>

namespace radar {
namespace processing {
namespace clustering {

DBSCAN::DBSCAN() {
    config_.name = "DBSCAN";
    config_.type = "DBSCAN";
    config_.parameters["epsilon"] = 50.0;
    config_.parameters["min_points"] = 2;
    config_.parameters["max_clusters"] = 100;
    config_.parameters["distance_metric"] = 0.0; // 0=euclidean, 1=mahalanobis
    config_.enabled = true;
}

std::vector<common::DetectionCluster> DBSCAN::cluster(
    const std::vector<common::Detection>& detections) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::vector<common::DetectionCluster> clusters;
    
    if (detections.empty()) {
        return clusters;
    }
    
    const double epsilon = config_.parameters.at("epsilon");
    const int min_points = static_cast<int>(config_.parameters.at("min_points"));
    const int max_clusters = static_cast<int>(config_.parameters.at("max_clusters"));
    
    // Initialize point classifications
    std::vector<PointType> point_types(detections.size(), PointType::UNCLASSIFIED);
    std::vector<int> cluster_assignments(detections.size(), -1);
    
    int current_cluster_id = 0;
    
    // Process each detection
    for (size_t i = 0; i < detections.size(); ++i) {
        if (point_types[i] != PointType::UNCLASSIFIED) {
            continue;
        }
        
        // Find neighbors
        auto neighbors = findNeighbors(detections, i, epsilon);
        
        if (neighbors.size() < static_cast<size_t>(min_points)) {
            point_types[i] = PointType::NOISE;
        } else {
            // Start new cluster
            if (current_cluster_id >= max_clusters) {
                break; // Limit number of clusters
            }
            
            expandCluster(detections, i, neighbors, current_cluster_id, 
                         epsilon, min_points, point_types, cluster_assignments);
            current_cluster_id++;
        }
    }
    
    // Create cluster objects
    clusters = createClusters(detections, cluster_assignments, current_cluster_id);
    
    // Update performance metrics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end_time - start_time);
    
    performance_metrics_.timestamp = end_time;
    performance_metrics_.processing_time_ms = duration.count();
    performance_metrics_.total_detections = detections.size();
    performance_metrics_.active_tracks = clusters.size();
    
    return clusters;
}

std::vector<size_t> DBSCAN::findNeighbors(
    const std::vector<common::Detection>& detections,
    size_t point_index,
    double epsilon) const {
    
    std::vector<size_t> neighbors;
    const auto& query_point = detections[point_index];
    
    for (size_t i = 0; i < detections.size(); ++i) {
        if (i == point_index) continue;
        
        double distance = calculateDistance(query_point, detections[i]);
        if (distance <= epsilon) {
            neighbors.push_back(i);
        }
    }
    
    return neighbors;
}

void DBSCAN::expandCluster(
    const std::vector<common::Detection>& detections,
    size_t point_index,
    std::vector<size_t>& neighbors,
    int cluster_id,
    double epsilon,
    int min_points,
    std::vector<PointType>& point_types,
    std::vector<int>& cluster_assignments) {
    
    // Add point to cluster
    point_types[point_index] = PointType::CORE;
    cluster_assignments[point_index] = cluster_id;
    
    // Process neighbors using a queue (breadth-first)
    std::queue<size_t> seed_queue;
    for (size_t neighbor : neighbors) {
        seed_queue.push(neighbor);
    }
    
    while (!seed_queue.empty()) {
        size_t current_point = seed_queue.front();
        seed_queue.pop();
        
        if (point_types[current_point] == PointType::NOISE) {
            point_types[current_point] = PointType::BORDER;
            cluster_assignments[current_point] = cluster_id;
        }
        
        if (point_types[current_point] != PointType::UNCLASSIFIED) {
            continue;
        }
        
        point_types[current_point] = PointType::CORE;
        cluster_assignments[current_point] = cluster_id;
        
        // Find neighbors of current point
        auto current_neighbors = findNeighbors(detections, current_point, epsilon);
        
        if (current_neighbors.size() >= static_cast<size_t>(min_points)) {
            // Add new neighbors to seed queue
            for (size_t neighbor : current_neighbors) {
                if (point_types[neighbor] == PointType::UNCLASSIFIED ||
                    point_types[neighbor] == PointType::NOISE) {
                    seed_queue.push(neighbor);
                }
            }
        } else {
            point_types[current_point] = PointType::BORDER;
        }
    }
}

std::vector<common::DetectionCluster> DBSCAN::createClusters(
    const std::vector<common::Detection>& detections,
    const std::vector<int>& cluster_assignments,
    int num_clusters) const {
    
    std::vector<common::DetectionCluster> clusters(num_clusters);
    
    // Initialize clusters
    for (int i = 0; i < num_clusters; ++i) {
        clusters[i].id = i + 1;
        clusters[i].algorithm = common::ClusteringAlgorithm::DBSCAN;
        clusters[i].timestamp = std::chrono::high_resolution_clock::now();
    }
    
    // Assign detections to clusters
    for (size_t i = 0; i < detections.size(); ++i) {
        int cluster_id = cluster_assignments[i];
        if (cluster_id >= 0 && cluster_id < num_clusters) {
            clusters[cluster_id].detections.push_back(detections[i]);
        }
    }
    
    // Calculate cluster properties
    for (auto& cluster : clusters) {
        if (!cluster.detections.empty()) {
            cluster.centroid = calculateCentroid(cluster.detections);
            cluster.covariance = calculateCovariance(cluster.detections, cluster.centroid);
            cluster.detection_count = cluster.detections.size();
            
            // Calculate cluster radius (max distance from centroid)
            double max_distance = 0.0;
            for (const auto& detection : cluster.detections) {
                double distance = std::sqrt(
                    std::pow(detection.position.x - cluster.centroid.x, 2) +
                    std::pow(detection.position.y - cluster.centroid.y, 2) +
                    std::pow(detection.position.z - cluster.centroid.z, 2)
                );
                max_distance = std::max(max_distance, distance);
            }
            cluster.cluster_radius = max_distance;
            
            // Calculate cluster density
            if (cluster.cluster_radius > 0) {
                double volume = (4.0/3.0) * M_PI * std::pow(cluster.cluster_radius, 3);
                cluster.cluster_density = cluster.detections.size() / volume;
            }
        }
    }
    
    // Remove empty clusters
    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
            [](const common::DetectionCluster& cluster) {
                return cluster.detections.empty();
            }),
        clusters.end()
    );
    
    return clusters;
}

bool DBSCAN::configure(const common::AlgorithmConfig& config) {
    if (!validateConfiguration(config)) {
        return false;
    }
    
    config_ = config;
    return true;
}

common::AlgorithmConfig DBSCAN::getConfiguration() const {
    return config_;
}

std::unordered_map<std::string, std::string> DBSCAN::getParameterDescriptions() const {
    return {
        {"epsilon", "Maximum distance between points in the same cluster (meters)"},
        {"min_points", "Minimum number of points required to form a dense region"},
        {"max_clusters", "Maximum number of clusters to create"},
        {"distance_metric", "Distance metric: 0=Euclidean, 1=Mahalanobis"}
    };
}

bool DBSCAN::validateConfiguration(const common::AlgorithmConfig& config) const {
    // Check required parameters
    std::vector<std::string> required_params = {"epsilon", "min_points", "max_clusters"};
    
    for (const auto& param : required_params) {
        if (config.parameters.find(param) == config.parameters.end()) {
            return false;
        }
    }
    
    // Validate parameter ranges
    double epsilon = config.parameters.at("epsilon");
    int min_points = static_cast<int>(config.parameters.at("min_points"));
    int max_clusters = static_cast<int>(config.parameters.at("max_clusters"));
    
    return epsilon > 0.0 && min_points >= 1 && max_clusters > 0;
}

common::ClusteringAlgorithm DBSCAN::getAlgorithmType() const {
    return common::ClusteringAlgorithm::DBSCAN;
}

std::string DBSCAN::getName() const {
    return "DBSCAN";
}

std::string DBSCAN::getVersion() const {
    return "1.0.0";
}

void DBSCAN::reset() {
    performance_metrics_ = common::PerformanceMetrics{};
}

common::PerformanceMetrics DBSCAN::getPerformanceMetrics() const {
    return performance_metrics_;
}

} // namespace clustering
} // namespace processing
} // namespace radar