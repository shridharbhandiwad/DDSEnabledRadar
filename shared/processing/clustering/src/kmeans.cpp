#include "clustering/kmeans.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <numeric>

namespace radar {
namespace processing {
namespace clustering {

KMeans::KMeans() {
    config_.name = "KMeans";
    config_.type = "KMEANS";
    config_.parameters["k"] = 3;
    config_.parameters["max_iterations"] = 100;
    config_.parameters["tolerance"] = 1e-4;
    config_.parameters["initialization"] = 0.0; // 0=kmeans++, 1=random
    config_.parameters["auto_k"] = 0.0; // 0=false, 1=true (auto determine k)
    config_.parameters["max_k"] = 10;
    config_.enabled = true;
    
    // Initialize random generator
    random_generator_.seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());
}

std::vector<common::DetectionCluster> KMeans::cluster(
    const std::vector<common::Detection>& detections) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::vector<common::DetectionCluster> clusters;
    
    if (detections.empty()) {
        return clusters;
    }
    
    const bool auto_k = static_cast<bool>(config_.parameters.at("auto_k"));
    const int max_k = static_cast<int>(config_.parameters.at("max_k"));
    const int max_iterations = static_cast<int>(config_.parameters.at("max_iterations"));
    const double tolerance = config_.parameters.at("tolerance");
    const int init_method = static_cast<int>(config_.parameters.at("initialization"));
    
    // Determine number of clusters
    int k = static_cast<int>(config_.parameters.at("k"));
    if (auto_k) {
        k = determineOptimalK(detections, max_k);
    }
    
    // Ensure k doesn't exceed number of detections
    k = std::min(k, static_cast<int>(detections.size()));
    if (k <= 0) {
        return clusters;
    }
    
    // Initialize centroids
    std::string init_method_str = (init_method == 0) ? "kmeans++" : "random";
    auto centroids = initializeCentroids(detections, k, init_method_str);
    
    std::vector<int> assignments(detections.size(), -1);
    std::vector<int> previous_assignments(detections.size(), -1);
    
    // Main K-Means iteration loop
    bool converged = false;
    int iteration = 0;
    
    while (!converged && iteration < max_iterations) {
        // Store previous assignments
        previous_assignments = assignments;
        
        // Assign points to nearest centroids
        assignments = assignPointsToCentroids(detections, centroids);
        
        // Update centroids
        auto old_centroids = centroids;
        updateCentroids(detections, assignments, centroids);
        
        // Check convergence
        converged = hasConverged(old_centroids, centroids, tolerance);
        
        // Also check if assignments haven't changed
        if (!converged) {
            converged = (assignments == previous_assignments);
        }
        
        iteration++;
    }
    
    // Create cluster objects
    clusters = createClusters(detections, assignments, centroids);
    
    // Update performance metrics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end_time - start_time);
    
    performance_metrics_.timestamp = end_time;
    performance_metrics_.processing_time_ms = duration.count();
    performance_metrics_.total_detections = detections.size();
    performance_metrics_.active_tracks = clusters.size();
    
    return clusters;
}

std::vector<KMeans::Centroid> KMeans::initializeCentroids(
    const std::vector<common::Detection>& detections,
    int k,
    const std::string& method) const {
    
    if (method == "kmeans++") {
        return initializeKMeansPlusPlusCentroids(detections, k);
    } else {
        return initializeRandomCentroids(detections, k);
    }
}

std::vector<KMeans::Centroid> KMeans::initializeRandomCentroids(
    const std::vector<common::Detection>& detections,
    int k) const {
    
    std::vector<Centroid> centroids;
    centroids.reserve(k);
    
    std::uniform_int_distribution<size_t> dist(0, detections.size() - 1);
    std::set<size_t> selected_indices;
    
    // Select k random unique points as initial centroids
    while (centroids.size() < static_cast<size_t>(k)) {
        size_t index = dist(random_generator_);
        if (selected_indices.find(index) == selected_indices.end()) {
            selected_indices.insert(index);
            centroids.emplace_back(detections[index].position);
        }
    }
    
    return centroids;
}

std::vector<KMeans::Centroid> KMeans::initializeKMeansPlusPlusCentroids(
    const std::vector<common::Detection>& detections,
    int k) const {
    
    std::vector<Centroid> centroids;
    centroids.reserve(k);
    
    // Choose first centroid randomly
    std::uniform_int_distribution<size_t> dist(0, detections.size() - 1);
    size_t first_index = dist(random_generator_);
    centroids.emplace_back(detections[first_index].position);
    
    // Choose remaining centroids using K-Means++ method
    for (int i = 1; i < k; ++i) {
        std::vector<double> distances(detections.size());
        double total_distance = 0.0;
        
        // Calculate squared distances to nearest existing centroid
        for (size_t j = 0; j < detections.size(); ++j) {
            double min_dist_sq = std::numeric_limits<double>::max();
            
            for (const auto& centroid : centroids) {
                double dist_sq = 
                    std::pow(detections[j].position.x - centroid.position.x, 2) +
                    std::pow(detections[j].position.y - centroid.position.y, 2) +
                    std::pow(detections[j].position.z - centroid.position.z, 2);
                min_dist_sq = std::min(min_dist_sq, dist_sq);
            }
            
            distances[j] = min_dist_sq;
            total_distance += min_dist_sq;
        }
        
        // Choose next centroid with probability proportional to squared distance
        std::uniform_real_distribution<double> uniform_dist(0.0, total_distance);
        double random_value = uniform_dist(random_generator_);
        
        double cumulative_distance = 0.0;
        for (size_t j = 0; j < detections.size(); ++j) {
            cumulative_distance += distances[j];
            if (cumulative_distance >= random_value) {
                centroids.emplace_back(detections[j].position);
                break;
            }
        }
    }
    
    return centroids;
}

std::vector<int> KMeans::assignPointsToCentroids(
    const std::vector<common::Detection>& detections,
    const std::vector<Centroid>& centroids) const {
    
    std::vector<int> assignments(detections.size());
    
    for (size_t i = 0; i < detections.size(); ++i) {
        double min_distance = std::numeric_limits<double>::max();
        int nearest_centroid = 0;
        
        for (size_t j = 0; j < centroids.size(); ++j) {
            double distance = calculateDistance(detections[i], 
                common::Detection{0, 0, {}, centroids[j].position});
            
            if (distance < min_distance) {
                min_distance = distance;
                nearest_centroid = static_cast<int>(j);
            }
        }
        
        assignments[i] = nearest_centroid;
    }
    
    return assignments;
}

bool KMeans::updateCentroids(
    const std::vector<common::Detection>& detections,
    const std::vector<int>& assignments,
    std::vector<Centroid>& centroids) const {
    
    bool changed = false;
    
    for (size_t i = 0; i < centroids.size(); ++i) {
        common::Point3D new_position{0.0, 0.0, 0.0};
        int count = 0;
        
        // Calculate new centroid position as mean of assigned points
        for (size_t j = 0; j < detections.size(); ++j) {
            if (assignments[j] == static_cast<int>(i)) {
                new_position.x += detections[j].position.x;
                new_position.y += detections[j].position.y;
                new_position.z += detections[j].position.z;
                count++;
            }
        }
        
        if (count > 0) {
            new_position.x /= count;
            new_position.y /= count;
            new_position.z /= count;
            
            // Check if centroid moved significantly
            double movement = std::sqrt(
                std::pow(new_position.x - centroids[i].position.x, 2) +
                std::pow(new_position.y - centroids[i].position.y, 2) +
                std::pow(new_position.z - centroids[i].position.z, 2)
            );
            
            if (movement > 1e-6) {
                changed = true;
                centroids[i].position = new_position;
            }
        }
    }
    
    return changed;
}

double KMeans::calculateWCSS(
    const std::vector<common::Detection>& detections,
    const std::vector<int>& assignments,
    const std::vector<Centroid>& centroids) const {
    
    double wcss = 0.0;
    
    for (size_t i = 0; i < detections.size(); ++i) {
        int cluster_id = assignments[i];
        if (cluster_id >= 0 && cluster_id < static_cast<int>(centroids.size())) {
            double distance_sq = 
                std::pow(detections[i].position.x - centroids[cluster_id].position.x, 2) +
                std::pow(detections[i].position.y - centroids[cluster_id].position.y, 2) +
                std::pow(detections[i].position.z - centroids[cluster_id].position.z, 2);
            wcss += distance_sq;
        }
    }
    
    return wcss;
}

int KMeans::determineOptimalK(
    const std::vector<common::Detection>& detections,
    int max_k) const {
    
    std::vector<double> wcss_values;
    
    // Calculate WCSS for different values of k
    for (int k = 1; k <= max_k && k <= static_cast<int>(detections.size()); ++k) {
        auto centroids = initializeCentroids(detections, k, "kmeans++");
        auto assignments = assignPointsToCentroids(detections, centroids);
        
        // Run a few iterations to get stable WCSS
        for (int iter = 0; iter < 10; ++iter) {
            updateCentroids(detections, assignments, centroids);
            assignments = assignPointsToCentroids(detections, centroids);
        }
        
        double wcss = calculateWCSS(detections, assignments, centroids);
        wcss_values.push_back(wcss);
    }
    
    // Find elbow point using simple heuristic
    int optimal_k = 1;
    if (wcss_values.size() >= 3) {
        double max_improvement_ratio = 0.0;
        
        for (size_t i = 1; i < wcss_values.size() - 1; ++i) {
            double improvement1 = wcss_values[i-1] - wcss_values[i];
            double improvement2 = wcss_values[i] - wcss_values[i+1];
            
            if (improvement2 > 0) {
                double ratio = improvement1 / improvement2;
                if (ratio > max_improvement_ratio) {
                    max_improvement_ratio = ratio;
                    optimal_k = static_cast<int>(i + 1);
                }
            }
        }
    }
    
    return optimal_k;
}

std::vector<common::DetectionCluster> KMeans::createClusters(
    const std::vector<common::Detection>& detections,
    const std::vector<int>& assignments,
    const std::vector<Centroid>& centroids) const {
    
    std::vector<common::DetectionCluster> clusters(centroids.size());
    
    // Initialize clusters
    for (size_t i = 0; i < centroids.size(); ++i) {
        clusters[i].id = i + 1;
        clusters[i].algorithm = common::ClusteringAlgorithm::KMEANS;
        clusters[i].timestamp = std::chrono::high_resolution_clock::now();
        clusters[i].centroid = centroids[i].position;
    }
    
    // Assign detections to clusters
    for (size_t i = 0; i < detections.size(); ++i) {
        int cluster_id = assignments[i];
        if (cluster_id >= 0 && cluster_id < static_cast<int>(clusters.size())) {
            clusters[cluster_id].detections.push_back(detections[i]);
        }
    }
    
    // Calculate cluster properties
    for (auto& cluster : clusters) {
        if (!cluster.detections.empty()) {
            cluster.detection_count = cluster.detections.size();
            cluster.covariance = calculateCovariance(cluster.detections, cluster.centroid);
            
            // Calculate cluster radius
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

bool KMeans::hasConverged(
    const std::vector<Centroid>& old_centroids,
    const std::vector<Centroid>& new_centroids,
    double tolerance) const {
    
    if (old_centroids.size() != new_centroids.size()) {
        return false;
    }
    
    for (size_t i = 0; i < old_centroids.size(); ++i) {
        double distance = std::sqrt(
            std::pow(old_centroids[i].position.x - new_centroids[i].position.x, 2) +
            std::pow(old_centroids[i].position.y - new_centroids[i].position.y, 2) +
            std::pow(old_centroids[i].position.z - new_centroids[i].position.z, 2)
        );
        
        if (distance > tolerance) {
            return false;
        }
    }
    
    return true;
}

void KMeans::setSeed(uint32_t seed) {
    random_generator_.seed(seed);
}

bool KMeans::configure(const common::AlgorithmConfig& config) {
    if (!validateConfiguration(config)) {
        return false;
    }
    
    config_ = config;
    return true;
}

common::AlgorithmConfig KMeans::getConfiguration() const {
    return config_;
}

std::unordered_map<std::string, std::string> KMeans::getParameterDescriptions() const {
    return {
        {"k", "Number of clusters (ignored if auto_k is enabled)"},
        {"max_iterations", "Maximum number of iterations"},
        {"tolerance", "Convergence tolerance for centroid movement"},
        {"initialization", "Initialization method: 0=K-Means++, 1=Random"},
        {"auto_k", "Automatically determine optimal k: 0=false, 1=true"},
        {"max_k", "Maximum k to consider when auto_k is enabled"}
    };
}

bool KMeans::validateConfiguration(const common::AlgorithmConfig& config) const {
    std::vector<std::string> required_params = {"k", "max_iterations", "tolerance"};
    
    for (const auto& param : required_params) {
        if (config.parameters.find(param) == config.parameters.end()) {
            return false;
        }
    }
    
    int k = static_cast<int>(config.parameters.at("k"));
    int max_iterations = static_cast<int>(config.parameters.at("max_iterations"));
    double tolerance = config.parameters.at("tolerance");
    
    return k > 0 && max_iterations > 0 && tolerance > 0.0;
}

common::ClusteringAlgorithm KMeans::getAlgorithmType() const {
    return common::ClusteringAlgorithm::KMEANS;
}

std::string KMeans::getName() const {
    return "K-Means";
}

std::string KMeans::getVersion() const {
    return "1.0.0";
}

void KMeans::reset() {
    performance_metrics_ = common::PerformanceMetrics{};
}

common::PerformanceMetrics KMeans::getPerformanceMetrics() const {
    return performance_metrics_;
}

} // namespace clustering
} // namespace processing
} // namespace radar