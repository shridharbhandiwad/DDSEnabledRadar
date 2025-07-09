#include "clustering/hierarchical_clustering.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>

namespace radar {
namespace processing {
namespace clustering {

HierarchicalClustering::HierarchicalClustering() {
    config_.name = "HierarchicalClustering";
    config_.type = "HIERARCHICAL";
    config_.parameters["linkage_criterion"] = 0.0;      // 0=single, 1=complete, 2=average, 3=ward
    config_.parameters["num_clusters"] = 3.0;           // Target number of clusters
    config_.parameters["distance_threshold"] = 50.0;    // Distance threshold for cutting dendrogram
    config_.parameters["use_threshold"] = 0.0;          // 0=use num_clusters, 1=use distance_threshold
    config_.enabled = true;
    
    linkage_criterion_ = LinkageCriterion::AVERAGE;
    target_num_clusters_ = static_cast<int>(config_.parameters["num_clusters"]);
    distance_threshold_ = config_.parameters["distance_threshold"];
}

std::vector<common::DetectionCluster> HierarchicalClustering::cluster(
    const std::vector<common::Detection>& detections) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::vector<common::DetectionCluster> result;
    
    if (detections.empty()) {
        return result;
    }
    
    if (detections.size() == 1) {
        // Single detection forms one cluster
        common::DetectionCluster single_cluster;
        single_cluster.id = 0;
        single_cluster.detections = detections;
        single_cluster.centroid = detections[0].position;
        single_cluster.detection_count = 1;
        single_cluster.cluster_radius = 0.0;
        single_cluster.cluster_density = 1.0;
        single_cluster.algorithm = common::ClusteringAlgorithm::HIERARCHICAL;
        single_cluster.timestamp = std::chrono::high_resolution_clock::now();
        
        result.push_back(single_cluster);
        return result;
    }
    
    // Build distance matrix
    buildDistanceMatrix(detections);
    
    // Perform agglomerative clustering
    dendrogram_root_ = performAgglomerativeClustering(detections);
    
    // Extract clusters based on configuration
    std::vector<int> assignments;
    bool use_threshold = static_cast<bool>(config_.parameters.at("use_threshold"));
    
    if (use_threshold) {
        assignments = cutDendrogram(distance_threshold_);
    } else {
        assignments = cutDendrogramToNClusters(target_num_clusters_);
    }
    
    // Create final clustering result
    result = createClusteringResult(assignments, detections);
    
    // Update performance metrics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end_time - start_time);
    
    performance_metrics_.timestamp = end_time;
    performance_metrics_.processing_time_ms = duration.count();
    performance_metrics_.total_detections = detections.size();
    
    return result;
}

void HierarchicalClustering::buildDistanceMatrix(const std::vector<common::Detection>& detections) {
    size_t n = detections.size();
    distance_matrix_.clear();
    distance_matrix_.resize(n, std::vector<double>(n, 0.0));
    
    // Build symmetric distance matrix
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            double dist = calculateDistance(detections[i], detections[j]);
            distance_matrix_[i][j] = dist;
            distance_matrix_[j][i] = dist;  // Symmetric
        }
    }
}

std::shared_ptr<HierarchicalClustering::ClusterNode> 
HierarchicalClustering::performAgglomerativeClustering(
    const std::vector<common::Detection>& detections) {
    
    // Initialize: each detection is its own cluster
    std::vector<std::shared_ptr<ClusterNode>> active_clusters;
    all_nodes_.clear();
    
    for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
        auto node = std::make_shared<ClusterNode>(i);
        node->id = i;
        node->merge_level = 0;
        active_clusters.push_back(node);
        all_nodes_.push_back(node);
    }
    
    int next_node_id = detections.size();
    
    // Agglomerative clustering: merge closest clusters iteratively
    while (active_clusters.size() > 1) {
        // Find closest pair of clusters
        auto closest_pair = findClosestClusters(active_clusters, detections);
        int idx1 = closest_pair.first;
        int idx2 = closest_pair.second;
        
        // Get the clusters to merge
        auto cluster1 = active_clusters[idx1];
        auto cluster2 = active_clusters[idx2];
        
        // Calculate merge distance
        double merge_distance = calculateClusterDistance(cluster1, cluster2, detections);
        
        // Create new merged cluster
        auto merged_cluster = mergeClusters(cluster1, cluster2, merge_distance, next_node_id++);
        all_nodes_.push_back(merged_cluster);
        
        // Remove the merged clusters and add the new one
        // Remove higher index first to avoid invalidating indices
        if (idx1 > idx2) {
            active_clusters.erase(active_clusters.begin() + idx1);
            active_clusters.erase(active_clusters.begin() + idx2);
        } else {
            active_clusters.erase(active_clusters.begin() + idx2);
            active_clusters.erase(active_clusters.begin() + idx1);
        }
        
        active_clusters.push_back(merged_cluster);
    }
    
    // The last remaining cluster is the root of the dendrogram
    return active_clusters[0];
}

std::pair<int, int> HierarchicalClustering::findClosestClusters(
    const std::vector<std::shared_ptr<ClusterNode>>& active_clusters,
    const std::vector<common::Detection>& detections) const {
    
    double min_distance = std::numeric_limits<double>::max();
    std::pair<int, int> closest_pair(0, 1);
    
    for (int i = 0; i < static_cast<int>(active_clusters.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(active_clusters.size()); ++j) {
            double distance = calculateClusterDistance(active_clusters[i], active_clusters[j], detections);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_pair = std::make_pair(i, j);
            }
        }
    }
    
    return closest_pair;
}

double HierarchicalClustering::calculateClusterDistance(
    const std::shared_ptr<ClusterNode>& cluster1,
    const std::shared_ptr<ClusterNode>& cluster2,
    const std::vector<common::Detection>& detections) const {
    
    switch (linkage_criterion_) {
        case LinkageCriterion::SINGLE:
            return calculateSingleLinkage(cluster1, cluster2);
        case LinkageCriterion::COMPLETE:
            return calculateCompleteLinkage(cluster1, cluster2);
        case LinkageCriterion::AVERAGE:
            return calculateAverageLinkage(cluster1, cluster2);
        case LinkageCriterion::WARD:
            return calculateWardLinkage(cluster1, cluster2, detections);
        default:
            return calculateAverageLinkage(cluster1, cluster2);
    }
}

double HierarchicalClustering::calculateSingleLinkage(
    const std::shared_ptr<ClusterNode>& cluster1,
    const std::shared_ptr<ClusterNode>& cluster2) const {
    
    double min_distance = std::numeric_limits<double>::max();
    
    for (int i : cluster1->point_indices) {
        for (int j : cluster2->point_indices) {
            min_distance = std::min(min_distance, distance_matrix_[i][j]);
        }
    }
    
    return min_distance;
}

double HierarchicalClustering::calculateCompleteLinkage(
    const std::shared_ptr<ClusterNode>& cluster1,
    const std::shared_ptr<ClusterNode>& cluster2) const {
    
    double max_distance = 0.0;
    
    for (int i : cluster1->point_indices) {
        for (int j : cluster2->point_indices) {
            max_distance = std::max(max_distance, distance_matrix_[i][j]);
        }
    }
    
    return max_distance;
}

double HierarchicalClustering::calculateAverageLinkage(
    const std::shared_ptr<ClusterNode>& cluster1,
    const std::shared_ptr<ClusterNode>& cluster2) const {
    
    double total_distance = 0.0;
    int count = 0;
    
    for (int i : cluster1->point_indices) {
        for (int j : cluster2->point_indices) {
            total_distance += distance_matrix_[i][j];
            count++;
        }
    }
    
    return count > 0 ? total_distance / count : 0.0;
}

double HierarchicalClustering::calculateWardLinkage(
    const std::shared_ptr<ClusterNode>& cluster1,
    const std::shared_ptr<ClusterNode>& cluster2,
    const std::vector<common::Detection>& detections) const {
    
    // Ward linkage: minimize increase in within-cluster sum of squares
    
    // Calculate current within-cluster sum of squares for both clusters
    double wss1 = calculateWithinClusterSS(cluster1, detections);
    double wss2 = calculateWithinClusterSS(cluster2, detections);
    
    // Create temporary merged cluster to calculate merged WSS
    auto temp_merged = std::make_shared<ClusterNode>();
    temp_merged->point_indices = cluster1->point_indices;
    temp_merged->point_indices.insert(temp_merged->point_indices.end(),
                                     cluster2->point_indices.begin(),
                                     cluster2->point_indices.end());
    
    double wss_merged = calculateWithinClusterSS(temp_merged, detections);
    
    // Ward distance is the increase in WSS
    return wss_merged - wss1 - wss2;
}

std::shared_ptr<HierarchicalClustering::ClusterNode> 
HierarchicalClustering::mergeClusters(
    std::shared_ptr<ClusterNode> cluster1,
    std::shared_ptr<ClusterNode> cluster2,
    double merge_distance,
    int node_id) const {
    
    auto merged_cluster = std::make_shared<ClusterNode>();
    merged_cluster->id = node_id;
    merged_cluster->left = cluster1;
    merged_cluster->right = cluster2;
    merged_cluster->merge_distance = merge_distance;
    merged_cluster->merge_level = std::max(cluster1->merge_level, cluster2->merge_level) + 1;
    
    // Combine point indices
    merged_cluster->point_indices = cluster1->point_indices;
    merged_cluster->point_indices.insert(merged_cluster->point_indices.end(),
                                        cluster2->point_indices.begin(),
                                        cluster2->point_indices.end());
    
    return merged_cluster;
}

common::Vector3d HierarchicalClustering::calculateClusterCentroid(
    const std::shared_ptr<ClusterNode>& cluster,
    const std::vector<common::Detection>& detections) const {
    
    if (cluster->point_indices.empty()) {
        return common::Vector3d::Zero();
    }
    
    common::Vector3d centroid = common::Vector3d::Zero();
    
    for (int idx : cluster->point_indices) {
        auto pos = detections[idx].position.toEigen();
        centroid += pos;
    }
    
    centroid /= static_cast<double>(cluster->point_indices.size());
    return centroid;
}

double HierarchicalClustering::calculateWithinClusterSS(
    const std::shared_ptr<ClusterNode>& cluster,
    const std::vector<common::Detection>& detections) const {
    
    if (cluster->point_indices.size() <= 1) {
        return 0.0;
    }
    
    auto centroid = calculateClusterCentroid(cluster, detections);
    double wss = 0.0;
    
    for (int idx : cluster->point_indices) {
        auto pos = detections[idx].position.toEigen();
        auto diff = pos - centroid;
        wss += diff.squaredNorm();
    }
    
    return wss;
}

std::vector<int> HierarchicalClustering::cutDendrogram(double threshold) const {
    if (!dendrogram_root_) {
        return {};
    }
    
    std::vector<int> assignments(distance_matrix_.size(), -1);
    int cluster_id = 0;
    
    extractClustersAtThreshold(dendrogram_root_, threshold, cluster_id, assignments);
    
    return assignments;
}

std::vector<int> HierarchicalClustering::cutDendrogramToNClusters(int num_clusters) const {
    if (!dendrogram_root_ || num_clusters <= 0) {
        return {};
    }
    
    std::vector<int> assignments(distance_matrix_.size(), -1);
    int cluster_id = 0;
    int current_clusters = 0;
    
    extractNClusters(dendrogram_root_, num_clusters, current_clusters, cluster_id, assignments);
    
    return assignments;
}

void HierarchicalClustering::extractClustersAtThreshold(
    const std::shared_ptr<ClusterNode>& node,
    double threshold,
    int& cluster_id,
    std::vector<int>& assignments) const {
    
    if (!node) return;
    
    // If this is a leaf node or merge distance exceeds threshold, create cluster
    if (!node->left || !node->right || node->merge_distance <= threshold) {
        // Assign all points in this subtree to current cluster
        for (int idx : node->point_indices) {
            assignments[idx] = cluster_id;
        }
        cluster_id++;
    } else {
        // Recursively process children
        extractClustersAtThreshold(node->left, threshold, cluster_id, assignments);
        extractClustersAtThreshold(node->right, threshold, cluster_id, assignments);
    }
}

void HierarchicalClustering::extractNClusters(
    const std::shared_ptr<ClusterNode>& node,
    int target_clusters,
    int& current_clusters,
    int& cluster_id,
    std::vector<int>& assignments) const {
    
    if (!node || current_clusters >= target_clusters) return;
    
    // If this is a leaf node or we've reached target clusters, create cluster
    if (!node->left || !node->right || current_clusters >= target_clusters) {
        // Assign all points in this subtree to current cluster
        for (int idx : node->point_indices) {
            assignments[idx] = cluster_id;
        }
        cluster_id++;
        current_clusters++;
    } else {
        // Check if splitting this node would exceed target clusters
        int potential_leaves = 2;  // This split would create 2 leaf clusters
        if (current_clusters + potential_leaves <= target_clusters) {
            // Safe to split
            extractNClusters(node->left, target_clusters, current_clusters, cluster_id, assignments);
            extractNClusters(node->right, target_clusters, current_clusters, cluster_id, assignments);
        } else {
            // Don't split, treat as single cluster
            for (int idx : node->point_indices) {
                assignments[idx] = cluster_id;
            }
            cluster_id++;
            current_clusters++;
        }
    }
}

std::vector<double> HierarchicalClustering::getMergeDistances() const {
    std::vector<double> distances;
    
    for (const auto& node : all_nodes_) {
        if (node && node->left && node->right) {  // Internal node
            distances.push_back(node->merge_distance);
        }
    }
    
    // Sort distances in ascending order
    std::sort(distances.begin(), distances.end());
    
    return distances;
}

std::vector<common::DetectionCluster> HierarchicalClustering::createClusteringResult(
    const std::vector<int>& assignments,
    const std::vector<common::Detection>& detections) const {
    
    std::vector<common::DetectionCluster> result;
    
    // Find maximum cluster ID to determine number of clusters
    int max_cluster_id = *std::max_element(assignments.begin(), assignments.end());
    
    if (max_cluster_id < 0) {
        return result;
    }
    
    // Initialize clusters
    std::vector<common::DetectionCluster> clusters(max_cluster_id + 1);
    for (int i = 0; i <= max_cluster_id; ++i) {
        clusters[i].id = i;
        clusters[i].detection_count = 0;
        clusters[i].algorithm = common::ClusteringAlgorithm::HIERARCHICAL;
        clusters[i].timestamp = std::chrono::high_resolution_clock::now();
    }
    
    // Assign detections to clusters
    for (size_t i = 0; i < detections.size(); ++i) {
        int cluster_id = assignments[i];
        if (cluster_id >= 0) {
            clusters[cluster_id].detections.push_back(detections[i]);
            clusters[cluster_id].detection_count++;
        }
    }
    
    // Calculate cluster properties
    for (auto& cluster : clusters) {
        if (cluster.detection_count > 0) {
            // Calculate centroid
            common::Vector3d centroid_sum = common::Vector3d::Zero();
            for (const auto& detection : cluster.detections) {
                centroid_sum += detection.position.toEigen();
            }
            centroid_sum /= static_cast<double>(cluster.detection_count);
            cluster.centroid = common::Point3D(centroid_sum.x(), centroid_sum.y(), centroid_sum.z());
            
            // Calculate cluster radius (maximum distance from centroid)
            double max_distance = 0.0;
            for (const auto& detection : cluster.detections) {
                double dist = (detection.position.toEigen() - centroid_sum).norm();
                max_distance = std::max(max_distance, dist);
            }
            cluster.cluster_radius = max_distance;
            
            // Calculate cluster density (detections per unit volume)
            if (cluster.cluster_radius > 0) {
                double volume = (4.0/3.0) * M_PI * std::pow(cluster.cluster_radius, 3);
                cluster.cluster_density = cluster.detection_count / volume;
            } else {
                cluster.cluster_density = std::numeric_limits<double>::infinity();
            }
            
            // Calculate covariance matrix
            if (cluster.detection_count > 1) {
                common::Matrix3d covariance = common::Matrix3d::Zero();
                for (const auto& detection : cluster.detections) {
                    common::Vector3d diff = detection.position.toEigen() - centroid_sum;
                    covariance += diff * diff.transpose();
                }
                covariance /= static_cast<double>(cluster.detection_count - 1);
                cluster.covariance = covariance;
            } else {
                cluster.covariance = common::Matrix3d::Identity();
            }
        }
    }
    
    result = clusters;
    
    return result;
}

double HierarchicalClustering::calculateDistance(
    const common::Detection& detection1,
    const common::Detection& detection2) const {
    
    auto pos1 = detection1.position.toEigen();
    auto pos2 = detection2.position.toEigen();
    
    return (pos1 - pos2).norm();
}

HierarchicalClustering::LinkageCriterion 
HierarchicalClustering::stringToLinkageCriterion(const std::string& criterion_str) const {
    if (criterion_str == "single") return LinkageCriterion::SINGLE;
    if (criterion_str == "complete") return LinkageCriterion::COMPLETE;
    if (criterion_str == "average") return LinkageCriterion::AVERAGE;
    if (criterion_str == "ward") return LinkageCriterion::WARD;
    return LinkageCriterion::AVERAGE;  // Default
}

void HierarchicalClustering::getLeafIndices(const std::shared_ptr<ClusterNode>& node,
                                           std::vector<int>& leaf_indices) const {
    if (!node) return;
    
    if (!node->left && !node->right) {
        // Leaf node
        leaf_indices.insert(leaf_indices.end(), 
                           node->point_indices.begin(), 
                           node->point_indices.end());
    } else {
        // Internal node - recurse
        getLeafIndices(node->left, leaf_indices);
        getLeafIndices(node->right, leaf_indices);
    }
}

bool HierarchicalClustering::configure(const common::AlgorithmConfig& config) {
    if (!validateConfiguration(config)) {
        return false;
    }
    
    config_ = config;
    
    // Update linkage criterion
    int linkage_int = static_cast<int>(config_.parameters.at("linkage_criterion"));
    linkage_criterion_ = static_cast<LinkageCriterion>(linkage_int);
    
    target_num_clusters_ = static_cast<int>(config_.parameters.at("num_clusters"));
    distance_threshold_ = config_.parameters.at("distance_threshold");
    
    return true;
}

common::AlgorithmConfig HierarchicalClustering::getConfiguration() const {
    return config_;
}

std::unordered_map<std::string, std::string> HierarchicalClustering::getParameterDescriptions() const {
    return {
        {"linkage_criterion", "Linkage criterion: 0=single, 1=complete, 2=average, 3=ward"},
        {"num_clusters", "Target number of clusters when not using distance threshold"},
        {"distance_threshold", "Distance threshold for cutting dendrogram"},
        {"use_threshold", "Use distance threshold (1) or number of clusters (0)"}
    };
}

bool HierarchicalClustering::validateConfiguration(const common::AlgorithmConfig& config) const {
    std::vector<std::string> required_params = {"linkage_criterion", "num_clusters", "distance_threshold"};
    
    for (const auto& param : required_params) {
        if (config.parameters.find(param) == config.parameters.end()) {
            return false;
        }
    }
    
    int linkage = static_cast<int>(config.parameters.at("linkage_criterion"));
    int num_clusters = static_cast<int>(config.parameters.at("num_clusters"));
    double threshold = config.parameters.at("distance_threshold");
    
    return linkage >= 0 && linkage <= 3 && num_clusters > 0 && threshold > 0.0;
}

common::ClusteringAlgorithm HierarchicalClustering::getAlgorithmType() const {
    return common::ClusteringAlgorithm::HIERARCHICAL;
}

std::string HierarchicalClustering::getName() const {
    return "Hierarchical Clustering";
}

std::string HierarchicalClustering::getVersion() const {
    return "1.0.0";
}

void HierarchicalClustering::reset() {
    dendrogram_root_.reset();
    all_nodes_.clear();
    distance_matrix_.clear();
    performance_metrics_ = common::PerformanceMetrics{};
}

common::PerformanceMetrics HierarchicalClustering::getPerformanceMetrics() const {
    return performance_metrics_;
}

} // namespace clustering
} // namespace processing
} // namespace radar