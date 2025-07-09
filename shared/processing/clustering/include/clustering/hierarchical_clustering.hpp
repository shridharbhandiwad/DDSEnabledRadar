#pragma once

#include "interfaces/i_clustering.hpp"
#include <vector>
#include <memory>

namespace radar {
namespace processing {
namespace clustering {

/**
 * @brief Hierarchical Clustering algorithm implementation
 * 
 * This class implements hierarchical clustering which builds a tree of clusters
 * by iteratively merging the closest pairs of clusters. Supports multiple
 * linkage criteria and provides dendrograms for cluster analysis.
 */
class HierarchicalClustering : public interfaces::IClustering {
private:
    /**
     * @brief Linkage criterion for determining cluster distances
     */
    enum class LinkageCriterion {
        SINGLE,    // Minimum distance between any two points
        COMPLETE,  // Maximum distance between any two points
        AVERAGE,   // Average distance between all point pairs
        WARD       // Minimize within-cluster variance
    };

    /**
     * @brief Node in the hierarchical clustering dendrogram
     */
    struct ClusterNode {
        int id{-1};                           // Unique identifier for this node
        std::vector<int> point_indices;       // Original point indices in this cluster
        std::shared_ptr<ClusterNode> left;    // Left child (smaller cluster)
        std::shared_ptr<ClusterNode> right;   // Right child (smaller cluster)
        double merge_distance{0.0};           // Distance at which this cluster was formed
        int merge_level{0};                   // Level in the dendrogram (0 = leaf)
        
        ClusterNode() = default;
        ClusterNode(int point_idx) : id(point_idx) {
            point_indices.push_back(point_idx);
        }
    };

    /**
     * @brief Distance matrix entry
     */
    struct DistanceEntry {
        int cluster1_id;
        int cluster2_id;
        double distance;
        
        DistanceEntry(int c1, int c2, double d) 
            : cluster1_id(c1), cluster2_id(c2), distance(d) {}
    };

    common::AlgorithmConfig config_;
    common::PerformanceMetrics performance_metrics_;
    
    // Hierarchical clustering parameters
    LinkageCriterion linkage_criterion_;
    int target_num_clusters_;
    double distance_threshold_;
    
    // Dendrogram structure
    std::shared_ptr<ClusterNode> dendrogram_root_;
    std::vector<std::shared_ptr<ClusterNode>> all_nodes_;
    
    // Distance matrix
    std::vector<std::vector<double>> distance_matrix_;

public:
    HierarchicalClustering();
    ~HierarchicalClustering() override = default;

    // IClustering interface implementation
    interfaces::ClusteringResult cluster(
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
     * @brief Get the dendrogram root node
     * @return Shared pointer to root node
     */
    std::shared_ptr<ClusterNode> getDendrogramRoot() const { return dendrogram_root_; }

    /**
     * @brief Cut dendrogram at specified distance threshold
     * @param threshold Distance threshold for cutting
     * @return Vector of cluster assignments
     */
    std::vector<int> cutDendrogram(double threshold) const;

    /**
     * @brief Cut dendrogram to get specified number of clusters
     * @param num_clusters Desired number of clusters
     * @return Vector of cluster assignments
     */
    std::vector<int> cutDendrogramToNClusters(int num_clusters) const;

    /**
     * @brief Get merge distances for all internal nodes
     * @return Vector of merge distances in order
     */
    std::vector<double> getMergeDistances() const;

private:
    /**
     * @brief Build initial distance matrix between all detection pairs
     * @param detections Input detections
     */
    void buildDistanceMatrix(const std::vector<common::Detection>& detections);

    /**
     * @brief Perform agglomerative clustering
     * @param detections Input detections
     * @return Root node of the dendrogram
     */
    std::shared_ptr<ClusterNode> performAgglomerativeClustering(
        const std::vector<common::Detection>& detections);

    /**
     * @brief Calculate distance between two clusters
     * @param cluster1 First cluster node
     * @param cluster2 Second cluster node
     * @param detections Original detections for reference
     * @return Distance between clusters
     */
    double calculateClusterDistance(
        const std::shared_ptr<ClusterNode>& cluster1,
        const std::shared_ptr<ClusterNode>& cluster2,
        const std::vector<common::Detection>& detections) const;

    /**
     * @brief Calculate single linkage distance (minimum)
     * @param cluster1 First cluster node
     * @param cluster2 Second cluster node
     * @return Minimum distance between any two points
     */
    double calculateSingleLinkage(
        const std::shared_ptr<ClusterNode>& cluster1,
        const std::shared_ptr<ClusterNode>& cluster2) const;

    /**
     * @brief Calculate complete linkage distance (maximum)
     * @param cluster1 First cluster node
     * @param cluster2 Second cluster node
     * @return Maximum distance between any two points
     */
    double calculateCompleteLinkage(
        const std::shared_ptr<ClusterNode>& cluster1,
        const std::shared_ptr<ClusterNode>& cluster2) const;

    /**
     * @brief Calculate average linkage distance
     * @param cluster1 First cluster node
     * @param cluster2 Second cluster node
     * @return Average distance between all point pairs
     */
    double calculateAverageLinkage(
        const std::shared_ptr<ClusterNode>& cluster1,
        const std::shared_ptr<ClusterNode>& cluster2) const;

    /**
     * @brief Calculate Ward linkage distance
     * @param cluster1 First cluster node
     * @param cluster2 Second cluster node
     * @param detections Original detections for centroid calculation
     * @return Ward distance based on variance increase
     */
    double calculateWardLinkage(
        const std::shared_ptr<ClusterNode>& cluster1,
        const std::shared_ptr<ClusterNode>& cluster2,
        const std::vector<common::Detection>& detections) const;

    /**
     * @brief Find the closest pair of clusters
     * @param active_clusters Current active clusters
     * @param detections Original detections
     * @return Pair of indices for closest clusters
     */
    std::pair<int, int> findClosestClusters(
        const std::vector<std::shared_ptr<ClusterNode>>& active_clusters,
        const std::vector<common::Detection>& detections) const;

    /**
     * @brief Merge two clusters into a new cluster
     * @param cluster1 First cluster to merge
     * @param cluster2 Second cluster to merge
     * @param merge_distance Distance at which merge occurs
     * @param node_id Unique ID for the new node
     * @return New merged cluster node
     */
    std::shared_ptr<ClusterNode> mergeClusters(
        std::shared_ptr<ClusterNode> cluster1,
        std::shared_ptr<ClusterNode> cluster2,
        double merge_distance,
        int node_id) const;

    /**
     * @brief Calculate centroid of a cluster
     * @param cluster Cluster node
     * @param detections Original detections
     * @return Centroid position
     */
    common::Vector3d calculateClusterCentroid(
        const std::shared_ptr<ClusterNode>& cluster,
        const std::vector<common::Detection>& detections) const;

    /**
     * @brief Calculate within-cluster sum of squares
     * @param cluster Cluster node
     * @param detections Original detections
     * @return Sum of squared distances to centroid
     */
    double calculateWithinClusterSS(
        const std::shared_ptr<ClusterNode>& cluster,
        const std::vector<common::Detection>& detections) const;

    /**
     * @brief Extract flat clusters from dendrogram at given threshold
     * @param node Current node in traversal
     * @param threshold Distance threshold
     * @param cluster_id Current cluster ID being assigned
     * @param assignments Output cluster assignments
     */
    void extractClustersAtThreshold(
        const std::shared_ptr<ClusterNode>& node,
        double threshold,
        int& cluster_id,
        std::vector<int>& assignments) const;

    /**
     * @brief Extract flat clusters to get specified number of clusters
     * @param node Current node in traversal
     * @param target_clusters Target number of clusters
     * @param current_clusters Current number of clusters found
     * @param cluster_id Current cluster ID being assigned
     * @param assignments Output cluster assignments
     */
    void extractNClusters(
        const std::shared_ptr<ClusterNode>& node,
        int target_clusters,
        int& current_clusters,
        int& cluster_id,
        std::vector<int>& assignments) const;

    /**
     * @brief Create final clustering result from assignments
     * @param assignments Cluster assignments for each detection
     * @param detections Original detections
     * @return Clustering result
     */
    interfaces::ClusteringResult createClusteringResult(
        const std::vector<int>& assignments,
        const std::vector<common::Detection>& detections) const;

    /**
     * @brief Calculate distance between two detections
     * @param detection1 First detection
     * @param detection2 Second detection
     * @return Euclidean distance
     */
    double calculateDistance(
        const common::Detection& detection1,
        const common::Detection& detection2) const;

    /**
     * @brief Convert linkage criterion string to enum
     * @param criterion_str String representation
     * @return LinkageCriterion enum value
     */
    LinkageCriterion stringToLinkageCriterion(const std::string& criterion_str) const;

    /**
     * @brief Get all leaf nodes (original points) from a cluster
     * @param node Cluster node
     * @param leaf_indices Output vector of leaf indices
     */
    void getLeafIndices(const std::shared_ptr<ClusterNode>& node,
                       std::vector<int>& leaf_indices) const;
};

} // namespace clustering
} // namespace processing
} // namespace radar