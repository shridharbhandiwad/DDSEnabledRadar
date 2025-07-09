#pragma once

#include "common/types.hpp"
#include <memory>
#include <vector>
#include <functional>

namespace radar {
namespace interfaces {

/**
 * @brief Association hypothesis for track-detection pairing
 */
struct AssociationHypothesis {
    std::vector<common::Association> associations;
    double probability{0.0};
    double likelihood{0.0};
    bool is_feasible{true};
};

/**
 * @brief Gate parameters for association validation
 */
struct AssociationGate {
    double mahalanobis_threshold{9.21};  // Chi-square threshold for 3D, 99% confidence
    double euclidean_threshold{100.0};   // Euclidean distance threshold in meters
    bool use_mahalanobis{true};
    bool use_velocity_gating{false};
    double velocity_threshold{50.0};     // m/s
};

/**
 * @brief Abstract interface for data association algorithms
 * 
 * This interface defines the contract for association algorithms used in
 * multi-target tracking. Implementations include Global Nearest Neighbor (GNN),
 * Joint Probabilistic Data Association (JPDA), Multiple Hypothesis Tracking (MHT),
 * and Nearest Neighbor (NN).
 */
class IAssociation {
public:
    virtual ~IAssociation() = default;

    /**
     * @brief Perform data association between tracks and detections
     * @param tracks Current active tracks
     * @param detections New detections to associate
     * @param gate Association gate parameters
     * @return Vector of association results
     */
    virtual std::vector<common::Association> associate(
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const AssociationGate& gate) = 0;

    /**
     * @brief Get multiple association hypotheses (for JPDA, MHT)
     * @param tracks Current active tracks
     * @param detections New detections to associate
     * @param gate Association gate parameters
     * @return Vector of association hypotheses
     */
    virtual std::vector<AssociationHypothesis> getHypotheses(
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const AssociationGate& gate) = 0;

    /**
     * @brief Calculate association cost/distance between track and detection
     * @param track Track to evaluate
     * @param detection Detection to evaluate
     * @param gate Association gate parameters
     * @return Association cost (lower is better)
     */
    virtual double calculateAssociationCost(
        const common::Track& track,
        const common::Detection& detection,
        const AssociationGate& gate) const = 0;

    /**
     * @brief Check if track-detection pair is within association gate
     * @param track Track to check
     * @param detection Detection to check
     * @param gate Association gate parameters
     * @return true if association is valid
     */
    virtual bool isWithinGate(
        const common::Track& track,
        const common::Detection& detection,
        const AssociationGate& gate) const = 0;

    /**
     * @brief Configure the association algorithm
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
    virtual common::AssociationAlgorithm getAlgorithmType() const = 0;

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
     * @brief Get performance metrics from last association operation
     * @return Performance metrics
     */
    virtual common::PerformanceMetrics getPerformanceMetrics() const = 0;

    /**
     * @brief Set custom distance function
     * @param distance_func Custom distance function
     */
    virtual void setDistanceFunction(
        std::function<double(const common::Track&, const common::Detection&)> distance_func) = 0;

protected:
    // Common utility functions for derived classes
    
    /**
     * @brief Calculate Mahalanobis distance between track prediction and detection
     * @param track Track with predicted state
     * @param detection Detection to compare
     * @return Mahalanobis distance
     */
    static double calculateMahalanobisDistance(
        const common::Track& track,
        const common::Detection& detection);

    /**
     * @brief Calculate Euclidean distance between track prediction and detection
     * @param track Track with predicted state
     * @param detection Detection to compare
     * @return Euclidean distance
     */
    static double calculateEuclideanDistance(
        const common::Track& track,
        const common::Detection& detection);

    /**
     * @brief Build assignment matrix for optimization algorithms
     * @param tracks Input tracks
     * @param detections Input detections
     * @param gate Association gate parameters
     * @return Cost matrix (tracks x detections)
     */
    virtual common::MatrixXd buildCostMatrix(
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const AssociationGate& gate) const = 0;

    /**
     * @brief Solve assignment problem using Hungarian algorithm or similar
     * @param cost_matrix Cost matrix to solve
     * @return Assignment vector (track_index -> detection_index, -1 for no assignment)
     */
    static std::vector<int> solveAssignment(const common::MatrixXd& cost_matrix);
};

/**
 * @brief Factory interface for creating association algorithms
 */
class IAssociationFactory {
public:
    virtual ~IAssociationFactory() = default;

    /**
     * @brief Create an association algorithm instance
     * @param algorithm Algorithm type to create
     * @param config Initial configuration
     * @return Unique pointer to association algorithm
     */
    virtual std::unique_ptr<IAssociation> create(
        common::AssociationAlgorithm algorithm,
        const common::AlgorithmConfig& config = {}) = 0;

    /**
     * @brief Get list of supported algorithms
     * @return Vector of supported algorithm types
     */
    virtual std::vector<common::AssociationAlgorithm> getSupportedAlgorithms() const = 0;

    /**
     * @brief Check if algorithm is supported
     * @param algorithm Algorithm type to check
     * @return true if algorithm is supported
     */
    virtual bool isSupported(common::AssociationAlgorithm algorithm) const = 0;

    /**
     * @brief Get default configuration for an algorithm
     * @param algorithm Algorithm type
     * @return Default configuration
     */
    virtual common::AlgorithmConfig getDefaultConfiguration(
        common::AssociationAlgorithm algorithm) const = 0;

    /**
     * @brief Get default gate parameters for an algorithm
     * @param algorithm Algorithm type
     * @return Default gate parameters
     */
    virtual AssociationGate getDefaultGate(common::AssociationAlgorithm algorithm) const = 0;
};

} // namespace interfaces
} // namespace radar