#pragma once

#include "interfaces/i_association.hpp"
#include <vector>
#include <memory>

namespace radar {
namespace processing {
namespace association {

/**
 * @brief Global Nearest Neighbor (GNN) association algorithm
 * 
 * This class implements the Global Nearest Neighbor algorithm which finds
 * the optimal global assignment between tracks and detections by minimizing
 * the total assignment cost using the Hungarian algorithm.
 */
class GNN : public interfaces::IAssociation {
private:
    common::AlgorithmConfig config_;
    common::PerformanceMetrics performance_metrics_;
    
    // Cost matrix for assignment optimization
    mutable common::MatrixXd cost_matrix_;
    
public:
    GNN();
    ~GNN() override = default;

    // IAssociation interface implementation
    std::vector<common::Association> associate(
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) override;

    std::vector<interfaces::AssociationHypothesis> getHypotheses(
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) override;

    double calculateAssociationCost(
        const common::Track& track,
        const common::Detection& detection,
        const interfaces::AssociationGate& gate) const override;

    bool isWithinGate(
        const common::Track& track,
        const common::Detection& detection,
        const interfaces::AssociationGate& gate) const override;

    bool configure(const common::AlgorithmConfig& config) override;
    common::AlgorithmConfig getConfiguration() const override;
    
    std::unordered_map<std::string, std::string> getParameterDescriptions() const override;
    bool validateConfiguration(const common::AlgorithmConfig& config) const override;
    
    common::AssociationAlgorithm getAlgorithmType() const override;
    std::string getName() const override;
    std::string getVersion() const override;
    
    void reset() override;
    common::PerformanceMetrics getPerformanceMetrics() const override;
    
    void setDistanceFunction(
        std::function<double(const common::Track&, const common::Detection&)> distance_func) override;

protected:
    // Cost matrix building (inherited from IAssociation)
    common::MatrixXd buildCostMatrix(
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) const override;

private:
    std::function<double(const common::Track&, const common::Detection&)> custom_distance_func_;

    /**
     * @brief Solve the assignment problem using Hungarian algorithm
     * @param tracks Input tracks
     * @param detections Input detections
     * @param gate Association gate parameters
     * @return Vector of associations
     */
    std::vector<common::Association> solveAssignmentProblem(
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) const;

    /**
     * @brief Create association objects from assignment results
     * @param tracks Input tracks
     * @param detections Input detections
     * @param assignments Assignment vector (track_index -> detection_index)
     * @param gate Association gate parameters
     * @return Vector of associations
     */
    std::vector<common::Association> createAssociations(
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const std::vector<int>& assignments,
        const interfaces::AssociationGate& gate) const;

    /**
     * @brief Calculate innovation vector between track and detection
     * @param track Track for comparison
     * @param detection Detection for comparison
     * @return Innovation vector
     */
    common::Vector3d calculateInnovation(
        const common::Track& track,
        const common::Detection& detection) const;

    /**
     * @brief Calculate innovation covariance matrix
     * @param track Track for comparison
     * @param detection Detection for comparison
     * @return Innovation covariance matrix
     */
    common::Matrix3d calculateInnovationCovariance(
        const common::Track& track,
        const common::Detection& detection) const;

    /**
     * @brief Calculate likelihood for track-detection pair
     * @param track Track for comparison
     * @param detection Detection for comparison
     * @param innovation Innovation vector
     * @param innovation_covariance Innovation covariance matrix
     * @return Likelihood value
     */
    double calculateLikelihood(
        const common::Track& track,
        const common::Detection& detection,
        const common::Vector3d& innovation,
        const common::Matrix3d& innovation_covariance) const;

    /**
     * @brief Apply assignment constraints (one-to-one mapping)
     * @param cost_matrix Cost matrix to modify
     * @param gate Association gate parameters
     */
    void applyAssignmentConstraints(
        common::MatrixXd& cost_matrix,
        const interfaces::AssociationGate& gate) const;

    /**
     * @brief Validate assignment results
     * @param assignments Assignment vector
     * @param tracks Input tracks
     * @param detections Input detections
     * @param gate Association gate parameters
     * @return true if assignments are valid
     */
    bool validateAssignments(
        const std::vector<int>& assignments,
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) const;

    /**
     * @brief Solve assignment problem using Hungarian algorithm
     * @param cost_matrix Cost matrix for assignment
     * @return Assignment vector (track_index -> detection_index)
     */
    std::vector<int> solveAssignment(const common::MatrixXd& cost_matrix) const;
};

} // namespace association
} // namespace processing
} // namespace radar