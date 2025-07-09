#pragma once

#include "interfaces/i_association.hpp"
#include <vector>
#include <memory>

namespace radar {
namespace processing {
namespace association {

/**
 * @brief Nearest Neighbor (NN) association algorithm
 * 
 * This class implements the simple Nearest Neighbor algorithm which performs
 * greedy assignment of tracks to detections based on distance metrics.
 * It's fast and effective for low-clutter environments.
 */
class NearestNeighbor : public interfaces::IAssociation {
private:
    common::AlgorithmConfig config_;
    common::PerformanceMetrics performance_metrics_;
    
    /**
     * @brief Association candidate structure
     */
    struct AssociationCandidate {
        int track_index;
        int detection_index;
        double distance;
        double likelihood;
        bool is_valid;
        
        AssociationCandidate(int t_idx, int d_idx, double dist, double like = 0.0) 
            : track_index(t_idx), detection_index(d_idx), distance(dist), 
              likelihood(like), is_valid(true) {}
    };

public:
    NearestNeighbor();
    ~NearestNeighbor() override = default;

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
     * @brief Find all valid association candidates within gates
     * @param tracks Input tracks
     * @param detections Input detections
     * @param gate Association gate parameters
     * @return Vector of valid association candidates
     */
    std::vector<AssociationCandidate> findValidCandidates(
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) const;

    /**
     * @brief Perform greedy nearest neighbor assignment
     * @param candidates Valid association candidates
     * @param tracks Input tracks
     * @param detections Input detections
     * @param gate Association gate parameters
     * @return Vector of associations
     */
    std::vector<common::Association> performGreedyAssignment(
        std::vector<AssociationCandidate>& candidates,
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) const;

    /**
     * @brief Create association object from candidate
     * @param candidate Association candidate
     * @param tracks Input tracks
     * @param detections Input detections
     * @param gate Association gate parameters
     * @return Association object
     */
    common::Association createAssociation(
        const AssociationCandidate& candidate,
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) const;

    /**
     * @brief Calculate innovation vector for track-detection pair
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
     * @return Likelihood value
     */
    double calculateLikelihood(
        const common::Track& track,
        const common::Detection& detection) const;

    /**
     * @brief Sort candidates by distance (ascending)
     * @param candidates Candidates to sort
     */
    void sortCandidatesByDistance(std::vector<AssociationCandidate>& candidates) const;

    /**
     * @brief Remove conflicting associations
     * @param candidates Candidates to process
     * @param assigned_tracks Set of already assigned track indices
     * @param assigned_detections Set of already assigned detection indices
     */
    void removeConflicts(
        std::vector<AssociationCandidate>& candidates,
        const std::vector<bool>& assigned_tracks,
        const std::vector<bool>& assigned_detections) const;
};

} // namespace association
} // namespace processing
} // namespace radar