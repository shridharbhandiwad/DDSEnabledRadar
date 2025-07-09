#pragma once

#include "interfaces/i_association.hpp"
#include <vector>
#include <memory>
#include <unordered_map>

namespace radar {
namespace processing {
namespace association {

/**
 * @brief Joint Probabilistic Data Association (JPDA) algorithm
 * 
 * This class implements the JPDA algorithm which handles uncertain data association
 * by considering all feasible associations and computing their probabilities.
 * It's particularly effective in cluttered environments with false alarms.
 */
class JPDA : public interfaces::IAssociation {
private:
    /**
     * @brief Association event structure for JPDA
     */
    struct AssociationEvent {
        std::vector<std::pair<int, int>> associations; // (track_id, detection_id) pairs
        double probability{0.0};
        double likelihood{0.0};
        bool is_feasible{true};
        
        AssociationEvent() = default;
        AssociationEvent(const std::vector<std::pair<int, int>>& assoc) : associations(assoc) {}
    };

    /**
     * @brief Track-detection association probability
     */
    struct AssociationProbability {
        int track_id;
        int detection_id;
        double probability{0.0};
        double likelihood{0.0};
        double distance{0.0};
        bool is_within_gate{false};
    };

    common::AlgorithmConfig config_;
    common::PerformanceMetrics performance_metrics_;
    
    // JPDA-specific parameters
    double false_alarm_rate_;
    double detection_probability_;
    double gate_probability_;
    std::unordered_map<int, double> track_existence_probabilities_;

public:
    JPDA();
    ~JPDA() override = default;

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

    /**
     * @brief Set track existence probability
     * @param track_id Track identifier
     * @param probability Existence probability [0,1]
     */
    void setTrackExistenceProbability(int track_id, double probability);

protected:
    // Cost matrix building (inherited from IAssociation)
    common::MatrixXd buildCostMatrix(
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) const override;

private:
    std::function<double(const common::Track&, const common::Detection&)> custom_distance_func_;

    /**
     * @brief Generate all feasible association events
     * @param tracks Input tracks
     * @param detections Input detections
     * @param gate Association gate parameters
     * @return Vector of feasible association events
     */
    std::vector<AssociationEvent> generateAssociationEvents(
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) const;

    /**
     * @brief Calculate probability for an association event
     * @param event Association event
     * @param tracks Input tracks
     * @param detections Input detections
     * @param gate Association gate parameters
     * @return Event probability
     */
    double calculateEventProbability(
        const AssociationEvent& event,
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) const;

    /**
     * @brief Calculate marginal association probabilities
     * @param events All association events with probabilities
     * @param tracks Input tracks
     * @param detections Input detections
     * @return Vector of association probabilities
     */
    std::vector<AssociationProbability> calculateMarginalProbabilities(
        const std::vector<AssociationEvent>& events,
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections) const;

    /**
     * @brief Generate all possible assignments recursively
     * @param tracks Input tracks
     * @param detections Input detections
     * @param gate Association gate parameters
     * @param current_assignment Current partial assignment
     * @param track_index Current track being processed
     * @param used_detections Set of already used detection indices
     * @param all_events Output vector for all generated events
     */
    void generateAssignmentsRecursive(
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate,
        std::vector<std::pair<int, int>>& current_assignment,
        int track_index,
        std::vector<bool>& used_detections,
        std::vector<AssociationEvent>& all_events) const;

    /**
     * @brief Check if an association event is feasible
     * @param event Association event to check
     * @param tracks Input tracks
     * @param detections Input detections
     * @param gate Association gate parameters
     * @return true if event is feasible
     */
    bool isEventFeasible(
        const AssociationEvent& event,
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) const;

    /**
     * @brief Calculate likelihood for specific track-detection pair
     * @param track_id Track identifier
     * @param detection_id Detection identifier
     * @param tracks Input tracks
     * @param detections Input detections
     * @param gate Association gate parameters
     * @return Likelihood value
     */
    double calculatePairLikelihood(
        int track_id,
        int detection_id,
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) const;

    /**
     * @brief Calculate clutter/false alarm probability for detection
     * @param detection_id Detection identifier
     * @param detections Input detections
     * @param gate Association gate parameters
     * @return False alarm probability
     */
    double calculateClutterProbability(
        int detection_id,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) const;

    /**
     * @brief Calculate missed detection probability for track
     * @param track_id Track identifier
     * @param tracks Input tracks
     * @return Missed detection probability
     */
    double calculateMissedDetectionProbability(
        int track_id,
        const std::vector<common::Track>& tracks) const;

    /**
     * @brief Create final associations from marginal probabilities
     * @param marginal_probs Marginal association probabilities
     * @param tracks Input tracks
     * @param detections Input detections
     * @param gate Association gate parameters
     * @return Vector of associations
     */
    std::vector<common::Association> createAssociationsFromProbabilities(
        const std::vector<AssociationProbability>& marginal_probs,
        const std::vector<common::Track>& tracks,
        const std::vector<common::Detection>& detections,
        const interfaces::AssociationGate& gate) const;

    /**
     * @brief Prune low-probability events to reduce computational load
     * @param events Vector of association events to prune
     * @param probability_threshold Minimum probability threshold
     */
    void pruneEvents(std::vector<AssociationEvent>& events, 
                    double probability_threshold) const;

    /**
     * @brief Normalize event probabilities to sum to 1.0
     * @param events Vector of association events to normalize
     */
    void normalizeEventProbabilities(std::vector<AssociationEvent>& events) const;
};

} // namespace association
} // namespace processing
} // namespace radar