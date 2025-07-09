#include "association/jpda.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>

namespace radar {
namespace processing {
namespace association {

JPDA::JPDA() {
    config_.name = "JPDA";
    config_.type = "JPDA";
    config_.parameters["false_alarm_rate"] = 0.01;      // False alarm rate per unit volume
    config_.parameters["detection_probability"] = 0.9;  // Probability of detection
    config_.parameters["gate_probability"] = 0.99;      // Gate probability
    config_.parameters["max_hypotheses"] = 1000.0;      // Maximum number of hypotheses
    config_.parameters["probability_threshold"] = 1e-6; // Minimum probability threshold
    config_.parameters["clutter_density"] = 1e-6;       // Clutter density (per unit volume)
    config_.parameters["track_existence_prob"] = 0.9;   // Default track existence probability
    config_.enabled = true;
    
    // Initialize JPDA parameters
    false_alarm_rate_ = config_.parameters["false_alarm_rate"];
    detection_probability_ = config_.parameters["detection_probability"];
    gate_probability_ = config_.parameters["gate_probability"];
}

std::vector<common::Association> JPDA::associate(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::vector<common::Association> associations;
    
    if (tracks.empty() || detections.empty()) {
        return associations;
    }
    
    // Generate all feasible association events
    auto events = generateAssociationEvents(tracks, detections, gate);
    
    // Calculate probabilities for each event
    for (auto& event : events) {
        event.probability = calculateEventProbability(event, tracks, detections, gate);
    }
    
    // Prune low-probability events
    double prob_threshold = config_.parameters.at("probability_threshold");
    pruneEvents(events, prob_threshold);
    
    // Normalize probabilities
    normalizeEventProbabilities(events);
    
    // Calculate marginal association probabilities
    auto marginal_probs = calculateMarginalProbabilities(events, tracks, detections);
    
    // Create final associations
    associations = createAssociationsFromProbabilities(marginal_probs, tracks, detections, gate);
    
    // Update performance metrics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end_time - start_time);
    
    performance_metrics_.timestamp = end_time;
    performance_metrics_.processing_time_ms = duration.count();
    performance_metrics_.total_detections = detections.size();
    performance_metrics_.active_tracks = tracks.size();
    
    return associations;
}

std::vector<interfaces::AssociationHypothesis> JPDA::getHypotheses(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) {
    
    std::vector<interfaces::AssociationHypothesis> hypotheses;
    
    // Generate all feasible association events
    auto events = generateAssociationEvents(tracks, detections, gate);
    
    // Calculate probabilities for each event
    for (auto& event : events) {
        event.probability = calculateEventProbability(event, tracks, detections, gate);
    }
    
    // Prune and normalize
    double prob_threshold = config_.parameters.at("probability_threshold");
    pruneEvents(events, prob_threshold);
    normalizeEventProbabilities(events);
    
    // Convert events to hypotheses
    for (const auto& event : events) {
        if (event.probability > prob_threshold) {
            interfaces::AssociationHypothesis hypothesis;
            hypothesis.probability = event.probability;
            hypothesis.likelihood = event.likelihood;
            hypothesis.is_feasible = event.is_feasible;
            
            // Create associations for this hypothesis
            for (const auto& assoc_pair : event.associations) {
                if (assoc_pair.first >= 0 && assoc_pair.second >= 0 &&
                    assoc_pair.first < static_cast<int>(tracks.size()) &&
                    assoc_pair.second < static_cast<int>(detections.size())) {
                    
                    common::Association association;
                    association.track_id = tracks[assoc_pair.first].id;
                    association.detection_id = detections[assoc_pair.second].id;
                    association.distance = calculateMahalanobisDistance(
                        tracks[assoc_pair.first], detections[assoc_pair.second]);
                    association.score = event.probability;
                    association.likelihood = calculatePairLikelihood(
                        assoc_pair.first, assoc_pair.second, tracks, detections, gate);
                    association.is_valid = true;
                    association.algorithm = common::AssociationAlgorithm::JPDA;
                    
                    hypothesis.associations.push_back(association);
                }
            }
            
            hypotheses.push_back(hypothesis);
        }
    }
    
    return hypotheses;
}

std::vector<JPDA::AssociationEvent> JPDA::generateAssociationEvents(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    std::vector<AssociationEvent> all_events;
    std::vector<std::pair<int, int>> current_assignment;
    std::vector<bool> used_detections(detections.size(), false);
    
    const int max_hypotheses = static_cast<int>(config_.parameters.at("max_hypotheses"));
    
    // Generate all possible assignments recursively
    generateAssignmentsRecursive(tracks, detections, gate, current_assignment, 
                                0, used_detections, all_events);
    
    // Limit number of hypotheses for computational efficiency
    if (all_events.size() > static_cast<size_t>(max_hypotheses)) {
        // Keep only the most promising events based on initial likelihood
        std::partial_sort(all_events.begin(), all_events.begin() + max_hypotheses, 
                         all_events.end(),
                         [](const AssociationEvent& a, const AssociationEvent& b) {
                             return a.likelihood > b.likelihood;
                         });
        all_events.resize(max_hypotheses);
    }
    
    return all_events;
}

void JPDA::generateAssignmentsRecursive(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate,
    std::vector<std::pair<int, int>>& current_assignment,
    int track_index,
    std::vector<bool>& used_detections,
    std::vector<AssociationEvent>& all_events) const {
    
    if (track_index >= static_cast<int>(tracks.size())) {
        // Complete assignment reached, create event
        AssociationEvent event(current_assignment);
        if (isEventFeasible(event, tracks, detections, gate)) {
            all_events.push_back(event);
        }
        return;
    }
    
    // Option 1: Track is not detected (missed detection)
    generateAssignmentsRecursive(tracks, detections, gate, current_assignment,
                                track_index + 1, used_detections, all_events);
    
    // Option 2: Track is associated with a detection
    for (int det_index = 0; det_index < static_cast<int>(detections.size()); ++det_index) {
        if (!used_detections[det_index] && 
            isWithinGate(tracks[track_index], detections[det_index], gate)) {
            
            // Add this association
            current_assignment.emplace_back(track_index, det_index);
            used_detections[det_index] = true;
            
            // Recurse to next track
            generateAssignmentsRecursive(tracks, detections, gate, current_assignment,
                                        track_index + 1, used_detections, all_events);
            
            // Backtrack
            current_assignment.pop_back();
            used_detections[det_index] = false;
        }
    }
}

double JPDA::calculateEventProbability(
    const AssociationEvent& event,
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    double probability = 1.0;
    std::vector<bool> detection_assigned(detections.size(), false);
    std::vector<bool> track_assigned(tracks.size(), false);
    
    // Process associations in the event
    for (const auto& assoc : event.associations) {
        int track_idx = assoc.first;
        int det_idx = assoc.second;
        
        if (track_idx >= 0 && det_idx >= 0) {
            // Valid association
            double pair_likelihood = calculatePairLikelihood(track_idx, det_idx, tracks, detections, gate);
            probability *= detection_probability_ * pair_likelihood;
            
            detection_assigned[det_idx] = true;
            track_assigned[track_idx] = true;
        }
    }
    
    // Account for missed detections
    for (int i = 0; i < static_cast<int>(tracks.size()); ++i) {
        if (!track_assigned[i]) {
            probability *= calculateMissedDetectionProbability(i, tracks);
        }
    }
    
    // Account for false alarms (unassigned detections)
    for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
        if (!detection_assigned[i]) {
            probability *= calculateClutterProbability(i, detections, gate);
        }
    }
    
    // Account for track existence probabilities
    for (int i = 0; i < static_cast<int>(tracks.size()); ++i) {
        double existence_prob = config_.parameters.at("track_existence_prob");
        if (track_existence_probabilities_.find(tracks[i].id) != track_existence_probabilities_.end()) {
            existence_prob = track_existence_probabilities_.at(tracks[i].id);
        }
        
        if (track_assigned[i]) {
            probability *= existence_prob;
        } else {
            probability *= (1.0 - existence_prob);
        }
    }
    
    return std::max(probability, 1e-15); // Avoid numerical underflow
}

std::vector<JPDA::AssociationProbability> JPDA::calculateMarginalProbabilities(
    const std::vector<AssociationEvent>& events,
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections) const {
    
    std::vector<AssociationProbability> marginal_probs;
    
    // Initialize marginal probabilities for all valid track-detection pairs
    for (int i = 0; i < static_cast<int>(tracks.size()); ++i) {
        for (int j = 0; j < static_cast<int>(detections.size()); ++j) {
            AssociationProbability prob;
            prob.track_id = tracks[i].id;
            prob.detection_id = detections[j].id;
            prob.probability = 0.0;
            prob.likelihood = 0.0;
            prob.distance = calculateMahalanobisDistance(tracks[i], detections[j]);
            prob.is_within_gate = isWithinGate(tracks[i], detections[j], 
                                             interfaces::AssociationGate{}); // Use default gate for check
            marginal_probs.push_back(prob);
        }
    }
    
    // Sum probabilities over all events where each pair is associated
    for (const auto& event : events) {
        for (const auto& assoc : event.associations) {
            int track_idx = assoc.first;
            int det_idx = assoc.second;
            
            if (track_idx >= 0 && det_idx >= 0 &&
                track_idx < static_cast<int>(tracks.size()) &&
                det_idx < static_cast<int>(detections.size())) {
                
                // Find corresponding marginal probability entry
                auto it = std::find_if(marginal_probs.begin(), marginal_probs.end(),
                    [&](const AssociationProbability& p) {
                        return p.track_id == tracks[track_idx].id && 
                               p.detection_id == detections[det_idx].id;
                    });
                
                if (it != marginal_probs.end()) {
                    it->probability += event.probability;
                    it->likelihood += event.likelihood * event.probability;
                }
            }
        }
    }
    
    // Normalize likelihoods
    for (auto& prob : marginal_probs) {
        if (prob.probability > 0) {
            prob.likelihood /= prob.probability;
        }
    }
    
    return marginal_probs;
}

std::vector<common::Association> JPDA::createAssociationsFromProbabilities(
    const std::vector<AssociationProbability>& marginal_probs,
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    std::vector<common::Association> associations;
    
    // Create associations for pairs with probability above threshold
    double prob_threshold = config_.parameters.at("probability_threshold");
    
    for (const auto& prob : marginal_probs) {
        if (prob.probability > prob_threshold && prob.is_within_gate) {
            common::Association association;
            association.track_id = prob.track_id;
            association.detection_id = prob.detection_id;
            association.distance = prob.distance;
            association.score = prob.probability;
            association.likelihood = prob.likelihood;
            association.is_valid = true;
            association.algorithm = common::AssociationAlgorithm::JPDA;
            
            // Calculate innovation for this pair
            auto track_it = std::find_if(tracks.begin(), tracks.end(),
                [&](const common::Track& t) { return t.id == prob.track_id; });
            auto det_it = std::find_if(detections.begin(), detections.end(),
                [&](const common::Detection& d) { return d.id == prob.detection_id; });
                
            if (track_it != tracks.end() && det_it != detections.end()) {
                auto predicted_pos = track_it->predicted_state.getPosition();
                auto measured_pos = det_it->position.toEigen();
                association.innovation = measured_pos - predicted_pos;
                
                // Calculate innovation covariance
                auto prediction_cov = track_it->predicted_state.covariance.block<3, 3>(0, 0);
                auto measurement_cov = det_it->position_covariance;
                association.innovation_covariance = prediction_cov + measurement_cov;
            }
            
            associations.push_back(association);
        }
    }
    
    return associations;
}

double JPDA::calculatePairLikelihood(
    int track_id,
    int detection_id,
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    if (track_id < 0 || detection_id < 0 ||
        track_id >= static_cast<int>(tracks.size()) ||
        detection_id >= static_cast<int>(detections.size())) {
        return 1e-10;
    }
    
    const auto& track = tracks[track_id];
    const auto& detection = detections[detection_id];
    
    // Calculate innovation
    auto predicted_pos = track.predicted_state.getPosition();
    auto measured_pos = detection.position.toEigen();
    auto innovation = measured_pos - predicted_pos;
    
    // Calculate innovation covariance
    auto prediction_cov = track.predicted_state.covariance.block<3, 3>(0, 0);
    auto measurement_cov = detection.position_covariance;
    auto innovation_cov = prediction_cov + measurement_cov;
    
    // Calculate likelihood using multivariate Gaussian PDF
    double det = innovation_cov.determinant();
    if (det <= 0) {
        return 1e-10;
    }
    
    auto weighted_innovation = innovation_cov.inverse() * innovation;
    double mahalanobis_sq = innovation.transpose() * weighted_innovation;
    
    double normalization = 1.0 / (std::pow(2.0 * M_PI, 1.5) * std::sqrt(det));
    double likelihood = normalization * std::exp(-0.5 * mahalanobis_sq);
    
    return std::max(likelihood, 1e-10);
}

double JPDA::calculateClutterProbability(
    int detection_id,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    if (detection_id < 0 || detection_id >= static_cast<int>(detections.size())) {
        return 1e-10;
    }
    
    double clutter_density = config_.parameters.at("clutter_density");
    
    // Simple uniform clutter model
    // In practice, this could be more sophisticated based on sensor characteristics
    return clutter_density;
}

double JPDA::calculateMissedDetectionProbability(
    int track_id,
    const std::vector<common::Track>& tracks) const {
    
    if (track_id < 0 || track_id >= static_cast<int>(tracks.size())) {
        return 1.0;
    }
    
    // Probability of missed detection = 1 - detection probability
    return (1.0 - detection_probability_);
}

bool JPDA::isEventFeasible(
    const AssociationEvent& event,
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    std::vector<bool> detection_used(detections.size(), false);
    std::vector<bool> track_used(tracks.size(), false);
    
    // Check that each detection and track is used at most once
    for (const auto& assoc : event.associations) {
        int track_idx = assoc.first;
        int det_idx = assoc.second;
        
        if (track_idx >= 0 && det_idx >= 0) {
            if (track_idx >= static_cast<int>(tracks.size()) ||
                det_idx >= static_cast<int>(detections.size())) {
                return false; // Invalid indices
            }
            
            if (detection_used[det_idx] || track_used[track_idx]) {
                return false; // Multiple assignments
            }
            
            detection_used[det_idx] = true;
            track_used[track_idx] = true;
            
            // Check gate constraints
            if (!isWithinGate(tracks[track_idx], detections[det_idx], gate)) {
                return false;
            }
        }
    }
    
    return true;
}

void JPDA::pruneEvents(std::vector<AssociationEvent>& events, 
                      double probability_threshold) const {
    
    events.erase(
        std::remove_if(events.begin(), events.end(),
            [probability_threshold](const AssociationEvent& event) {
                return event.probability < probability_threshold;
            }),
        events.end()
    );
}

void JPDA::normalizeEventProbabilities(std::vector<AssociationEvent>& events) const {
    double total_probability = 0.0;
    
    for (const auto& event : events) {
        total_probability += event.probability;
    }
    
    if (total_probability > 1e-15) {
        for (auto& event : events) {
            event.probability /= total_probability;
        }
    }
}

// Interface method implementations
double JPDA::calculateAssociationCost(
    const common::Track& track,
    const common::Detection& detection,
    const interfaces::AssociationGate& gate) const {
    
    if (custom_distance_func_) {
        return custom_distance_func_(track, detection);
    }
    
    // For JPDA, cost is negative log-likelihood
    auto predicted_pos = track.predicted_state.getPosition();
    auto measured_pos = detection.position.toEigen();
    auto innovation = measured_pos - predicted_pos;
    
    auto prediction_cov = track.predicted_state.covariance.block<3, 3>(0, 0);
    auto measurement_cov = detection.position_covariance;
    auto innovation_cov = prediction_cov + measurement_cov;
    
    double det = innovation_cov.determinant();
    if (det <= 0) {
        return 1000.0; // High cost for invalid covariance
    }
    
    auto weighted_innovation = innovation_cov.inverse() * innovation;
    double mahalanobis_sq = innovation.transpose() * weighted_innovation;
    
    return mahalanobis_sq; // Use Mahalanobis distance as cost
}

bool JPDA::isWithinGate(
    const common::Track& track,
    const common::Detection& detection,
    const interfaces::AssociationGate& gate) const {
    
    // Check Mahalanobis distance gate
    if (gate.use_mahalanobis) {
        double mahal_dist = calculateMahalanobisDistance(track, detection);
        if (mahal_dist > gate.mahalanobis_threshold) {
            return false;
        }
    }
    
    // Check Euclidean distance gate
    double euclidean_dist = calculateEuclideanDistance(track, detection);
    if (euclidean_dist > gate.euclidean_threshold) {
        return false;
    }
    
    return true;
}

common::MatrixXd JPDA::buildCostMatrix(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    const size_t num_tracks = tracks.size();
    const size_t num_detections = detections.size();
    
    common::MatrixXd cost_matrix(num_tracks, num_detections);
    
    for (size_t i = 0; i < num_tracks; ++i) {
        for (size_t j = 0; j < num_detections; ++j) {
            cost_matrix(i, j) = calculateAssociationCost(tracks[i], detections[j], gate);
        }
    }
    
    return cost_matrix;
}

void JPDA::setTrackExistenceProbability(int track_id, double probability) {
    track_existence_probabilities_[track_id] = std::max(0.0, std::min(1.0, probability));
}

bool JPDA::configure(const common::AlgorithmConfig& config) {
    if (!validateConfiguration(config)) {
        return false;
    }
    
    config_ = config;
    false_alarm_rate_ = config_.parameters.at("false_alarm_rate");
    detection_probability_ = config_.parameters.at("detection_probability");
    gate_probability_ = config_.parameters.at("gate_probability");
    
    return true;
}

common::AlgorithmConfig JPDA::getConfiguration() const {
    return config_;
}

std::unordered_map<std::string, std::string> JPDA::getParameterDescriptions() const {
    return {
        {"false_alarm_rate", "False alarm rate per unit volume"},
        {"detection_probability", "Probability of detection for valid targets"},
        {"gate_probability", "Gate probability for validation"},
        {"max_hypotheses", "Maximum number of hypotheses to consider"},
        {"probability_threshold", "Minimum probability threshold for associations"},
        {"clutter_density", "Clutter density per unit volume"},
        {"track_existence_prob", "Default track existence probability"}
    };
}

bool JPDA::validateConfiguration(const common::AlgorithmConfig& config) const {
    std::vector<std::string> required_params = {
        "false_alarm_rate", "detection_probability", "gate_probability"
    };
    
    for (const auto& param : required_params) {
        if (config.parameters.find(param) == config.parameters.end()) {
            return false;
        }
    }
    
    double false_alarm_rate = config.parameters.at("false_alarm_rate");
    double detection_prob = config.parameters.at("detection_probability");
    double gate_prob = config.parameters.at("gate_probability");
    
    return false_alarm_rate >= 0.0 && 
           detection_prob >= 0.0 && detection_prob <= 1.0 &&
           gate_prob >= 0.0 && gate_prob <= 1.0;
}

common::AssociationAlgorithm JPDA::getAlgorithmType() const {
    return common::AssociationAlgorithm::JPDA;
}

std::string JPDA::getName() const {
    return "Joint Probabilistic Data Association";
}

std::string JPDA::getVersion() const {
    return "1.0.0";
}

void JPDA::reset() {
    performance_metrics_ = common::PerformanceMetrics{};
    track_existence_probabilities_.clear();
    custom_distance_func_ = nullptr;
}

common::PerformanceMetrics JPDA::getPerformanceMetrics() const {
    return performance_metrics_;
}

void JPDA::setDistanceFunction(
    std::function<double(const common::Track&, const common::Detection&)> distance_func) {
    custom_distance_func_ = distance_func;
}

} // namespace association
} // namespace processing
} // namespace radar