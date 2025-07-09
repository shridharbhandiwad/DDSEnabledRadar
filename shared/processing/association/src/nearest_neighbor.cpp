#include "association/nearest_neighbor.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace radar {
namespace processing {
namespace association {

NearestNeighbor::NearestNeighbor() {
    config_.name = "NearestNeighbor";
    config_.type = "NEAREST_NEIGHBOR";
    config_.parameters["distance_metric"] = 0.0;         // 0=euclidean, 1=mahalanobis
    config_.parameters["max_distance"] = 100.0;          // Maximum association distance
    config_.parameters["use_likelihood"] = 0.0;          // 0=false, 1=true (use likelihood for scoring)
    config_.parameters["priority_method"] = 0.0;         // 0=distance, 1=likelihood, 2=combined
    config_.enabled = true;
}

std::vector<common::Association> NearestNeighbor::associate(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::vector<common::Association> associations;
    
    if (tracks.empty() || detections.empty()) {
        return associations;
    }
    
    // Find all valid association candidates
    auto candidates = findValidCandidates(tracks, detections, gate);
    
    // Perform greedy assignment
    associations = performGreedyAssignment(candidates, tracks, detections, gate);
    
    // Update performance metrics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end_time - start_time);
    
    performance_metrics_.timestamp = end_time;
    performance_metrics_.processing_time_ms = duration.count();
    performance_metrics_.total_detections = detections.size();
    performance_metrics_.active_tracks = tracks.size();
    
    return associations;
}

std::vector<interfaces::AssociationHypothesis> NearestNeighbor::getHypotheses(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) {
    
    std::vector<interfaces::AssociationHypothesis> hypotheses;
    
    // For Nearest Neighbor, there's only one deterministic hypothesis
    auto associations = associate(tracks, detections, gate);
    
    if (!associations.empty()) {
        interfaces::AssociationHypothesis hypothesis;
        hypothesis.associations = associations;
        hypothesis.probability = 1.0;  // Deterministic result
        
        // Calculate total likelihood
        hypothesis.likelihood = 0.0;
        for (const auto& assoc : associations) {
            hypothesis.likelihood += std::log(std::max(assoc.likelihood, 1e-10));
        }
        
        hypothesis.is_feasible = true;
        hypotheses.push_back(hypothesis);
    }
    
    return hypotheses;
}

std::vector<NearestNeighbor::AssociationCandidate> NearestNeighbor::findValidCandidates(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    std::vector<AssociationCandidate> candidates;
    const double max_distance = config_.parameters.at("max_distance");
    const bool use_likelihood = static_cast<bool>(config_.parameters.at("use_likelihood"));
    
    for (int t_idx = 0; t_idx < static_cast<int>(tracks.size()); ++t_idx) {
        for (int d_idx = 0; d_idx < static_cast<int>(detections.size()); ++d_idx) {
            
            // Check if within gate
            if (!isWithinGate(tracks[t_idx], detections[d_idx], gate)) {
                continue;
            }
            
            // Calculate distance
            double distance = calculateAssociationCost(tracks[t_idx], detections[d_idx], gate);
            
            // Check distance threshold
            if (distance > max_distance) {
                continue;
            }
            
            // Calculate likelihood if requested
            double likelihood = 1.0;
            if (use_likelihood) {
                likelihood = calculateLikelihood(tracks[t_idx], detections[d_idx]);
            }
            
            // Create candidate
            candidates.emplace_back(t_idx, d_idx, distance, likelihood);
        }
    }
    
    return candidates;
}

std::vector<common::Association> NearestNeighbor::performGreedyAssignment(
    std::vector<AssociationCandidate>& candidates,
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    std::vector<common::Association> associations;
    
    if (candidates.empty()) {
        return associations;
    }
    
    // Sort candidates by priority (distance or likelihood)
    const int priority_method = static_cast<int>(config_.parameters.at("priority_method"));
    
    switch (priority_method) {
        case 0: // Distance-based (ascending)
            std::sort(candidates.begin(), candidates.end(),
                [](const AssociationCandidate& a, const AssociationCandidate& b) {
                    return a.distance < b.distance;
                });
            break;
            
        case 1: // Likelihood-based (descending)
            std::sort(candidates.begin(), candidates.end(),
                [](const AssociationCandidate& a, const AssociationCandidate& b) {
                    return a.likelihood > b.likelihood;
                });
            break;
            
        case 2: // Combined score (distance/likelihood ratio, ascending)
            std::sort(candidates.begin(), candidates.end(),
                [](const AssociationCandidate& a, const AssociationCandidate& b) {
                    double score_a = a.distance / std::max(a.likelihood, 1e-10);
                    double score_b = b.distance / std::max(b.likelihood, 1e-10);
                    return score_a < score_b;
                });
            break;
            
        default:
            sortCandidatesByDistance(candidates);
            break;
    }
    
    // Track which tracks and detections have been assigned
    std::vector<bool> assigned_tracks(tracks.size(), false);
    std::vector<bool> assigned_detections(detections.size(), false);
    
    // Greedy assignment
    for (const auto& candidate : candidates) {
        int t_idx = candidate.track_index;
        int d_idx = candidate.detection_index;
        
        // Check if already assigned
        if (assigned_tracks[t_idx] || assigned_detections[d_idx]) {
            continue;
        }
        
        // Create association
        auto association = createAssociation(candidate, tracks, detections, gate);
        associations.push_back(association);
        
        // Mark as assigned
        assigned_tracks[t_idx] = true;
        assigned_detections[d_idx] = true;
    }
    
    return associations;
}

common::Association NearestNeighbor::createAssociation(
    const AssociationCandidate& candidate,
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    common::Association association;
    
    const auto& track = tracks[candidate.track_index];
    const auto& detection = detections[candidate.detection_index];
    
    association.track_id = track.id;
    association.detection_id = detection.id;
    association.distance = candidate.distance;
    association.score = 1.0 / (1.0 + candidate.distance); // Convert distance to score
    association.likelihood = candidate.likelihood;
    association.is_valid = true;
    association.algorithm = common::AssociationAlgorithm::NEAREST_NEIGHBOR;
    
    // Calculate innovation
    association.innovation = calculateInnovation(track, detection);
    
    // Calculate innovation covariance
    association.innovation_covariance = calculateInnovationCovariance(track, detection);
    
    return association;
}

common::Vector3d NearestNeighbor::calculateInnovation(
    const common::Track& track,
    const common::Detection& detection) const {
    
    // Innovation = measurement - prediction
    auto predicted_pos = track.predicted_state.getPosition();
    auto measured_pos = detection.position.toEigen();
    
    return measured_pos - predicted_pos;
}

common::Matrix3d NearestNeighbor::calculateInnovationCovariance(
    const common::Track& track,
    const common::Detection& detection) const {
    
    // Innovation covariance = H * P * H^T + R
    // For position measurement, H is identity for position components
    auto prediction_cov = track.predicted_state.covariance.block<3, 3>(0, 0);
    auto measurement_cov = detection.position_covariance;
    
    return prediction_cov + measurement_cov;
}

double NearestNeighbor::calculateLikelihood(
    const common::Track& track,
    const common::Detection& detection) const {
    
    auto innovation = calculateInnovation(track, detection);
    auto innovation_cov = calculateInnovationCovariance(track, detection);
    
    // Calculate multivariate Gaussian likelihood
    double det = innovation_cov.determinant();
    if (det <= 0) {
        return 1e-10;
    }
    
    auto weighted_innovation = innovation_cov.inverse() * innovation;
    double mahalanobis_sq = innovation.transpose() * weighted_innovation;
    
    // Multivariate Gaussian PDF
    double normalization = 1.0 / (std::pow(2.0 * M_PI, 1.5) * std::sqrt(det));
    double likelihood = normalization * std::exp(-0.5 * mahalanobis_sq);
    
    return std::max(likelihood, 1e-10);
}

void NearestNeighbor::sortCandidatesByDistance(std::vector<AssociationCandidate>& candidates) const {
    std::sort(candidates.begin(), candidates.end(),
        [](const AssociationCandidate& a, const AssociationCandidate& b) {
            return a.distance < b.distance;
        });
}

void NearestNeighbor::removeConflicts(
    std::vector<AssociationCandidate>& candidates,
    const std::vector<bool>& assigned_tracks,
    const std::vector<bool>& assigned_detections) const {
    
    candidates.erase(
        std::remove_if(candidates.begin(), candidates.end(),
            [&](const AssociationCandidate& candidate) {
                return assigned_tracks[candidate.track_index] || 
                       assigned_detections[candidate.detection_index];
            }),
        candidates.end()
    );
}

double NearestNeighbor::calculateAssociationCost(
    const common::Track& track,
    const common::Detection& detection,
    const interfaces::AssociationGate& gate) const {
    
    if (custom_distance_func_) {
        return custom_distance_func_(track, detection);
    }
    
    const int distance_metric = static_cast<int>(config_.parameters.at("distance_metric"));
    
    switch (distance_metric) {
        case 0: // Euclidean distance
            return calculateEuclideanDistance(track, detection);
            
        case 1: // Mahalanobis distance
            return calculateMahalanobisDistance(track, detection);
            
        default:
            return calculateEuclideanDistance(track, detection);
    }
}

bool NearestNeighbor::isWithinGate(
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
    
    // Check velocity gate if enabled
    if (gate.use_velocity_gating) {
        auto track_velocity = track.current_state.getVelocity();
        auto detection_velocity = detection.velocity;
        
        double velocity_diff = std::sqrt(
            std::pow(track_velocity.x() - detection_velocity.x(), 2) +
            std::pow(track_velocity.y() - detection_velocity.y(), 2) +
            std::pow(track_velocity.z() - detection_velocity.z(), 2)
        );
        
        if (velocity_diff > gate.velocity_threshold) {
            return false;
        }
    }
    
    return true;
}

common::MatrixXd NearestNeighbor::buildCostMatrix(
    const std::vector<common::Track>& tracks,
    const std::vector<common::Detection>& detections,
    const interfaces::AssociationGate& gate) const {
    
    const size_t num_tracks = tracks.size();
    const size_t num_detections = detections.size();
    const double max_distance = config_.parameters.at("max_distance");
    
    common::MatrixXd cost_matrix(num_tracks, num_detections);
    
    for (size_t i = 0; i < num_tracks; ++i) {
        for (size_t j = 0; j < num_detections; ++j) {
            if (isWithinGate(tracks[i], detections[j], gate)) {
                cost_matrix(i, j) = calculateAssociationCost(tracks[i], detections[j], gate);
            } else {
                cost_matrix(i, j) = max_distance; // High cost for out-of-gate associations
            }
        }
    }
    
    return cost_matrix;
}

bool NearestNeighbor::configure(const common::AlgorithmConfig& config) {
    if (!validateConfiguration(config)) {
        return false;
    }
    
    config_ = config;
    return true;
}

common::AlgorithmConfig NearestNeighbor::getConfiguration() const {
    return config_;
}

std::unordered_map<std::string, std::string> NearestNeighbor::getParameterDescriptions() const {
    return {
        {"distance_metric", "Distance metric: 0=Euclidean, 1=Mahalanobis"},
        {"max_distance", "Maximum association distance threshold"},
        {"use_likelihood", "Use likelihood for scoring: 0=false, 1=true"},
        {"priority_method", "Priority method: 0=distance, 1=likelihood, 2=combined"}
    };
}

bool NearestNeighbor::validateConfiguration(const common::AlgorithmConfig& config) const {
    std::vector<std::string> required_params = {"distance_metric", "max_distance"};
    
    for (const auto& param : required_params) {
        if (config.parameters.find(param) == config.parameters.end()) {
            return false;
        }
    }
    
    double max_distance = config.parameters.at("max_distance");
    int distance_metric = static_cast<int>(config.parameters.at("distance_metric"));
    
    return max_distance > 0.0 && distance_metric >= 0 && distance_metric <= 1;
}

common::AssociationAlgorithm NearestNeighbor::getAlgorithmType() const {
    return common::AssociationAlgorithm::NEAREST_NEIGHBOR;
}

std::string NearestNeighbor::getName() const {
    return "Nearest Neighbor";
}

std::string NearestNeighbor::getVersion() const {
    return "1.0.0";
}

void NearestNeighbor::reset() {
    performance_metrics_ = common::PerformanceMetrics{};
    custom_distance_func_ = nullptr;
}

common::PerformanceMetrics NearestNeighbor::getPerformanceMetrics() const {
    return performance_metrics_;
}

void NearestNeighbor::setDistanceFunction(
    std::function<double(const common::Track&, const common::Detection&)> distance_func) {
    custom_distance_func_ = distance_func;
}

} // namespace association
} // namespace processing
} // namespace radar